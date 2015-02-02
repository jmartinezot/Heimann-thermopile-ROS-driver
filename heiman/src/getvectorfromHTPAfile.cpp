#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8MultiArray.h"
#include "sensor_msgs/Image.h"
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/times.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>

#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>


#include <sstream>

//#include "../include/HTPAdrivers/globals.h"

#include "math.h"
#include "heiman/vector_evidencias_HTPA.h"

bool first = true;

ros::Publisher vectorfromHTPA_pub;

int array_temperaturas[32][31];
float array_gaussians[32][31];

float gaussian_kernel[5][5] = {{1, 4, 7, 4, 1},{4, 16, 26, 16, 4},{7, 26, 41, 26, 7},{4, 16, 26, 16, 4},{1, 4, 7, 4, 1}};
float array_convolucionado[32][31];
float array_evidencias[32];

double temperatura_mean;
double temperatura_stddev;
double temperatura_limite_inferior_zona_optima;
double temperatura_limite_superior_zona_optima;
double temperatura_exponente_penalizacion;

FILE* ficheroHTPA;

// la termopila tiene un FOV de 38 grados

float pdf_gaussian(float x, float media, float deviation)
{
  float x_standardized, exponential, first_term;
  x_standardized = (x - media) / deviation;
  exponential = exp(-0.5 * powf(x_standardized, 2.0));
  first_term = 1 / (deviation * sqrtf(2 * 3.14159265));
  //ROS_INFO("pdf_gaussian %5.2f",first_term * exponential);	
  return first_term * exponential;
}

// creamos array_gaussians a partir de las temperaturas en decimas de Kelvin
// tenemos limite_inferior y limite_superior para definir la zona donde la probabilidad sera maxima
// tenemos exponente para ver a que elevamos la cola superior para que su probabilidad sea menor
// suponemos que la media es igual al limite_superior

void compute_gaussians(float limite_inferior, float limite_superior, float exponente)
{
  for (int i = 0; i < 32; i++)
    for (int j = 0; j < 31; j++)
      {
	float temp, gaussian_temp;
	temp = (float)array_temperaturas[i][j] / 10.0 - 273.0;
	// si esta en el intervalo optimo se asigna al limite_superior, porque es la media
	if (temp >= limite_inferior && temp <= limite_superior)
	  temp = limite_superior;
	// si es menor que el limite_inferior, se le acerca al limite_superior;
	// es como si se comprimiera el intervalo
	else if (temp < limite_inferior)
	  temp += (limite_superior - limite_inferior);
	gaussian_temp = pdf_gaussian(temp, (float)temperatura_mean, (float)temperatura_stddev) /
	  pdf_gaussian((float)temperatura_mean, (float)temperatura_mean, (float)temperatura_stddev);
	// se penaliza a los valores por encima de limite_superior, tanto mas cuanto mas alejados esten de dicho limite
	if (temp > limite_superior)
	  gaussian_temp = powf(gaussian_temp, exponente * (temp / limite_superior));
	array_gaussians[i][j] = gaussian_temp;
      }
}

// temperaturas es 32 x 31, x e y son las coordenadas del elemento
// el centro es (2,2); hay que ir de -2 a +2 en las dos dimensiones, teniendo cuidado
// de que no sea negativo o mayor o igual a 32 en x o a 31 en y
float convolute_element(float temperaturas[32][31], int x, int y)
{
  float sum = 0;
  float denominador = 0;
  for (int i = -2; i <= 2; i++)
    for (int j = -2; j <= 2; j++)
      {
	//ROS_INFO("x:%d y:%d i:%d j:%d",x,y,i,j);
	int coord_x_actual = x + i;
	int coord_y_actual = y + j;
	// solo sumamos las coordenadas que quedan dentro de la matriz
	if (coord_x_actual >= 0 && coord_y_actual >= 0 && coord_x_actual < 32 && coord_y_actual < 31)
	  {
	    sum += (temperaturas[x+i][y+j] * gaussian_kernel[2+i][2+j]);
	    denominador += gaussian_kernel[2+i][2+j];
	  }
      }
  return sum/denominador;
}

// temperaturas y resultado son 32 x 31
void convolute_gaussian(float temperaturas[32][31], float resultado[32][31])
{
  for (int i = 0; i < 32; i++)
    for (int j = 0; j < 31; j++)
      {
	//ROS_INFO("i:%d j:%d",i,j);
	resultado[i][j] = convolute_element(temperaturas,i,j);
      }
}

void extract_bigger(float convolucionado[32][31], float evidencias[32], float &temperatura)
{
  for (int i = 0; i < 32; i ++)
    {
      float max = -274.0;
      for (int j = 0; j < 31; j++)
	{
	  if (convolucionado[i][j] > max)
	    {
	      max = convolucionado[i][j];
	      temperatura = (float)array_temperaturas[i][j] / 10.0 - 273.0;
	    }
	}
      evidencias[i] = max;
    }
}

//void HTPAoutputCallback(const std_msgs::String::ConstPtr& msg)
void HTPAoutputFile()
{
  
  
  
  heiman::vector_evidencias_HTPA msg_HTPA;
  
  std::stringstream ss;	
  
  //	ROS_INFO("Entro al callback");
  
  //ROS_INFO("Termino lo de leer HTPAoutput");
  // ahora calculamos las probabilidades; en principio pasamos de que hayamos leído las dos partes o no: siempre habrá algo
  
  if (!ros::param::get("~/temperatura_mean", temperatura_mean)) temperatura_mean = 32.0;  
  if (!ros::param::get("~/temperatura_stddev", temperatura_stddev)) temperatura_stddev = 2.0;  	
  if (!ros::param::get("~/temperatura_limite_inferior_zona_optima", temperatura_limite_inferior_zona_optima)) 
    temperatura_limite_inferior_zona_optima = 29.0;  
  if (!ros::param::get("~/temperatura_limite_superior_zona_optima", temperatura_limite_superior_zona_optima)) 
    temperatura_limite_superior_zona_optima = 30.0;  	
  if (!ros::param::get("~/temperatura_exponente_penalizacion", temperatura_exponente_penalizacion)) 
    temperatura_exponente_penalizacion = 5.5;  	
  
  compute_gaussians((float)temperatura_limite_inferior_zona_optima, 
		    (float)temperatura_limite_superior_zona_optima, (float)temperatura_exponente_penalizacion);	
  
  convolute_gaussian(array_gaussians, array_convolucionado);	
  //ROS_INFO("Termino convolute");
  float temperatura_maxima_probabilidad;	
  extract_bigger(array_convolucionado, array_evidencias, temperatura_maxima_probabilidad);
  //ROS_INFO("Termino extract_bigger");
  
  float max_evidencia = -1.0;
  int indice_max_evidencia = 0;
  for (int i = 0; i < 32; i++)
    {
      // la termopila tiene un FOV de 38 grados, es decir, de -19 a +19, que queda en 1.1875 grados por elemento;
      // empezamos en -19 + (1.1875 / 2) y vamos sumando 1.1875
      msg_HTPA.vector_evidencia[i] = array_evidencias[i];
      msg_HTPA.vector_angulos_grados[i] = -19 + (1.1875 / 2) + i * 1.1875;
      msg_HTPA.vector_angulos_radianes[i] = msg_HTPA.vector_angulos_grados[i] * 2 * 3.14159265 / 360;
      if (array_evidencias[i] > max_evidencia) 
	{
	  max_evidencia = array_evidencias[i];
	  indice_max_evidencia = i;
	}
    }
  msg_HTPA.valor_maximo_evidencia = max_evidencia;
  msg_HTPA.indice_maximo_evidencia = indice_max_evidencia;
  msg_HTPA.grados_maximo_evidencia = msg_HTPA.vector_angulos_grados[indice_max_evidencia];
  msg_HTPA.radianes_maximo_evidencia = msg_HTPA.vector_angulos_radianes[indice_max_evidencia];
  msg_HTPA.temperatura_maximo_evidencia = temperatura_maxima_probabilidad;
  msg_HTPA.gaussian_temperatura_maximo_evidencia = 
    pdf_gaussian(temperatura_maxima_probabilidad, (float)temperatura_mean, (float)temperatura_stddev) /
    pdf_gaussian((float)temperatura_mean, (float)temperatura_mean, (float)temperatura_stddev);
  
  ROS_INFO("valor: %5.2f indice: %d temperatura %5.2f",max_evidencia,indice_max_evidencia,temperatura_maxima_probabilidad);
  
  vectorfromHTPA_pub.publish(msg_HTPA);
  
  //	ROS_INFO("Salgo del callback");
}

void un_fichero(char* nombre)
{
  ficheroHTPA = fopen(nombre,"r");
  for (int i = 0; i < 32; i++)
    for (int j = 0; j < 31; j++)
      {
	fscanf(ficheroHTPA,"%d",&array_temperaturas[i][j]);
      }
  fclose(ficheroHTPA);
  HTPAoutputFile();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "getvectorfromHTPAfile");
  ros::NodeHandle n;
  
  vectorfromHTPA_pub = n.advertise<heiman::vector_evidencias_HTPA>("vector_evidencias", 1);
  
  //FILE* ficheroHTPA;
  //ROS_INFO("dfsd");
  ficheroHTPA = fopen(argv[1],"r");
  //ROS_INFO("dfsd");
  for (int i = 0; i < 32; i++)
    for (int j = 0; j < 31; j++)
      {
	//int k;
	//ROS_INFO("dfsd");
	fscanf(ficheroHTPA,"%d",&array_temperaturas[i][j]);
	//fscanf(ficheroHTPA,"%d",&k);
      }
  fclose(ficheroHTPA);
  
  // NUEVO
  
  un_fichero("/home/bee/Desktop/datos_temperatura/BAI_HTPA8390_1318851328.00KINECT1.txt");
  un_fichero("/home/bee/Desktop/datos_temperatura/BAI_HTPA8420_1318851328.00KINECT1.txt");
  un_fichero("/home/bee/Desktop/datos_temperatura/BAI_HTPA8430_1318851328.00KINECT1.txt");
  un_fichero("/home/bee/Desktop/datos_temperatura/BAI_HTPA8550_1318851328.00KINECT1.txt");
  un_fichero("/home/bee/Desktop/datos_temperatura/BAI_HTPA8580_1318851328.00KINECT1.txt");
  un_fichero("/home/bee/Desktop/datos_temperatura/BAI_HTPA8680_1318851328.00KINECT1.txt");
  un_fichero("/home/bee/Desktop/datos_temperatura/BAI_HTPA8970_1318851328.00KINECT1.txt");
  un_fichero("/home/bee/Desktop/datos_temperatura/EZ_HTPA5510_1318851328.00KINECT0.txt");
  un_fichero("/home/bee/Desktop/datos_temperatura/EZ_HTPA6000_1318851328.00KINECT1.txt");
  un_fichero("/home/bee/Desktop/datos_temperatura/EZ_HTPA7910_1318851328.00KINECT0.txt");
  
  // FIN NUEVO
  
  ros::Rate loop_rate(10);
  
  while (ros::ok())
    {	
      //HTPAoutputFile();
      ros::spinOnce();
      
      loop_rate.sleep();
    }
  
  
  return 0;
}

