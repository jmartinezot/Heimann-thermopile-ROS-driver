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

#include <dynamic_reconfigure/server.h>
#include <heiman/getvectorfromHTPApublishedConfig.h>

#include <sstream>

//#include "../include/HTPAdrivers/globals.h"

#include "math.h"
#include <heiman/vector_evidencias_HTPA.h>
#include <heiman/vector_evidencias_HTPA_con_imagen.h>

bool first = true;

ros::Publisher vectorfromHTPA_pub;
ros::Publisher HTPA_and_vector_pub;

int array_temperaturas[32][31];
float array_gaussians[32][31];

float gaussian_kernel[5][5] = {{1, 4, 7, 4, 1},{4, 16, 26, 16, 4},{7, 26, 41, 26, 7},{4, 16, 26, 16, 4},{1, 4, 7, 4, 1}};
float array_convolucionado[32][31];
float array_evidencias[32];

bool respect_launch_parameters = true;
// la termopila tiene un FOV de 38 grados

	double temperatura_mean;
	double temperatura_stddev;

float pdf_gaussian(float x, float media, float deviation)
{
  float x_standardized, exponential, first_term;
  x_standardized = (x - media) / deviation;
  exponential = exp(-0.5 * powf(x_standardized, 2.0));
  first_term = 1 / (deviation * sqrtf(2 * 3.14159265));
  ROS_INFO("pdf_gaussian %5.2f",first_term * exponential);	
  return first_term * exponential;
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

void config_callback(heiman::getvectorfromHTPApublishedConfig &config, uint32_t level) {
  if (respect_launch_parameters)
    {
      // solo se inicializan los parámetros una vez
      if (!ros::param::get("~/temperatura_mean", temperatura_mean)) temperatura_mean = config.temperatura_mean;  
      if (!ros::param::get("~/temperatura_stddev", temperatura_stddev)) temperatura_stddev = config.temperatura_stddev;  	
      respect_launch_parameters = false;
    }	
  else
    {
      temperatura_mean = config.temperatura_mean; 
      temperatura_stddev = config.temperatura_stddev;
    }
}


//void HTPAoutputCallback(const std_msgs::String::ConstPtr& msg)
void HTPAoutputCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg)
{
  
  
  heiman::vector_evidencias_HTPA msg_HTPA;
  heiman::vector_evidencias_HTPA_con_imagen msg_HTPA_con_imagen;
  
  int i,length;
  std::stringstream ss;	
  
  ROS_INFO("Entro al callback");
  
  length = msg->layout.dim[0].stride;
  //printf("Length = %d\n",length);
	// each set of 2 bytes is a temperature in Kelvin*10; first the low-byte, then the high-byte
  if (length == 1058)
    //if (first == true)
    // the first 529 bytes (0 .. 528)
    {
      for (i = 0; i < 529; i++)
	{
	  // analyze every two bytes (i*2 and i*2+1)
	  //int temp = (unsigned char)str[i*2] + ((unsigned char)str[i*2+1])*256;
	  int temp = msg->data[i*2] + (msg->data[i*2+1])*256;
	  //printf("first_byte: %d second_byte: %d temp: %d \n",(unsigned char)str[i*2], (unsigned char)str[i*2+1],temp);
	  int quotient32 = i / 32;
	  int remainder32 = i % 32;
	  bool odd = remainder32 % 2 == 1;
	  int pixel = quotient32 * 32 + remainder32 / 2;	
	  if (odd)
	    {
	      pixel += 16;
	    }
	  int pixel_i = pixel / 32;
	  int pixel_j = pixel % 32;
	  array_temperaturas[pixel_i][pixel_j] = temp; 
	}
      first = false;
    }
  //if (length == 99926)
  else
    // the last 463 bytes (529 .. 991)
    {
      for (i = 0; i < 463; i++)
	{
	  // analyze every two bytes (i*2 and i*2+1)
	  //int temp = (unsigned char)str[i*2] + ((unsigned char)str[i*2+1])*256;
	  int temp = msg->data[i*2] + (msg->data[i*2+1])*256;
	  int quotient32 = (i + 529)/ 32;
	  int remainder32 = (i + 529) % 32;
	  bool odd = remainder32 % 2 == 1;
	  int pixel = quotient32 * 32 + remainder32 / 2;	
	  if (odd)
	    {
	      pixel += 16;
	    }
	  //pixel += 529; // this is the second part, the first 529 bytes have been already written down
	  //printf("Pixel: %d\n", pixel);
	  int pixel_i = pixel / 32;
	  int pixel_j = pixel % 32;	
	  array_temperaturas[pixel_i][pixel_j] = temp; 
	}
      first = true;
    }
  //ROS_INFO("Termino lo de leer HTPAoutput");
  // ahora calculamos las probabilidades; en principio pasamos de que hayamos leído las dos partes o no: siempre habrá algo
  
  //if (!ros::param::get("~/temperatura_mean", temperatura_mean)) temperatura_mean = 32.0;  
  //if (!ros::param::get("~/temperatura_stddev", temperatura_stddev)) temperatura_stddev = 2.0;  	
  
  for (int i = 0; i < 32; i ++)
    for (int j = 0; j < 31; j++)
      array_gaussians[i][j] = 
	pdf_gaussian((float)array_temperaturas[i][j] / 10.0 - 273.0, (float)temperatura_mean, (float)temperatura_stddev) /
	pdf_gaussian((float)temperatura_mean, (float)temperatura_mean, (float)temperatura_stddev);
  //ROS_INFO("Termino array_gaussians");	
  
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
      
      msg_HTPA_con_imagen.vector_evidencia[i] = array_evidencias[i];
      msg_HTPA_con_imagen.vector_angulos_grados[i] = -19 + (1.1875 / 2) + i * 1.1875;
      msg_HTPA_con_imagen.vector_angulos_radianes[i] = msg_HTPA.vector_angulos_grados[i] * 2 * 3.14159265 / 360;
      
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
  
  
  msg_HTPA_con_imagen.valor_maximo_evidencia = max_evidencia;
  msg_HTPA_con_imagen.indice_maximo_evidencia = indice_max_evidencia;
  msg_HTPA_con_imagen.grados_maximo_evidencia = msg_HTPA_con_imagen.vector_angulos_grados[indice_max_evidencia];
  msg_HTPA_con_imagen.radianes_maximo_evidencia = msg_HTPA_con_imagen.vector_angulos_radianes[indice_max_evidencia];
  msg_HTPA_con_imagen.temperatura_maximo_evidencia = temperatura_maxima_probabilidad;	
  msg_HTPA_con_imagen.gaussian_temperatura_maximo_evidencia = 
    pdf_gaussian(temperatura_maxima_probabilidad, (float)temperatura_mean, (float)temperatura_stddev) /
    pdf_gaussian((float)temperatura_mean, (float)temperatura_mean, (float)temperatura_stddev);
  
  msg_HTPA_con_imagen.imagen = *msg;		
  
  vectorfromHTPA_pub.publish(msg_HTPA);
  
  HTPA_and_vector_pub.publish(msg_HTPA_con_imagen);
  
  
  
  ROS_INFO("Salgo del callback");
}


int 
main(int argc, char **argv)
{
  ros::init(argc, argv, "getvectorfromHTPApublished");
  ros::NodeHandle n;
  
  vectorfromHTPA_pub = n.advertise<heiman::vector_evidencias_HTPA>("vector_evidencias", 1);
  HTPA_and_vector_pub = n.advertise<heiman::vector_evidencias_HTPA_con_imagen>("vector_evidencias_con_imagen", 1);
  
  ros::Subscriber sub = n.subscribe("HTPAoutput", 0, HTPAoutputCallback);
  
  dynamic_reconfigure::Server<heiman::getvectorfromHTPApublishedConfig> server;
  dynamic_reconfigure::Server<heiman::getvectorfromHTPApublishedConfig>::CallbackType f;
  
  f = boost::bind(&config_callback, _1, _2);
  server.setCallback(f);      
  
  ros::Rate loop_rate(10);
  
  while (ros::ok())
    {	
      
      ros::spinOnce();
      
      loop_rate.sleep();
    }
  
  
  return 0;
}

