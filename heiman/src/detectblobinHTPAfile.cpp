#include "ros/ros.h"
//#include "std_msgs/String.h"
//#include "std_msgs/UInt8MultiArray.h"
//#include "sensor_msgs/Image.h"
//#include <sys/socket.h>
//#include <sys/types.h>
//#include <sys/times.h>
//#include <netinet/in.h>
//#include <arpa/inet.h>
//#include <netdb.h>
//#include <unistd.h>

//#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
//#include <cv_bridge/CvBridge.h>


#include <sstream>

//#include "../include/HTPAdrivers/globals.h"

#include "math.h"
//#include "HTPAdrivers/vector_evidencias_HTPA.h"


int array_temperaturas[32][31];
float array_gaussians[32][31];
int array_binarizado[32][31];
int array_binarizado_grande[320][310];

float gaussian_kernel[5][5] = {{1, 4, 7, 4, 1},{4, 16, 26, 16, 4},{7, 26, 41, 26, 7},{4, 16, 26, 16, 4},{1, 4, 7, 4, 1}};
float array_convolucionado[32][31];

double temperatura_mean;
double temperatura_stddev;
double temperatura_limite_inferior_zona_optima;
double temperatura_limite_superior_zona_optima;
double temperatura_exponente_penalizacion;
int umbral_binarizacion;

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

//void HTPAoutputCallback(const std_msgs::String::ConstPtr& msg)
void HTPAoutputFile()
{

	std::stringstream ss;	
	
//	ROS_INFO("Entro al callback");

	//ROS_INFO("Termino lo de leer HTPAoutput");
	// ahora calculamos las probabilidades; en principio pasamos de que hayamos leído las dos partes o no: siempre habrá algo

	if (!ros::param::get("~/temperatura_mean", temperatura_mean)) temperatura_mean = 30.0;  
	if (!ros::param::get("~/temperatura_stddev", temperatura_stddev)) temperatura_stddev = 2.0;  	
	if (!ros::param::get("~/temperatura_limite_inferior_zona_optima", temperatura_limite_inferior_zona_optima)) 
		temperatura_limite_inferior_zona_optima = 29.0;  
	if (!ros::param::get("~/temperatura_limite_superior_zona_optima", temperatura_limite_superior_zona_optima)) 
		temperatura_limite_superior_zona_optima = 30.0;  	
	if (!ros::param::get("~/temperatura_exponente_penalizacion", temperatura_exponente_penalizacion)) 
		temperatura_exponente_penalizacion = 5.5;  	
	if (!ros::param::get("~/umbral_binarizacion", umbral_binarizacion)) 
		umbral_binarizacion = 175;		
	
	compute_gaussians((float)temperatura_limite_inferior_zona_optima, 
			(float)temperatura_limite_superior_zona_optima, (float)temperatura_exponente_penalizacion);	
			
	convolute_gaussian(array_gaussians, array_convolucionado);	
	//ROS_INFO("Termino convolute");
	
	
//	ROS_INFO("Salgo del callback");
}

void un_fichero(char* nombre)
{
IplImage* HTPAimage = cvCreateImage(cvSize(320,310),IPL_DEPTH_8U,1);	

	ficheroHTPA = fopen(nombre,"r");
	for (int j = 0; j < 31; j++)
		for (int i = 0; i < 32; i++)
		{
			fscanf(ficheroHTPA,"%d",&array_temperaturas[i][j]);
		}
	fclose(ficheroHTPA);
	HTPAoutputFile();
	// escalamos array_gaussians a 0 .. 255
	for (int i = 0; i < 32; i++)
		for (int j = 0; j < 31; j++)
			//array_binarizado[i][j] = (int)(array_convolucionado[i][j] * 255);
			array_binarizado[i][j] = (int)(array_gaussians[i][j] * 255);
	ROS_INFO("fsdfds");
	// binarizamos
	for (int i = 0; i < 32; i++)
		for (int j = 0; j < 31; j++)
			if (array_binarizado[i][j] > umbral_binarizacion)
			//if (array_temperaturas[i][j] > 3030) // 30 grados
				array_binarizado[i][j] = 255;
			else array_binarizado[i][j] = 0;
	ROS_INFO("fsdfds");
	// copiar al array grande (10 veces mayor en cada dimension)
	for (int i = 0; i < 32; i++)
		for (int j = 0; j < 31; j++)
			{
				for (int k1 = i * 10; k1 < (i+1) * 10; k1 ++)
					for (int k2 = j * 10; k2 < (j+1) * 10; k2++)
						array_binarizado_grande[k1][k2] = array_binarizado[i][j];
			}
	ROS_INFO("fsdfds");
	// rellenamos la imagen
	for (int m1 = 0; m1 < 320; m1++)
		for (int m2 = 0; m2 < 310; m2++)
		{
			//ROS_INFO("m1 %d m2 %d array_binarizado_grande %d",m1,m2,array_binarizado_grande[m1][m2]);
			//((uchar *)(HTPAimage->imageData + m1*HTPAimage->widthStep))[m2] = (uchar*)array_binarizado_grande[m1][m2];
			CvScalar s;
			//s=cvGet2D(img,i,j); // get the (i,j) pixel value
			//printf("intensity=%f\n",s.val[0]);
			s.val[0]=array_binarizado_grande[m1][m2];
			cvSet2D(HTPAimage,m2,m1,s); // set the (i,j) pixel value
		}
	ROS_INFO("fsdfds");				
	//escribimos la imagen a disco
	char outFileName[1000];
	sprintf(outFileName,"%s.bmp",nombre);
	cvSaveImage(outFileName,HTPAimage);
	ROS_INFO("fsdfds");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "getvectorfromHTPAfile");
	ros::NodeHandle n;


	//FILE* ficheroHTPA;
	//ROS_INFO("dfsd");
/*	ficheroHTPA = fopen(argv[1],"r");
	//ROS_INFO("dfsd");
	for (int i = 0; i < 32; i++)
		for (int j = 0; j < 31; j++)
		{
			//int k;
			//ROS_INFO("dfsd");
			fscanf(ficheroHTPA,"%d",&array_temperaturas[i][j]);
			//fscanf(ficheroHTPA,"%d",&k);
		}
	fclose(ficheroHTPA);*/
	
// NUEVO

un_fichero("/home/jmmartinez/Desktop/datos_temperatura/BAI_HTPA8390_1318851328.00KINECT1.txt");
un_fichero("/home/jmmartinez/Desktop/datos_temperatura/BAI_HTPA8420_1318851328.00KINECT1.txt");
un_fichero("/home/jmmartinez/Desktop/datos_temperatura/BAI_HTPA8430_1318851328.00KINECT1.txt");
un_fichero("/home/jmmartinez/Desktop/datos_temperatura/BAI_HTPA8550_1318851328.00KINECT1.txt");
un_fichero("/home/jmmartinez/Desktop/datos_temperatura/BAI_HTPA8580_1318851328.00KINECT1.txt");
un_fichero("/home/jmmartinez/Desktop/datos_temperatura/BAI_HTPA8680_1318851328.00KINECT1.txt");
un_fichero("/home/jmmartinez/Desktop/datos_temperatura/BAI_HTPA8970_1318851328.00KINECT1.txt");
un_fichero("/home/jmmartinez/Desktop/datos_temperatura/EZ_HTPA5510_1318851328.00KINECT0.txt");
un_fichero("/home/jmmartinez/Desktop/datos_temperatura/EZ_HTPA6000_1318851328.00KINECT1.txt");
un_fichero("/home/jmmartinez/Desktop/datos_temperatura/EZ_HTPA7910_1318851328.00KINECT0.txt");

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

