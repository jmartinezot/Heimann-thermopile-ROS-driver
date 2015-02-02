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
// Eli
//#include <cv_bridge/CvBridge.h>
#include <cv_bridge/cv_bridge.h>
#include  <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
//------------------------------------------
#include <dynamic_reconfigure/server.h>
#include <heiman/getallfromHTPApublishedConfig.h>

#include <sstream>

//#include "../include/HTPAdrivers/globals.h"

#include "math.h"
#include <heiman/vector_evidencias_HTPA.h>

#include <heiman/temperatures.h>

// en este nodo, a partir del topic HTPAprocessed, vamos a publicar
// - "vector_evidencias" (32 floats); para su cálculo tendremos en cuenta los parámetros "standard_mean" y "standard_deviation"
// - "color_temp_image" (imagen en falso color); para su cálculo tendremos en cuenta "lower_temp_limit", "upper_temp_limit" y "zoom_temp"
// - "binary_temp_image" (imagen binarizada); calculada aplicando el parámetro "threshold_temp" a los valores de temperatura (Celsius)
// - "blob_temp_image" (imagen binarizada); calculada aplicando el parámetro "min_size_temp" a los blobs de "binary_temp_image"
// - "color_evidence_image" (imagen en falso color); calculada a partir de "lower_evidence_limit", "upper_evidence_limit" y "zoom_evidence"
// - "binary_evidence_image" (imagen binarizada); calculada aplicando el parámetro "threshold_evidence" a los valores de evidencia (entre 0 y 1)
// - "blob_evidence_image" (imagen binarizada); calculada aplicando el parámetro "min_size_evidence" a los blobs de "binary_evidence_image"


bool first = true;

ros::Publisher vectorfromHTPA_pub;

image_transport::Publisher color_temp_image_pub;	
image_transport::Publisher binary_temp_image_pub;
image_transport::Publisher blob_temp_image_pub;
image_transport::Publisher color_evidence_image_pub;
image_transport::Publisher binary_evidence_image_pub;
image_transport::Publisher blob_evidence_image_pub; 	

int max_blobs;
CvRect r;
CvMemStorage* storage; 
CvSeq *contours;
int blob_area;

// Eli
using namespace cv;
using namespace std;
// -------
void remove_small_and_big_blobs(IplImage* source, int minimumArea, int maximumArea)
{  
  vector<vector <Point> >mycontours;
  vector<Vec4i> hierarchy;
  RNG rng(12345);
  Mat src_gray;

	// Eli	
	int thres = 100;
	int max_thres = 255;
	int i;
	Mat src = cvarrToMat(source);
	Mat canny_out;
	//cvCopy(source, imageCopy);
	
	cvtColor( src, src_gray, CV_BGR2GRAY );
	blur( src_gray, src_gray, Size(3,3) );
	Canny( src_gray, canny_out, thres, thres*2, 3);
	findContours(canny_out, mycontours, hierarchy, CV_RETR_TREE,
		     CV_CHAIN_APPROX_SIMPLE, Point(0,0));
	//cvFindContours(imageCopy,storage,&contours,sizeof(CvContour),CV_RETR_EXTERNAL);
	//-----
	Mat drawing = Mat::zeros( canny_out.size(), CV_8UC3 );
	  for (int i = 0; i < mycontours.size(); i++)
	    {
	      Scalar color = Scalar( rng.uniform(0, 255), 
				     rng.uniform(0,255), 
				     rng.uniform(0,255) );
	      drawContours( drawing, mycontours, i, color, 2, 8, 
			    hierarchy, 0, Point() );


	    }

// // looks for blobs bigger than minimumArea
// //	IplImage *imageCopy;
// 	int i;
// 	cvCopy(source,imageCopy);
// 	cvFindContours(imageCopy,storage,&contours,sizeof(CvContour),CV_RETR_EXTERNAL);

// 	if(contours==0)return;

// 	i = 0;

// 	for(;contours;contours=contours->h_next)
// 	{

// 		//if(i==max_blobs)return i;

// 		r = cvBoundingRect( contours );
		
// 		if(r.height*r.width < minimumArea || r.height*r.width > maximumArea)
// 		{
// 			// se borra de la imagen			
// 			cvDrawContours( source, contours, cvScalar(0), cvScalar(0), 0, CV_FILLED);
// //			i++;
// 		}

// 	}
// 	cvReleaseImage(&imageCopy);
}



int array_temperaturas[32][31];
float array_gaussians[32][31];

float gaussian_kernel[5][5] = {{1, 4, 7, 4, 1},{4, 16, 26, 16, 4},{7, 26, 41, 26, 7},{4, 16, 26, 16, 4},{1, 4, 7, 4, 1}};
float array_convolucionado[32][31];
float array_evidencias[32];

bool respect_launch_parameters = true;
// la termopila tiene un FOV de 38 grados

	double temperatura_mean;
	double temperatura_stddev;
	double lower_temp_limit, upper_temp_limit;
	int zoom_temp;
	double lower_evidence_limit, upper_evidence_limit;
	int zoom_evidence;	
	double threshold_temp, threshold_evidence;
	int min_size_temp, min_size_evidence;
	int max_size_temp, max_size_evidence;

float pdf_gaussian(float x, float media, float deviation)
{
	float x_standardized, exponential, first_term;
	x_standardized = (x - media) / deviation;
	exponential = exp(-0.5 * powf(x_standardized, 2.0));
	first_term = 1 / (deviation * sqrtf(2 * 3.14159265));
	//ROS_INFO("pdf_gaussian %5.2f",first_term * exponential);	
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

void convertdatatoRGB(float data, int &R, int &G, int &B, float lower, float upper)
// convierte datos en RGB; menor que lower será totalmente azul, mayor que upper será totalmente rojo,
// y los datos intermedios tendrán un color intermedio 
	{
		if (data < lower)
			{ R = 0; G = 0; B = 255; return;}
		if (data > upper)
			{ R = 255; G = 0; B = 0; return;}
		float width = (float)(upper - lower);
		double percentage_hot = (double)(data - lower) / (double)width;
		double percentage_cold = 1.0 - percentage_hot;
		{
			R = (int)(percentage_hot * 255);
			G = 0;
			B = (int)(percentage_cold * 255);
		}	
		return;
	}

void convertTtoRGB(int T, int &R, int &G, int &B)
// convert a tempeature in Kelvin*10 into a RGB value
// the more blue the colder, the more red the hotter; lower_limit_kelvin_decimals = 2980 (273K + 25); 
// upper_limit_kelvin_decimals = 3180 (273K + 45) 
	{
		int lower_limit_kelvin_decimals = (lower_temp_limit + 273) * 10;
		int upper_limit_kelvin_decimals = (upper_temp_limit + 273) * 10;		
		convertdatatoRGB(T, R, G, B, lower_limit_kelvin_decimals, upper_limit_kelvin_decimals);
	}

void create_binary_temp_image(int data[32][31], float threshold, int zoom, IplImage &image)
{
	int pixel_value;
	for (int pixel_i = 0; pixel_i < 31; pixel_i++)
		for (int pixel_j = 0; pixel_j < 32; pixel_j++)
		{
			if (data[pixel_i][pixel_j] < threshold) 
				pixel_value = 0;
			else pixel_value = 255;
			for (int zoom_i = 0; zoom_i < zoom; zoom_i++)
				for (int zoom_j = 0; zoom_j < zoom; zoom_j++)
					{
	((uchar *)(image.imageData + (pixel_i*zoom+zoom_i)*image.widthStep))[(pixel_j*zoom+zoom_j)*image.nChannels + 0] = pixel_value;			
					}			
		}			
}

void create_binary_evidence_image(float data[32][31], float threshold, int zoom, IplImage &image)
{
	int pixel_value;
	for (int pixel_i = 0; pixel_i < 31; pixel_i++)
		for (int pixel_j = 0; pixel_j < 32; pixel_j++)
		{
			if (data[pixel_i][pixel_j] < threshold) 
				pixel_value = 0;
			else pixel_value = 255;
			for (int zoom_i = 0; zoom_i < zoom; zoom_i++)
				for (int zoom_j = 0; zoom_j < zoom; zoom_j++)
					{
	((uchar *)(image.imageData + (pixel_i*zoom+zoom_i)*image.widthStep))[(pixel_j*zoom+zoom_j)*image.nChannels + 0] = pixel_value;			
					}			
		}			
}

void create_color_temp_image(int temperatura[32][31], IplImage &image)
{
	int R, G, B;
	for (int pixel_i = 0; pixel_i < 31; pixel_i++)
		for (int pixel_j = 0; pixel_j < 32; pixel_j++)
		{
			convertTtoRGB(temperatura[pixel_i][pixel_j], R, G, B);
			//ROS_INFO("pixel_i %d pixel_j %d R %d G %d B %d",pixel_i,pixel_j,R,G,B);
			for (int zoom_i = 0; zoom_i < zoom_temp; zoom_i++)
				for (int zoom_j = 0; zoom_j < zoom_temp; zoom_j++)
					{
			((uchar *)(image.imageData + (pixel_i*zoom_temp+zoom_i)*image.widthStep))[(pixel_j*zoom_temp+zoom_j)*image.nChannels + 0] = B;			
			((uchar *)(image.imageData + (pixel_i*zoom_temp+zoom_i)*image.widthStep))[(pixel_j*zoom_temp+zoom_j)*image.nChannels + 1] = G;			
			((uchar *)(image.imageData + (pixel_i*zoom_temp+zoom_i)*image.widthStep))[(pixel_j*zoom_temp+zoom_j)*image.nChannels + 2] = R;	
					}			
		}			
}

void create_color_evidence_image(float evidencia[32][31], IplImage &image)
{
	int R, G, B;
	for (int pixel_i = 0; pixel_i < 31; pixel_i++)
		for (int pixel_j = 0; pixel_j < 32; pixel_j++)
		{
			//ROS_INFO("pixel_i %d pixel_j %d R %d G %d B %d",pixel_i,pixel_j,R,G,B);
			convertdatatoRGB(evidencia[pixel_i][pixel_j], R, G, B, lower_evidence_limit, upper_evidence_limit);
			for (int zoom_i = 0; zoom_i < zoom_evidence; zoom_i++)
				for (int zoom_j = 0; zoom_j < zoom_evidence; zoom_j++)
					{
			((uchar *)(image.imageData + (pixel_i*zoom_evidence+zoom_i)*image.widthStep))[(pixel_j*zoom_evidence+zoom_j)*image.nChannels + 0] = B;			
			((uchar *)(image.imageData + (pixel_i*zoom_evidence+zoom_i)*image.widthStep))[(pixel_j*zoom_evidence+zoom_j)*image.nChannels + 1] = G;			
			((uchar *)(image.imageData + (pixel_i*zoom_evidence+zoom_i)*image.widthStep))[(pixel_j*zoom_evidence+zoom_j)*image.nChannels + 2] = R;	
					}			
		}			
}



void config_callback(heiman::getallfromHTPApublishedConfig &config, uint32_t level) {
	if (respect_launch_parameters)
	{
		  // solo se inicializan los parámetros una vez
		if (!ros::param::get("~/temperatura_mean", temperatura_mean)) temperatura_mean = config.temperatura_mean;  
		if (!ros::param::get("~/temperatura_stddev", temperatura_stddev)) temperatura_stddev = config.temperatura_stddev;  	
		if (!ros::param::get("~/zoom_temp", zoom_temp)) zoom_temp = config.zoom_temp;  	
		if (!ros::param::get("~/upper_temp_limit", upper_temp_limit)) upper_temp_limit = config.upper_temp_limit;  	
		if (!ros::param::get("~/lower_temp_limit", lower_temp_limit)) lower_temp_limit = config.lower_temp_limit;  	
		if (!ros::param::get("~/zoom_evidence", zoom_evidence)) zoom_evidence = config.zoom_evidence;  	
		if (!ros::param::get("~/upper_evidence_limit", upper_evidence_limit)) upper_evidence_limit = config.upper_evidence_limit;  	
		if (!ros::param::get("~/lower_evidence_limit", lower_evidence_limit)) lower_evidence_limit = config.lower_evidence_limit;  	
		if (!ros::param::get("~/threshold_temp", threshold_temp)) threshold_temp = config.threshold_temp; 	
		if (!ros::param::get("~/threshold_evidence", threshold_evidence)) threshold_evidence = config.threshold_evidence; 	
		if (!ros::param::get("~/min_size_temp", min_size_temp)) min_size_temp = config.min_size_temp; 
		if (!ros::param::get("~/min_size_evidence", min_size_evidence)) min_size_evidence = config.min_size_evidence; 
		if (!ros::param::get("~/max_size_temp", max_size_temp)) max_size_temp = config.max_size_temp; 
		if (!ros::param::get("~/max_size_evidence", max_size_evidence)) max_size_evidence = config.max_size_evidence; 		
		respect_launch_parameters = false;
	}	
	else
	{
		temperatura_mean = config.temperatura_mean; 
		temperatura_stddev = config.temperatura_stddev;
		zoom_temp = config.zoom_temp;
		upper_temp_limit = config.upper_temp_limit;
		lower_temp_limit = config.lower_temp_limit;
		zoom_evidence = config.zoom_evidence;  	
		upper_evidence_limit = config.upper_evidence_limit;  	
		lower_evidence_limit = config.lower_evidence_limit;  	
		threshold_temp = config.threshold_temp; 	
		threshold_evidence = config.threshold_evidence; 	
		min_size_temp = config.min_size_temp; 
		min_size_evidence = config.min_size_evidence; 	
		max_size_temp = config.max_size_temp; 
		max_size_evidence = config.max_size_evidence; 				
	}
}


//void HTPAoutputCallback(const std_msgs::String::ConstPtr& msg)
void HTPAprocessedCallback(const heiman::temperatures::ConstPtr& msg)
{
	heiman::vector_evidencias_HTPA msg_HTPA;
	
	// crear imagenes (por el zoom, que puede ser variable)
	IplImage *color_temp_image, *binary_temp_image, *blob_temp_image, *color_evidence_image, *binary_evidence_image, *blob_evidence_image;
	color_temp_image = cvCreateImage(cvSize(32*zoom_temp,31*zoom_temp),IPL_DEPTH_8U,3);
	binary_temp_image = cvCreateImage(cvSize(32*zoom_temp,31*zoom_temp),IPL_DEPTH_8U,1);
	blob_temp_image = cvCreateImage(cvSize(32*zoom_temp,31*zoom_temp),IPL_DEPTH_8U,1);
	color_evidence_image = cvCreateImage(cvSize(32*zoom_evidence,31*zoom_evidence),IPL_DEPTH_8U,3);
	binary_evidence_image = cvCreateImage(cvSize(32*zoom_evidence,31*zoom_evidence),IPL_DEPTH_8U,1);
	blob_evidence_image = cvCreateImage(cvSize(32*zoom_evidence,31*zoom_evidence),IPL_DEPTH_8U,1);	

	int i,length;

	for (i = 0; i < 992; i++)
	{
		int pixel_i = i / 32;
		int pixel_j = i % 32;
		array_temperaturas[pixel_i][pixel_j] = msg->temperaturas[i]; 
	}
	//ROS_INFO("Termino lo de leer HTPAoutput");
	// ahora calculamos las probabilidades; en principio pasamos de que hayamos leído las dos partes o no: siempre habrá algo

	//if (!ros::param::get("~/temperatura_mean", temperatura_mean)) temperatura_mean = 32.0;  
	//if (!ros::param::get("~/temperatura_stddev", temperatura_stddev)) temperatura_stddv = 2.0;  	
	
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
	
	vectorfromHTPA_pub.publish(msg_HTPA);
	ROS_INFO("Antes de crear las imagenes");
	// creamos las imágenes
	create_color_temp_image(array_temperaturas, *color_temp_image);
	ROS_INFO("Despues de crear color_temp_image");
	create_color_evidence_image(array_convolucionado, *color_evidence_image);
	ROS_INFO("Despues de crear color_evidence_image");
	create_binary_temp_image(array_temperaturas, threshold_temp, zoom_temp, *binary_temp_image);
	ROS_INFO("Despues de crear binary_temp_image");
	create_binary_evidence_image(array_convolucionado, threshold_evidence, zoom_evidence, *binary_evidence_image);	
	ROS_INFO("Despues de crear binary_evidence_image");
	cvCopy(binary_temp_image,blob_temp_image);
	//remove_small_and_big_blobs(blob_temp_image, min_size_temp, max_size_temp);
	ROS_INFO("Despues de crear blob_temp_image");
	cvCopy(binary_evidence_image,blob_evidence_image);
	//remove_small_and_big_blobs(blob_evidence_image, min_size_evidence, max_size_evidence);	
	ROS_INFO("Despues de crear blob_evidence_image");
	// publicamos las imágenes
	// Eli
  cv_bridge::CvImagePtr cpt(new cv_bridge::CvImage);
  ros::Time time = ros::Time::now();
  cpt->header.stamp = time;
  cpt->header.frame_id = "image";
  cpt->encoding = "bgr8";
  cpt->image = color_temp_image; //HTPAimage;
  color_temp_image_pub.publish(cpt->toImageMsg());
  //	sensor_msgs::ImagePtr msg1 = sensor_msgs::cv_bridge::cvToImgMsg(color_temp_image, "bgr8");
  //color_temp_image_pub.publish(msg1);	
  ROS_INFO("msg1");
  //----------------------------------------
  cpt->encoding = "mono8";
  cpt->image = binary_temp_image; //HTPAimage;
  binary_temp_image_pub.publish(cpt->toImageMsg());

  //sensor_msgs::ImagePtr msg2 = sensor_msgs::cv_bridge::cvToImgMsg(binary_temp_image, "mono8");
  //binary_temp_image_pub.publish(msg2);
  ROS_INFO("msg2");	
	//----------------------------------------
  cpt->encoding = "mono8";
  cpt->image = blob_temp_image; //HTPAimage;
  blob_temp_image_pub.publish(cpt->toImageMsg());

  //	sensor_msgs::ImagePtr msg3 = sensor_msgs::cv_bridge::cvToImgMsg(blob_temp_image, "mono8");
  //	blob_temp_image_pub.publish(msg3);	
  ROS_INFO("msg3");
  //----------------------------------------
  cpt->encoding = "bgr8";
  cpt->image = color_evidence_image; //HTPAimage;
  color_evidence_image_pub.publish(cpt->toImageMsg());

  //sensor_msgs::ImagePtr msg4 = sensor_msgs::cv_bridge::cvToImgMsg(color_evidence_image, "bgr8");
  //color_temp_image_pub.publish(msg4);
  ROS_INFO("msg4");
  //----------------------------------------
  cpt->encoding = "mono8";
  cpt->image = binary_evidence_image; //HTPAimage;
  binary_evidence_image_pub.publish(cpt->toImageMsg());
  //sensor_msgs::ImagePtr msg5 = sensor_msgs::cv_bridge::cvToImgMsg(binary_evidence_image, "mono8");
  //binary_evidence_image_pub.publish(msg5);
  ROS_INFO("msg5");	
	//----------------------------------------
  cpt->encoding = "mono8";
  cpt->image = blob_evidence_image; //HTPAimage;
  blob_evidence_image_pub.publish(cpt->toImageMsg());
  //sensor_msgs::ImagePtr msg6 = sensor_msgs::cv_bridge::cvToImgMsg(blob_evidence_image, "mono8");
  //blob_evidence_image_pub.publish(msg6);
  ROS_INFO("msg6");	
  //----------------------------------------
	
  cvReleaseImage(&color_temp_image);
  cvReleaseImage(&binary_temp_image);	
  cvReleaseImage(&blob_temp_image);
  cvReleaseImage(&color_evidence_image);
  cvReleaseImage(&binary_evidence_image);
  cvReleaseImage(&blob_evidence_image);
  
  ROS_INFO("Salgo del callback");
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "getallfromHTPAprocessed");
	ros::NodeHandle n;
	
	vectorfromHTPA_pub = n.advertise<heiman::vector_evidencias_HTPA>("vector_evidencias", 1);
	image_transport::ImageTransport it1(n);
  	color_temp_image_pub = it1.advertise("color_temp_image", 1);	
  	image_transport::ImageTransport it2(n);
  	binary_temp_image_pub = it2.advertise("binary_temp_image", 1);
	image_transport::ImageTransport it3(n);
  	blob_temp_image_pub = it3.advertise("blob_temp_image", 1);
 	image_transport::ImageTransport it4(n);
  	color_evidence_image_pub = it4.advertise("color_evidence_image", 1);
  	image_transport::ImageTransport it5(n);
  	binary_evidence_image_pub = it5.advertise("binary_evidence_image", 1);
 	image_transport::ImageTransport it6(n);
  	blob_evidence_image_pub = it6.advertise("blob_evidence_image", 1); 	
  	 
    ros::Subscriber sub = n.subscribe("HTPAprocessed", 0, HTPAprocessedCallback);
    
	  dynamic_reconfigure::Server<heiman::getallfromHTPApublishedConfig> server;
	  dynamic_reconfigure::Server<heiman::getallfromHTPApublishedConfig>::CallbackType f;

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

