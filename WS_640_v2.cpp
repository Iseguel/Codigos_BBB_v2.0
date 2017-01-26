#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <cstdio>
#include <iostream>
#include <highgui.h>

#include <unistd.h>
//#include "BlackGPIO.h"
//#include "BlackUART.h"
//#include "BlackPWM.h"

using namespace cv;
using namespace std;
//using namespace BlackLib;

class WatershedSegmenter{
private:
    Mat markers;
public:
    void setMarkers(Mat& markerImage)
    {
        markerImage.convertTo(markers, CV_32S);
    }

    Mat process(Mat &image)
    {
        watershed(image, markers);
        markers.convertTo(markers,CV_8U);
        return markers;
    }
};

vector<Vec4i> hierarchy;

Mat markerMask;
Mat image, mask, image_HSV;


float Modulo = 0;
float Angulo = 0;
Point2f mc = Point(320,480); //640x480
//Point2f mc = Point(160,240); //320x240
float dWidth;
float dHeight;

int Muestras = 15;

float DC_X = 3;
float DC_Y = 3;
float tiempo = 1350000;
float VectMCX[15];
float VectMCY[15]; 
float angulo, anguloPromedio;
double margen = 300000;
float mcFx;
float mcFy;

//BlackPWM ServoX (P9_14);
//BlackPWM ServoY (P8_19);

void centroDeMasa(Mat maskResult);
void ActualizaVect(int Muestras);
float VectorDir();
float Distancia(float X1, float Y1, float X2, float Y2);
float CalcAngulo(float HIP, float CATOP, int mcX);
void Trapesio( Mat image, int estado );
void GIRO (float angulo);
//void Motores(float X, float Y, float espera);



int main( int argc, char** argv )
{   
    //BlackUART Arduino(UART4, Baud38400, ParityDefault, StopOne, Char8);
    //Arduino.open(ReadWrite);
    //Arduino.flush( bothDirection );

    //ServoX.setPeriodTime(20,milisecond);
    //ServoX.setPolarity(reverse);   

    //ServoY.setPeriodTime(20,milisecond);
    //ServoY.setPolarity(reverse);
   
    //ServoY.setRunState(stop);    
    //ServoX.setRunState(stop);   

    //string escribeUART;
    //string leeUART;

    float dist, X1, Y1, X2, Y2;
    float HIP, CATOP;
    int mcX;
    int cont = 0;
    int cuadros = 0;
    

    //VideoCapture cap(1); // abre la camara de video Nro
    VideoCapture cap("/home/ismael/Proyectos/Carretera.avi"); // open the video file for reading

    if ( !cap.isOpened() )  // if not success, exit program
    {
         cout << "No se puede abrir el archivo de video" << endl;
         return -1;
    }

    cap.set(CV_CAP_PROP_POS_MSEC, 10000); //parte el video a los 10000ms
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 640); // ancho de la imagen 
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480); // alto de la imagen
    //cap.set(CV_CAP_PROP_FPS, 15); 

    //double fps = cap.get(CV_CAP_PROP_FPS); //get the frames per seconds of the video
    dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
    dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

    cout << "Tamaño cuadro : " << dWidth << " x " << dHeight << endl;
    //cout << "Cuadros por segundo : " << fps << endl;

    //namedWindow("imagen",WINDOW_OPENGL); //crea una ventana con el nombre "Video"
  
    //VideoWriter wrtSRC("WS_v2.0_320.avi", CV_FOURCC('X','V','I','D'), 15/2, Size(640,320));
  
  while(true)
  {
    bool bSuccess = cap.read(image); // lee un nuevo frame de video

    if (!bSuccess) //si no es exitosa, sale del loop
     {
      cout << "No se puede leer de video stream" << endl;
      break;
    }

    cvtColor(image, image_HSV, CV_RGB2HSV);

    Mat blank(image.size(),CV_8U,Scalar(0xFF));
    
    // Create markers image
    Mat markers(image.size(),CV_8U,Scalar(-1));
    //Rect(topleftcornerX, topleftcornerY, width, height);
    //top rectangle
    markers(Rect(0,0,image.cols, 5)) = Scalar::all(1);
    //bottom rectangle
    markers(Rect(0,image.rows-5,image.cols, 5)) = Scalar::all(1);
    //left rectangle
    markers(Rect(0,0,5,image.rows)) = Scalar::all(1);
    //right rectangle
    markers(Rect(image.cols-5,0,5,image.rows)) = Scalar::all(1);
    //centre rectangle
    int centreW = image.cols/2;
    int centreH = image.rows/2;

    //markers(Rect((centreW-35),200, 70, 40)) = Scalar::all(255);

    Trapesio(markers,1);
   
    //imshow("markers", markers);

    //Create watershed segmentation object
    WatershedSegmenter segmenter;
    segmenter.setMarkers(markers);
    Mat wshedMask = segmenter.process(image_HSV);
    convertScaleAbs(wshedMask, mask, 1, 0);
    threshold(mask, mask, 1, 255, THRESH_BINARY);
    //imshow("mascara",mask);
    Mat temp;
    int niters = 3;
    dilate(mask, temp, Mat(), Point(-1,-1), niters);
    dilate(temp, temp, Mat(), Point(-1,-1), niters);
    erode(temp, mask, Mat(), Point(-1,-1), niters*2);

    //wrtSRC << image;

    centroDeMasa(mask);

    ActualizaVect(Muestras);

    if(cont == Muestras){                
        VectorDir();    
        GIRO(Angulo);
        //Motores(DC_X, DC_Y, tiempo);
        cont =0;
    }

    cont++;
    arrowedLine(image, Point(320,480),Point(mcFx,mcFy), Scalar(0,255,0),2, 8, 0, 0.1) ;
    //line(image, Point(160,240),mc, Scalar(0,255,0),2, 8, 0) ;
    // dibuja un rectangulo cerca del origen para el contraste con las letras 
    rectangle(image, Point(10,10), Point(230,70), Scalar(50,50,50),-1,8,0);
    //Dibuja mascara para watershed
    Trapesio(image,0);

    // imprime las coordenadas del cento de masa mc
    int MCX = mc.x;
    int MCY = mc.y;

    char texto_mc[50];   
    sprintf(texto_mc," (%d , %d)x,y ", MCX , MCY );  
    putText(image, texto_mc, Point(mcFx,mcFy), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,0),1,1);
    circle( image, mc, 4, Scalar(100,0,255), -1, 8, 0 );
    
    // imprime el modulo y angulo del vector en la imagen origen
    char texto_modulo[50];   
    sprintf(texto_modulo," Modulo: %f px", Modulo );  
    putText(image, texto_modulo, Point(10,25), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,0),1,1); 
    char texto_angulo[50];   
    sprintf(texto_angulo," %f Grados", Angulo );  
    putText(image, texto_angulo, Point(10 , 45), FONT_HERSHEY_PLAIN, 1.2, Scalar(0,255,240),1,1); 
    
    // imprime el numero del cuadro en la imagen origen
    char texto_Cuadros[50];   
    sprintf(texto_Cuadros," Cuadro Nro %i ", cuadros );  
    putText(image, texto_Cuadros, Point(10,65), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,0),1,1); 



    //cout<<"Angulo "<<Angulo << endl;
    
    //cout << "[0]=" << anguloVect[0] << "  [1]=" << anguloVect[1] << "   [2] = " << anguloVect[2] << endl; 
    /*
    //Imprime los cuadros por segundo en la imagen 
    fps = cap.get(CV_CAP_PROP_FPS); //get the frames per seconds of the video
    char texto_2[10];   
    sprintf(texto_2," %lf fps", fps );  
    putText(image, texto_2, Point(10,30), FONT_HERSHEY_PLAIN, 1, Scalar(38,200,70),1.5,1);
    */   
    imshow("imagen", image);
    //wrtSRC << image;
    cuadros = cuadros +1;
    //cout << "Cuadros:" << cuadros << endl;

    
 
  
    int c;
    c = waitKey( 20 );
    if( (char)c == 27 /*|| cuadros >= 2000*/){ 
        //wrtSRC.release();
        //ServoX.setRunState(stop);    
        //ServoY.setRunState(stop);
        break;
    }
  }
}


void centroDeMasa(Mat maskResult){

        //apertura        
        //dilate(maskResult, temp, Mat(), Point(-1,-1), niters);
        //dilate(temp, temp, Mat(), Point(-1,-1), niters);
        //erode(temp, maskResult, Mat(), Point(-1,-1), niters*2);
        
        //imshow("temp", temp);
        //imshow("MascaraResultado Filtrada", maskResult);

        Moments mu = moments( maskResult, false); 
        // Se calcula el centro de masa de  la mascara como un punto [mc]
        double M01 = mu.m01;
        double M10 = mu.m10;
        double Area = mu.m00;
        mc = Point( M10/Area , M01/Area );

        //Encuentra y dibuja los contornos
        vector<vector<Point> > contours;    
        findContours( maskResult, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
        drawContours( image, contours, -1, Scalar(240,50,0), 2, CV_AA, hierarchy, abs(4) );

        return;
}

void ActualizaVect(int Muestras){

    
    for(int num = Muestras; num > 0; num--){

        VectMCX[num] = VectMCX[num-1];
        VectMCY[num] = VectMCY[num-1];
        
    }
    VectMCX[0] = mc.x; 
    VectMCY[0] = mc.y;

    return;

}


float VectorDir(){

        mcFx = 0;
        mcFy = 0;

        for(int f = 0; f < Muestras; f++){
            //cout << "Nro:" << f<< endl;
            mcFx = mcFx + VectMCX[f];
            //cout << "vectX:" << VectMCX[f];
            mcFy = mcFy + VectMCY[f];
            //cout << "  VectY:" << VectMCX[f] << endl;
        }
        mcFx = (mcFx/Muestras);
        mcFy = (mcFy/Muestras);

        //cout << "mcFx:" << mcFx;
        //cout << "  mcFy:" << mcFy << endl;
        //Calcula el modulo y angulo del vector
        Modulo = Distancia(320, 480, mcFx, mcFy);  //640x480
        Angulo = CalcAngulo(Modulo, (480 - mcFy), mcFx);

        //Modulo = Distancia(160, 240, mcFx, mcFy); //320x240
        //Angulo = CalcAngulo(Modulo, (240 - mcFy), mcFx);
        
        return Modulo, Angulo;

}

float Distancia(float X1, float Y1, float X2, float Y2){
    float d;
    d = sqrt( ((X2-X1)*(X2-X1)) + ((Y2-Y1)*(Y2-Y1))  );
    return d;
}

float CalcAngulo(float HIP, float CATOP, int mcX){
    float ang;
    ang = asin(CATOP/HIP) * 180.0 / 3.141592654;
    if(mcX < 320){
        ang = 180 - ang;
    }
    return ang;
}

void Trapesio( Mat image, int estado )
{
  int lineType = 8;

  // Create some points //
  Point Trapesio_points[1][4];
  
  Trapesio_points[0][0] = Point(290, 430);   //    (0,0)   (0,1)
  Trapesio_points[0][1] = Point(350, 430); 
  Trapesio_points[0][2] = Point(370, 480);   // (0,3)         (0,2)
  Trapesio_points[0][3] = Point(270, 480);

  const Point* ppt[1] = { Trapesio_points[0] };
  int npt[] = { 4 };
  if(estado == 0){
    polylines(image, ppt, npt, 1, 1, Scalar(55,10,255), 2, 8, 0 );
     //fillPoly( image, ppt, npt, 1, Scalar( 10, 10, 100 ), lineType ); 
    //polylines(image, ppt, npt, 1, Scalar( 255, 255, 255 ), 1, 8, 0 )
  }
  if(estado == 1){
    fillPoly( image, ppt, npt, 1, Scalar( 255, 255, 255 ), lineType );
  }
 }

void GIRO (float angulo){
    if(angulo <= 60){  ////////// Menores a 60 grados /////////
        // GIRO 0° a 10°
        if(angulo >= 0 && angulo <= 10){
            DC_X = 3;
            DC_Y = 3;
            tiempo = 1350000 ;
            return;
        }
        // GIRO 11° a 20°
        if(angulo > 10 && angulo <= 20){
            DC_X = 3;
            DC_Y = 3;
            tiempo = 1200000 - margen;
            return;
        }
        // GIRO 21° a 30°
        if(angulo > 20 && angulo <= 30){
            DC_X = 3;
            DC_Y = 3;
            tiempo = 1050000 - margen;
            return;
        }
        // GIRO 31° a 40°
        if(angulo > 30 && angulo <= 40){
            DC_X = 3;
            DC_Y = 3;
            tiempo = 900000 - margen;
            return;
        }
        // GIRO 41° a 50°
        if(angulo > 40 && angulo <= 50){
            DC_X = 3;
            DC_Y = 3;
            tiempo = 750000 - margen;
            return;
        }
        // GIRO 51° a 60°
        if(angulo > 50 && angulo <= 60){
            DC_X = 3;
            DC_Y = 3;
            tiempo = 600000 - margen;
            return;
        }
    }
    if(angulo > 60 && angulo < 120){ ////////Entre 60 y 120 grados///////
        // GIRO 61° a 70°
        if(angulo > 60 && angulo <= 70){
            DC_X = 3;
            DC_Y = 3;
            tiempo = 450000 - margen;
            return;
        }
        // GIRO 71° a 80°
        if(angulo > 70 && angulo <= 80){
            DC_X = 3;
            DC_Y = 3;
            tiempo = 300000 - (margen/2);
            return;
        }
        // GIRO 81° a 90°
        if(angulo > 80 && angulo <= 90){
            DC_X = 3;
            DC_Y = 12;
            tiempo = 150000 + margen;
            return;
        }
        // GIRO 91° a 100°
        if(angulo > 90 && angulo <= 100){
            DC_X = 3;
            DC_Y = 12;
            tiempo = 150000 + margen;
            return;
        }
        // GIRO 101° a 110°
        if(angulo > 100 && angulo <= 110){
            DC_X = 12;
            DC_Y = 12;
            tiempo = 300000 - (margen/2);
            return;
        }
        // GIRO 111° a 120°
        if(angulo > 110 && angulo <= 120){
            DC_X = 12;
            DC_Y = 12;
            tiempo = 450000 - margen;
            return;
        }
    }
    if(angulo >= 120){ ////////// Mayores a 120 grados /////////
        // GIRO 121° a 130°
        if(angulo > 120 && angulo <= 130){
            DC_X = 12;
            DC_Y = 12;
            tiempo = 600000 - margen;
            return;
        }
        // GIRO 131° a 140° 
        if(angulo > 130 && angulo <= 140){
            DC_X = 12;
            DC_Y = 12;
            tiempo = 750000 - margen;
            return;
        }
        // GIRO 141° a 150°
        if(angulo > 140 && angulo <= 150){
            DC_X = 12;
            DC_Y = 12;
            tiempo = 900000 - margen;
            return;
        }
        // GIRO 151° a 160°
        if(angulo > 150 && angulo <= 160){
            DC_X = 12;
            DC_Y = 12;
            tiempo = 1050000 - margen;
            return;
        }
        // GIRO 161° a 170°
        if(angulo > 160 && angulo <= 170){
            DC_X = 12;
            DC_Y = 12;
            tiempo = 1200000 - margen;
            return;
        }
        // GIRO 171° a 180°
        if(angulo > 170 && angulo <= 180){
            DC_X = 12;
            DC_Y = 12;
            tiempo = 1350000 ;
            return;
        }
    }
    if(angulo < 0 || angulo > 120){
        DC_X = 7.35;
        DC_Y = 7.35;
    }

    
}
/*
void Motores(float X, float Y, float espera){

    ServoX.setRunState(run);    
    ServoY.setRunState(run);
    ServoX.setDutyPercent (X);
    ServoY.setDutyPercent (Y);
    
    //cout << angulo <<" grados";
    //cout << " PWM X: " << ServoX.getValue();
    //cout << "  PWM Y: " << ServoY.getValue() << endl;
    usleep(espera);
    ServoX.setRunState(stop);    
    ServoY.setRunState(stop);

    return;
}
*/

