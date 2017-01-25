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
Mat image, mask, image_HSV, image_HLS, image_Luv, image_Lab;

float Distancia(float X1, float Y1, float X2, float Y2);
float CalcAngulo(float HIP, float CATOP, int mcX);
void Trapesio( Mat image, int estado );

int main( int argc, char** argv )
{   
    //BlackUART Arduino(UART4, Baud38400, ParityDefault, StopOne, Char8);
    //Arduino.open(ReadWrite);
    //Arduino.flush( bothDirection );

    //string escribeUART;
    //string leeUART;

    float dist, X1, Y1, X2, Y2;
    float HIP, CATOP;
    int mcX;
    int cont = 0;
    int cuadros = 0;
    Mat Mascara1, Mascara2, Mascara3, Mascara4;
    Mat Mascara5, Mascara6;
    Mat maskResult, maskResult1, maskResult2;
    Mat maskResult3, maskResult4, maskResult5;
    float Modulo = 0;
    float Angulo = 0;
    Point2f mc = Point(320,480);

    //VideoCapture cap(0); // abre la camara de video Nro
    VideoCapture cap("/home/ismael/Proyectos/Carretera.avi"); // open the video file for reading

    if ( !cap.isOpened() )  // if not success, exit program
    {
         cout << "No se puede abrir el archivo de video" << endl;
         return -1;
    }

    cap.set(CV_CAP_PROP_POS_MSEC, 100000); //parte el video a los 10000ms
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 640); // ancho de la imagen 
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480); // alto de la imagen
    //cap.set(CV_CAP_PROP_FPS, 20); 

    double fps = cap.get(CV_CAP_PROP_FPS); //get the frames per seconds of the video
    double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
    double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

    cout << "Tamaño cuadro : " << dWidth << " x " << dHeight << endl;
    cout << "Cuadros por segundo : " << fps << endl;

    //namedWindow("imagen",WINDOW_OPENGL); //crea una ventana con el nombre "Video"
  
    //VideoWriter wrtSRC("TestWatershed_v2.0_640.avi", CV_FOURCC('X','V','I','D'), 15, Size(640,480));
  
  while(true)
  {
    bool bSuccess = cap.read(image); // lee un nuevo frame de video

    if (!bSuccess) //si no es exitosa, sale del loop
     {
      cout << "No se puede leer de video stream" << endl;
      break;
    }

    cvtColor(image, image_HSV, CV_RGB2HSV);
    //imshow("HSV",image_HSV);

    //cvtColor(image, image_HLS, CV_RGB2HLS);
    //imshow("HLS",image_HLS);

    //cvtColor(image, image_Luv, CV_RGB2Luv);
    //imshow("Luv",image_Luv);

    //cvtColor(image, image_Lab, CV_RGB2Lab);
    //imshow("Lab",image_Lab);

   

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
    int centreW = image.cols/4;
    int centreH = image.rows/4;

    Trapesio(markers,1);
    
    //markers(Rect(270, 400, 140, 75)) = Scalar::all(255);
   
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

    if(cont == 0){
        Mascara1 = mask;
        //imshow("mascara1", Mascara1);
    }
    if(cont == 1){
        Mascara2 = mask;
        //imshow("mascara2", Mascara2);
    }
    if(cont == 2){
        Mascara3 = mask;
        //imshow("mascara3", Mascara2);
    }
    if(cont == 3){
        Mascara4 = mask;
        //imshow("mascara3", Mascara2);
    }
    if(cont == 4){
        Mascara5 = mask;
        //imshow("mascara3", Mascara2);
    
        bitwise_or(Mascara1, Mascara2, maskResult1 );
        bitwise_or(Mascara3,maskResult1,maskResult2);
        bitwise_or(Mascara4,maskResult2,maskResult3);
        bitwise_or(Mascara5,maskResult3,maskResult);

        dilate(maskResult, temp, Mat(), Point(-1,-1), niters);
        dilate(temp, temp, Mat(), Point(-1,-1), niters);
        erode(temp, maskResult, Mat(), Point(-1,-1), niters*2);

        cont = 0;

        
        //imshow("temp", temp);
        //imshow("MascaraResultado Filtrada", maskResult);

        Moments mu = moments( maskResult, false); 
        // Se calcula el centro de masa de  la mascara como un punto [mc]
        double M01 = mu.m01;
        double M10 = mu.m10;
        double Area = mu.m00;
        mc = Point( M10/Area , M01/Area );

        //Calcula el modulo y angulo del vector
        Modulo = Distancia(320, 480, mc.x, mc.y);
        Angulo = CalcAngulo(Modulo, (480 - mc.y), mc.x);

        //Encuentra y dibuja los contornos
        vector<vector<Point> > contours;    
        findContours( maskResult, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
        drawContours( image, contours, -1, Scalar(240,50,0), 2, CV_AA, hierarchy, abs(4) );    
        
        //wrtSRC << image;
    }

    cont++;

    if(cont > 5){
        cont = 0;
    }
    
    //Dibuja una linea desde el "origen", centro inferior de la imagen hasta el CM 
    //arrowedLine(image, Point(320,480),mc, Scalar(0,255,0),2, 8, 0, 0.1) ;
    Trapesio(image,0);
    // imprime las coordenadas del cento de masa mc
    char texto_mc[10];   
    sprintf(texto_mc," (%f , %f)x,y ", mc.x , mc.y );  
    putText(image, texto_mc, mc, FONT_HERSHEY_PLAIN, 1, Scalar(0,255,0),1,1);
    circle( image, mc, 4, Scalar(100,0,255), -1, 8, 0 );

    // dibuja un rectangulo cerca del origen para el contraste con las letras 
    rectangle(image, Point(10,15), Point(230,90), Scalar(50,50,50),-1,8,0);
    // imprime el modulo y angulo del vector en la imagen origen
    char texto_modulo[100];   
    sprintf(texto_modulo," Modulo: %f px", Modulo );  
    putText(image, texto_modulo, Point(10,45), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,0),1,1); 
    char texto_angulo[100];   
    sprintf(texto_angulo," %f Grados", Angulo );  
    putText(image, texto_angulo, Point(10 , 65), FONT_HERSHEY_PLAIN, 1.2, Scalar(0,255,240),1,1); 
    // imprime el numero del cuadro en la imagen origen
    char texto_Cuadros[100];   
    sprintf(texto_Cuadros," Cuadro Nro %i ", cuadros );  
    putText(image, texto_Cuadros, Point(10,85), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,0),1,1); 

/*
    // ENVIA ANGULO Y MODLO POR SERIAL DE BBB
    escribeUART  = Angulo;  //Angulo----------
    Arduino << escribeUART;
    usleep(2500);
    //Arduino >> leeUART;
    //usleep(2500);   
    //cout << "lectura: " << leeUART << endl;
    //usleep(2500);
    /*
    escribeUART  = Modulo;  //Modulo----------
    usleep(2500);
    Arduino >> leeUART;
    usleep(2500);   
    cout << "lectura: " << leeUART << endl;
    usleep(2500);
    */
    /*
    // ENVIA DIRECCION POR SERIAL
    if(Angulo>70 && Angulo<110){ //ADELANTE
      if(Modulo < 70){
            if(mc.x > 320){
                    escribeUART  = "5";  //DERECHA
                Arduino << escribeUART;
                usleep(2500);
                Arduino >> leeUART;
                usleep(2500);
                
                cout<< "DIRECCION CORREGIDA***DERECHA***" << endl;
            }else{
                    escribeUART  = "3";  //IZQUIERDA
                Arduino << escribeUART;
                usleep(2500);
                Arduino >> leeUART;
                usleep(2500);
                
                cout<< "DIRECCION CORREGIDA***IZQUIERDA***" << endl;
            }
            
        }else {
              escribeUART  = "1";  //ADELANTE
                Arduino << escribeUART;
                usleep(2500);
                Arduino >> leeUART;
                usleep(2500);
            cout << "ADELANTE " << endl;
            }
        
    }
    if(Angulo>=135 && Angulo<180){ //IZQUIERDA
        escribeUART  = "3";  //IZQUIERDA
        Arduino << escribeUART;
        usleep(2500);
        Arduino >> leeUART;
        usleep(2500);   
       cout << "IZQUIERDA " << endl;
    }
    if(Angulo>=110 && Angulo<135){ //IZQUIERDA SUAVE
        escribeUART  = "4";  //IZQUIERDA SUAVE
        Arduino << escribeUART;
        usleep(2500);
        Arduino >> leeUART;
        usleep(2500);   
        cout << "IZQUIERDA SUAVE" << endl;
    }
    if(Angulo>=0 && Angulo<45){ //DERECHA
        escribeUART  = "5";  //DERECHA
        Arduino << escribeUART;
        usleep(2500);
        Arduino >> leeUART;
        usleep(2500);   
        cout << "DERECHA" << endl;
    }
    if(Angulo>=45 && Angulo<70){ //DERECHA SUAVE
        escribeUART  = "6";  //DERECHA SUAVE
        Arduino << escribeUART;
        usleep(2500);
        Arduino >> leeUART;
        usleep(2500);   
        cout << "DERECHA SUAVE" << endl;
    }
    if(Angulo>180){ //ATRAS
        escribeUART  = "2";  //ATRAS
        Arduino << escribeUART;
        usleep(2500);
        Arduino >> leeUART;
        usleep(2500);   
       cout << "ATRAS" << endl;
    }
    */
    
    //cout<<"Angulo: "<<Angulo<<" --- Modulo:"<<Modulo<<endl; 
    
    
    //Imprime los cuadros por segundo en la imagen 
    fps = cap.get(CV_CAP_PROP_FPS); //get the frames per seconds of the video
    char texto_2[10];   
    sprintf(texto_2," %lf fps", fps );  
    putText(image, texto_2, Point(10,30), FONT_HERSHEY_PLAIN, 1, Scalar(38,200,70),1.5,1);
      
    //updateWindow("imagen");
    //imshow("imagen", image);
    //wrtSRC << image;
    //cout << "cuadro:" << cuadros << endl;

    
  
   int c;
   c = waitKey( 20 );
   if( (char)c == 27 || cuadros == 2408){ 
        //wrtSRC.release();
        while(1){
            //imshow( "imagen", image );
            c = waitKey( 20 );
            if( (char)c == 27 /*|| cuadros >= 2000*/){ 
                //wrtSRC.release();
                break; 
            }
        }
        break; 
    }
    cuadros = cuadros + 1;
  }
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

  /** Create some points */
  Point Trapesio_points[1][4];
  Trapesio_points[0][0] = Point(325, 410);   //    (0,0)   (0,1)
  Trapesio_points[0][1] = Point(355, 410); 
  Trapesio_points[0][2] = Point(420, 485);   // (0,3)         (0,2)
  Trapesio_points[0][3] = Point(280, 485);

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
      