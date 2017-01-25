
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <highgui.h>

#include <unistd.h>
#include "BlackGPIO.h"
#include "BlackUART.h"
#include "BlackPWM.h"

using namespace cv;
using namespace std;

using namespace BlackLib;

int ffillMode = 1;
int loDiff = 20, upDiff = 20;
int connectivity = 4;
bool useMask = true;
int newMaskVal = 255;
int area;

Mat src, src0;

Point point1, point2, Vdirec;
Rect rect; 
Rect ccomp;
Mat roiImg , mask;
int i =0 ;



vector<Vec4i> hierarchy;

float Distancia(float X1, float Y1, float X2, float Y2);
float CalcAngulo(float HIP, float CATOP, int mcX);

int main( int argc, char** argv )
{   
    BlackUART Arduino(UART4, Baud38400, ParityDefault, StopOne, Char8);
    Arduino.open(ReadWrite);
    Arduino.flush( bothDirection );

    double DC_X = 3;
    double DC_Y = 3;

    BlackPWM ServoX (P9_14);
    BlackPWM ServoY (P8_19);

    ServoX.setPeriodTime(20,milisecond);
    //ServoX.setPolarity(reverse);   

    ServoY.setPeriodTime(20,milisecond);
    //ServoY.setPolarity(reverse);

   
    ServoY.setRunState(stop);    
    ServoX.setRunState(stop);   


    string escribeUART;
    string leeUART;

    float dist, X1, Y1, X2, Y2;
    float HIP, CATOP;
    int mcX;
    float sumaX = 0;
    float sumaY = 0;
    int cont = 0;
    int cuadros = 0;
    Mat Mascara1, Mascara2, Mascara3, Mascara4;
    Mat Mascara5, Mascara6;
    Mat maskResult, maskResult1, maskResult2;
    Mat maskResult3, maskResult4, maskResult5;
    float Modulo = 0;
    float Angulo = 0;
    Point2f mc = Point(160,240);

    

    //VideoCapture cap("/home/ismael/Proyectos/MAH00738.avi"); // open the video file for reading
    //VideoCapture cap("/home/ismael/Proyectos/GOPR5146.avi");
    VideoCapture cap(0);

    if ( !cap.isOpened() )  // if not success, exit program
    {
         cout << "No se puede abrir el archivo de video" << endl;
         return -1;
    }

    //cap.set(CV_CAP_PROP_POS_MSEC, 10000); //parte el video a los 10000ms
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 320); // ancho de la imagen 
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240); // alto de la imagen
    //cap.set(CV_CAP_PROP_FPS, 15); 

    //double fps = cap.get(CV_CAP_PROP_FPS); //get the frames per seconds of the video
    double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
    double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

    cout << "Tamaño cuadro : " << dWidth << " x " << dHeight << endl;
    //cout << "Cuadros por segundo : " << fps << endl;

    //namedWindow("Video",CV_WINDOW_AUTOSIZE); //crea una ventana con el nombre "Video"

    //createTrackbar( "lo_diff", "Video", &loDiff, 255, 0 );
    //createTrackbar( "up_diff", "Video", &upDiff, 255, 0 );

    mask.create(dHeight+2, dWidth+2, CV_8UC1);

    VideoWriter wrtSRC("ROW_servos_v2.0_320.avi", CV_FOURCC('X','V','I','D'), 15/2, Size(320,240));
    //VideoWriter wrtMask("CAMARA_FloodFill-MASK.avi", CV_FOURCC('P','I','M','1'), 20, Size(640,480));

  while(true)
  {

    bool bSuccess = cap.read(src); // lee un nuevo frame de video

    if (!bSuccess) //si no es exitosa, sale del loop
     {
      cout << "No se puede leer de video stream" << endl;
      break;
    }
    src.copyTo(src0);

    // Segmentacion con FloodFill
    Point seed = Point(160,210);  //punto de inicio del cultivo
    circle( src, seed, 3, Scalar(0,0,255),1, 2,0);// dibuja un circulo en la ROG

    int lo = loDiff;
    int up = upDiff;
    int flags = connectivity + (newMaskVal << 8) + (CV_FLOODFILL_FIXED_RANGE);
    
    mask = Scalar::all(0);
    threshold(mask, mask, 1, 128, THRESH_BINARY);
    floodFill(src0, mask, seed, Scalar(255,0,0), &ccomp, Scalar(lo, lo, lo), Scalar(up, up, up), flags);
    
    Mat temp;
        int niters = 3;
        dilate(mask, temp, Mat(), Point(-1,-1), niters);
        dilate(temp, temp, Mat(), Point(-1,-1), niters);
        erode(temp, mask, Mat(), Point(-1,-1), niters*2);
    //imshow("Mascara actual", mask); 
    //Busca contornos y los dibuja
    
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
        
        //imshow("temp", temp);
        //imshow("MascaraResultado Filtrada", maskResult);

        Moments mu = moments( maskResult, false); 
        // Se calcula el centro de masa de  la mascara como un punto [mc]
        double M01 = mu.m01;
        double M10 = mu.m10;
        double Area = mu.m00;
        mc = Point( M10/Area , M01/Area );

        //Calcula el modulo y angulo del vector
        Modulo = Distancia(160, 240, mc.x, mc.y);
        Angulo = CalcAngulo(Modulo, (240 - mc.y), mc.x);

        //Encuentra y dibuja los contornos
        vector<vector<Point> > contours;    
        findContours( maskResult, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
        drawContours( src, contours, -1, Scalar(240,50,0), 2, CV_AA, hierarchy, abs(4) );

        wrtSRC << src;
    }

    cont++;
    //Dibuja una linea desde el "origen", centro inferior de la imagen hasta el CM 
    //arrowedLine(src, Point(160,240),mc, Scalar(0,255,0),2, 8, 0, 0.1) ;

    rectangle(src, Point(10,15), Point(230,70), Scalar(50,50,50),-1,8,0);

    // imprime el modulo y angulo del vector en la imagen origen
    char texto_modulo[100];   
    sprintf(texto_modulo," Modulo: %f px", Modulo );  
    putText(src, texto_modulo, Point(10,45), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,0),1,1); 
    char texto_angulo[100];   
    sprintf(texto_angulo," %f Grados", Angulo );  
    putText(src, texto_angulo, Point(10 , 65), FONT_HERSHEY_PLAIN, 1.2, Scalar(0,255,240),1,1); 

    wrtSRC << src;
/*
    //Imprime los cuadros por segundo en la imagen 
    fps = cap.get(CV_CAP_PROP_FPS); //get the frames per seconds of the video
    char texto_2[10];   
    sprintf(texto_2," %lf fps", fps );  
    putText(src, texto_2, Point(10,30), FONT_HERSHEY_PLAIN, 1, Scalar(38,200,70),1.5,1);
    imshow( "Video", src );
*/   

    // ENVIA ANGULO Y MODLO POR SERIAL DE BBB
    //escribeUART  = Angulo;  //Angulo----------
    //Arduino << escribeUART;
    //usleep(2500);
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
      if(Modulo < 30){
            if(Angulo > 90){
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
    */
    if(Angulo>=75 && Angulo<105){ //ADELANTE
        ServoX.setRunState(run);    
        ServoY.setRunState(run);
        ServoX.setDutyPercent (4);
        ServoY.setDutyPercent (11);
        usleep(10000);
        ServoX.setRunState(stop);    
        ServoY.setRunState(stop);   
    }
    if(Angulo>=140 && Angulo<=180){ //IZQUIERDA
        ServoX.setRunState(run);    
        ServoY.setRunState(run);
        ServoX.setDutyPercent (11);
        ServoY.setDutyPercent (11);
        usleep(10000);
        ServoX.setRunState(stop);    
        ServoY.setRunState(stop); 
    }
    if(Angulo>=105 && Angulo<140){ //IZQUIERDA SUAVE
        ServoX.setRunState(stop);    
        ServoY.setRunState(run);
        ServoY.setDutyPercent (11);
        usleep(10000);
        ServoX.setRunState(stop);    
        ServoY.setRunState(stop); 
    }
    if(Angulo>=0 && Angulo<40){ //DERECHA
        ServoX.setRunState(run);    
        ServoY.setRunState(run);
        ServoX.setDutyPercent (4);
        ServoY.setDutyPercent (4);
        usleep(10000);
        ServoX.setRunState(stop);    
        ServoY.setRunState(stop); 
    }
    if(Angulo>=45 && Angulo<70){ //DERECHA SUAVE
        ServoX.setRunState(run);    
        ServoY.setRunState(stop);
        ServoX.setDutyPercent (4);
        usleep(10000);
        ServoX.setRunState(stop);    
        ServoY.setRunState(stop); 
    }
    

    //cout<<"Angulo: "<<Angulo<<" --- Modulo:"<<Modulo<<endl; 

    cuadros = cuadros + 1;
    //cout << "Cuadros:" << cuadros << endl;

   if(cont > 5){
    cont = 0;
   }
   
   int c;
   c = waitKey( 20 );
   if( (char)c == 27 || cuadros >= 500){ 
        wrtSRC.release();
        break; 
    }
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
    if(mcX < 160){
        ang = 180 - ang;
    }
    return ang;
}

