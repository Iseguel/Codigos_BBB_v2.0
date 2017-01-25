

/*
  ////////////////////////
  --Ismael Seguel Avello--
  ////////////////////////

Codigo para deteccion de senderos en base a color
segmentando con Floodfil
*/

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <highgui.h>
#include <unistd.h>
#include "BlackPWM.h"
//#include "BlackGPIO.h"
//#include "BlackUART.h"

using namespace cv;
using namespace std;
using namespace BlackLib;

int ffillMode = 1;
int loDiff = 28, upDiff = 28;
int connectivity = 4;
bool useMask = true;
int newMaskVal = 255;
int area;

Mat src, src0, src_HSV;

Point point1, point2, Vdirec;
Rect rect; 
Rect ccomp;
Mat roiImg , mask;
int i =0 ;



vector<Vec4i> hierarchy;

double DC_X = 3;
double DC_Y = 3;
double tiempo = 1350000;
float anguloVect[3]; 
float angulo, anguloPromedio;
double margen = 300000;

BlackPWM ServoX (P9_14);
BlackPWM ServoY (P8_19);

float Distancia(float X1, float Y1, float X2, float Y2);
float CalcAngulo(float HIP, float CATOP, int mcX);
void Motores(float X, float Y, float espera);
void GIRO (float angulo);

int main( int argc, char** argv )
{   
    //BlackUART Arduino(UART4, Baud38400, ParityDefault, StopOne, Char8);
    //Arduino.open(ReadWrite);
    //Arduino.flush( bothDirection );

    anguloVect[0] = 90.0; anguloVect[1] = 90.0; anguloVect[2] = 90.0;

    ServoX.setPeriodTime(20,milisecond);

    ServoY.setPeriodTime(20,milisecond);

    ServoY.setRunState(stop);    
    ServoX.setRunState(stop);


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
    //Point2f mc = Point(320,480); //640x480
    Point2f mc = Point(160,240); //320x240
   

    //VideoCapture cap("/home/ismael/Proyectos/MAH00738.avi"); // open the video file for reading
    //VideoCapture cap("/home/ismael/Proyectos/GOPR5146.avi");
    //VideoCapture cap("/home/ismael/Proyectos/GOPR5146.avi");
    VideoCapture cap(0);

    if ( !cap.isOpened() )  // if not success, exit program
    {
         cout << "No se puede abrir el archivo de video" << endl;
         return -1;
    }

    //cap.set(CV_CAP_PROP_POS_MSEC, 10000); //parte el video a los 10000ms
    //cap.set(CV_CAP_PROP_FRAME_WIDTH, 640); // ancho de la imagen 
    //cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480); // alto de la imagen
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 320); // ancho de la imagen 
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240); // alto de la imagen
    //cap.set(CV_CAP_PROP_FPS, 30); 

    //double fps = cap.get(CV_CAP_PROP_FPS); //get the frames per seconds of the video
    double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
    double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

    cout << "Tamaño cuadro : " << dWidth << " x " << dHeight << endl;
    //cout << "Cuadros por segundo : " << fps << endl;

    //namedWindow("Video",CV_WINDOW_AUTOSIZE); //crea una ventana con el nombre "Video"

    //createTrackbar( "lo_diff", "Video", &loDiff, 255, 0 );
    //createTrackbar( "up_diff", "Video", &upDiff, 255, 0 );

    mask.create(dHeight+2, dWidth+2, CV_8UC1);

    VideoWriter wrtSRC("ROW_v2.0_320.avi", CV_FOURCC('X','V','I','D'), 15/2, Size(320,240));
    
  while(true)
  {

    bool bSuccess = cap.read(src); // lee un nuevo frame de video

    if (!bSuccess) //si no es exitosa, sale del loop
     {
      cout << "No se puede leer de video stream" << endl;
      break;
    }
    src.copyTo(src0);
    cvtColor(src, src_HSV, CV_RGB2HSV);
    
    // Segmentacion con FloodFill
    //punto de inicio del cultivo
    //Point seed = Point(320,420);  //640x480
    Point seed = Point(160,230); //320x240
    circle( src, seed, 3, Scalar(0,0,255),1, 2,0);// dibuja un circulo en la ROG

    int lo = loDiff;
    int up = upDiff;
    int flags = connectivity + (newMaskVal << 8) + (CV_FLOODFILL_FIXED_RANGE);
    
    mask = Scalar::all(0);
    threshold(mask, mask, 1, 128, THRESH_BINARY);
    //floodFill(src0, mask, seed, Scalar(255,0,0), &ccomp, Scalar(lo, lo, lo), Scalar(up, up, up), flags);
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
        cont = 0;
        Mascara6 = mask;

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
        //Modulo = Distancia(320, 480, mc.x, mc.y);  //640x480
        //Angulo = CalcAngulo(Modulo, (480 - mc.y), mc.x);

        Modulo = Distancia(160, 240, mc.x, mc.y); //320x240
        Angulo = CalcAngulo(Modulo, (240 - mc.y), mc.x);

        //Encuentra y dibuja los contornos
        vector<vector<Point> > contours;    
        findContours( maskResult, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
        drawContours( src, contours, -1, Scalar(240,50,0), 2, CV_AA, hierarchy, abs(4) );
   
        wrtSRC << src;

        anguloVect[2] = anguloVect[1];
        anguloVect[1] = anguloVect[0];
        anguloVect[0] = Angulo;

        angulo = (anguloVect[0] + anguloVect[0] + anguloVect[1] + anguloVect[2]);
        anguloPromedio = angulo/4;

        //cout << "angulo total:" << angulo << " --> promedio:" << anguloPromedio <<endl;
    
        GIRO(Angulo);
    
        Motores(DC_X, DC_Y, tiempo);

    }
    cont++;
    //Dibuja una linea desde el "origen", centro inferior de la imagen hasta el CM 
    //arrowedLine(src, Point(320,480),mc, Scalar(0,255,0),2, 8, 0, 0.1) ;
    //line(src, Point(320,480), mc, Scalar(0,255,0), 2, 8, 0);
    line(src, Point(160,240),mc, Scalar(0,255,0),2, 8, 0) ;
    rectangle(src, Point(10,10), Point(230,70), Scalar(50,50,50),-1,8,0);

    // imprime las coordenadas del cento de masa mc
     int MCX = mc.x;
     int MCY = mc.y;

    char texto_mc[100];   
    sprintf(texto_mc," (%d , %d)x,y ", MCX , MCY );  
    putText(src, texto_mc, mc, FONT_HERSHEY_PLAIN, 1, Scalar(0,255,0),1,1);
    circle( src, mc, 4, Scalar(100,0,255), -1, 8, 0 );
    // imprime el modulo y angulo del vector en la imagen origen
    char texto_modulo[100];   
    sprintf(texto_modulo," Modulo: %f px", Modulo );  
    putText(src, texto_modulo, Point(10,25), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,0),1,1); 
    char texto_angulo[100];   
    sprintf(texto_angulo," %f Grados", Angulo );  
    putText(src, texto_angulo, Point(10 , 45), FONT_HERSHEY_PLAIN, 1.2, Scalar(0,255,240),1,1); 

    // imprime el numero del cuadro en la imagen origen
    char texto_Cuadros[100];   
    sprintf(texto_Cuadros," Cuadro Nro %i ", cuadros );  
    putText(src, texto_Cuadros, Point(10,65), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,0),1,1);

    //cout<<"Angulo: "<<Angulo<<" --- Modulo:"<<Modulo<<endl; 
/*
    //Imprime los cuadros por segundo en la imagen 
    //fps = cap.get(CV_CAP_PROP_FPS); //get the frames per seconds of the video
    char texto_2[10];   
    sprintf(texto_2," %lf fps", fps );  
    putText(src, texto_2, Point(10,30), FONT_HERSHEY_PLAIN, 1, Scalar(38,200,70),1.5,1);
 */ 
    //imshow( "Video", src );
    wrtSRC << src;
    cuadros = cuadros + 1;
    //cout << "cuadro:" << cuadros << endl;

    
      
   int c;
   c = waitKey( 20 );
   if( (char)c == 27 || cuadros >= 1000){ 
        wrtSRC.release();
        ServoX.setRunState(stop);    
        ServoY.setRunState(stop);
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

