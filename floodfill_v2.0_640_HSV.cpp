

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
//#include "BlackGPIO.h"
//#include "BlackUART.h"

using namespace cv;
using namespace std;
//using namespace BlackLib;

int ffillMode = 1;
int loDiff = 20, upDiff = 20;
int connectivity = 4;
bool useMask = true;
int newMaskVal = 255;
int area;

Mat src, src0, src_HSV, src_HLS, src_bayerRG, src_Luv, src_Lab;




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
    //BlackUART Arduino(UART4, Baud38400, ParityDefault, StopOne, Char8);
    //Arduino.open(ReadWrite);
    //Arduino.flush( bothDirection );

    string escribeUART;
    string leeUART;

    float dist, X1, Y1, X2, Y2;
    float HIP, CATOP;
    int mcX;
    int cont = 0;
    int cuadros = 0, foto = 0;
    Mat Mascara1, Mascara2, Mascara3, Mascara4;
    Mat Mascara5, Mascara6;
    Mat maskResult, maskResult1, maskResult2;
    Mat maskResult3, maskResult4, maskResult5;
    Mat maskOut;
    float Modulo = 0;
    float Angulo = 0;
    Point2f mc = Point(320,480);

    

    //VideoCapture cap("/home/ismael/Proyectos/MAH00738.avi"); // open the video file for reading
    //VideoCapture cap("/home/ismael/Proyectos/GOPR5146.avi");
    VideoCapture cap("/home/ismael/Proyectos/Carretera.avi");
    //VideoCapture cap(1);

    if ( !cap.isOpened() )  // if not success, exit program
    {
         cout << "No se puede abrir el archivo de video" << endl;
         return -1;
    }

    cap.set(CV_CAP_PROP_POS_MSEC, 100000); //parte el video a los 10000ms
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 640); // ancho de la imagen 
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480); // alto de la imagen
    //cap.set(CV_CAP_PROP_FPS, 30); 

    double fps = cap.get(CV_CAP_PROP_FPS); //get the frames per seconds of the video
    double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
    double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

    cout << "Tamaño cuadro : " << dWidth << " x " << dHeight << endl;
    cout << "Cuadros por segundo : " << fps << endl;

    

    //namedWindow("Video",CV_WINDOW_AUTOSIZE); //crea una ventana con el nombre "Video"

    //createTrackbar( "lo_diff", "Video", &loDiff, 255, 0 );
    //createTrackbar( "up_diff", "Video", &upDiff, 255, 0 );

    mask.create(dHeight+2, dWidth+2, CV_8UC1);

    //VideoWriter wrtSRC("TestFloodFill_v2.0_640.avi", CV_FOURCC('X','V','I','D'), 15, Size(640,480));
    
  while(true)
  {

    bool bSuccess = cap.read(src); // lee un nuevo frame de video

    if (!bSuccess) //si no es exitosa, sale del loop
     {
      cout << "No se puede leer de video stream" << endl;
      break;
    }
    

    Mat acc = Mat::zeros(mask.size(), CV_32FC1);

    src.copyTo(src0);

    cvtColor(src, src_HSV, CV_RGB2HSV);
    imshow("HSV",src_HSV);

    //cvtColor(src, src_HLS, CV_RGB2HLS);
    //imshow("HLS",src_HLS);

    //cvtColor(src, src_Luv, CV_RGB2Luv);
    //imshow("Luv",src_Luv);

    //cvtColor(src, src_Lab, CV_RGB2Lab);
    //imshow("Lab",src_Lab);

   
    
    

    // Segmentacion con FloodFill
    Point seed = Point(320,420);  //punto de inicio del cultivo
    circle( src, seed, 3, Scalar(0,0,255),1, 2,0);// dibuja un circulo en la ROG

    int lo = loDiff;
    int up = upDiff;
    int flags = connectivity + (newMaskVal << 8) + (CV_FLOODFILL_FIXED_RANGE);
    
    mask = Scalar::all(0);
    threshold(mask, mask, 1, 128, THRESH_BINARY);
    floodFill(src_HSV, mask, seed, Scalar(255,0,0), &ccomp, Scalar(lo, lo, lo), Scalar(up, up, up), flags);
    
    Mat temp;
    int niters = 3;
    dilate(mask, temp, Mat(), Point(-1,-1), niters);
    dilate(temp, temp, Mat(), Point(-1,-1), niters);
    erode(temp, mask, Mat(), Point(-1,-1), niters*2);
    
    //imshow("Mascara actual", mask); 
    //Busca contornos y los dibuja

    //accumulateWeighted(src, acc, 0.5);
    
    //imshow("acumulado", acc);
    //imshow("mask", mask);

    
    
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
        //accumulate(maskResult, maskOut, src);
        

        imshow("MascaraResultado Filtrada", maskResult);
        


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
        //drawContours( src, contours, -1, Scalar(240,50,0), 2, CV_AA, hierarchy, abs(4) );
   
        //wrtSRC << src;
        
    }
    cont++;
    /*
    //Dibuja una linea desde el "origen", centro inferior de la imagen hasta el CM 
    arrowedLine(src, Point(320,480),mc, Scalar(0,255,0),2, 8, 0, 0.1) ;
    // imprime las coordenadas del cento de masa mc
    char texto_mc[10];   
    sprintf(texto_mc," (%f , %f)x,y ", mc.x , mc.y );  
    putText(src, texto_mc, mc, FONT_HERSHEY_PLAIN, 1, Scalar(0,255,0),1,1);
    circle( src, mc, 4, Scalar(100,0,255), -1, 8, 0 );

    // dibuja un rectangulo cerca del origen para el contraste con las letras 
    rectangle(src, Point(10,15), Point(230,90), Scalar(50,50,50),-1,8,0);
    // imprime el modulo y angulo del vector en la imagen origen
    char texto_modulo[100];   
    sprintf(texto_modulo," Modulo: %f px", Modulo );  
    putText(src, texto_modulo, Point(10,45), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,0),1,1); 
    char texto_angulo[100];   
    sprintf(texto_angulo," %f Grados", Angulo );  
    putText(src, texto_angulo, Point(10 , 65), FONT_HERSHEY_PLAIN, 1.2, Scalar(0,255,240),1,1); 
    // imprime el numero del cuadro en la imagen origen
    char texto_Cuadros[100];   
    sprintf(texto_Cuadros," Cuadro Nro %i ", cuadros );  
    putText(src, texto_Cuadros, Point(10,85), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,0),1,1); 
    */
      // ENVIA ANGULO Y MODLO POR SERIAL DE BBB
    
    ////////escribeUART  = Angulo;  //Angulo----------
    ////////Arduino << escribeUART;
    ////////usleep(2500);

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
      if(Modulo < 60){
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
    putText(src, texto_2, Point(10,30), FONT_HERSHEY_PLAIN, 1, Scalar(38,200,70),1.5,1);
 
    imshow( "Video", src );
    //wrtSRC << src;
    
    //cout << "cuadro:" << cuadros << endl;
    /*
    if(cuadros == 105 || cuadros == 205 || cuadros == 305){
        vector<int> compression_params;
        compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
        compression_params.push_back(9);
        char texto_foto[100];
        sprintf(texto_foto,"SegmentROGorig_%i.png",cuadros);
        imwrite(texto_foto, src, compression_params);
        char texto_foto2[100];
        sprintf(texto_foto2,"SegmentROG_%i.png",cuadros);
        imwrite(texto_foto2, maskResult, compression_params);
    }    
    */

    /*

    if(foto >= 100 && cont == 4){
            foto = 0;
            vector<int> compression_params;
            compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
            compression_params.push_back(9);
            char texto_foto[100];
            sprintf(texto_foto,"SegmentROGorig_%i.png",cuadros);
            imwrite(texto_foto, src, compression_params);
            char texto_foto2[100];
            sprintf(texto_foto2,"SegmentROG_%i.png",cuadros);
            imwrite(texto_foto2, maskResult, compression_params);
        }  
        foto++;

   if(cont > 5){
    cont = 0;
   }
    */
   int c;
   c = waitKey( 20 );
   if( (char)c == 27 || cuadros == 2408){ 
        //wrtSRC.release();
        while(1){
            imshow( "Video", src );
            imshow("original",src0);
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

