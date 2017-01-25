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

double DC_X = 3;
double DC_Y = 3;
double tiempo = 1350000;

BlackPWM ServoX (P9_14);
BlackPWM ServoY (P8_19);

void Motores(float X, float Y, float espera);
void GIRO (float angulo);

int angulo;
double margen = 300000;
 
int main (void) {

    ServoX.setPeriodTime(20,milisecond);

    ServoY.setPeriodTime(20,milisecond);

    ServoY.setRunState(stop);    
    ServoX.setRunState(stop);
    
    while (1){
        cout << "----------" << endl;
        cin >> angulo;
        GIRO(angulo);
        Motores(DC_X, DC_Y, tiempo);
    /*
        int c;
        c = waitKey( 20 );
        if( (char)c == 27){ 
            ServoX.setRunState(stop);    
            ServoY.setRunState(stop);
            break;
        }
    */
    }
}

void GIRO (float angulo){
    if(angulo <= 60){  ////////// Menores a 60 grados /////////
        // GIRO 0° a 10°
        if(angulo >= 0 && angulo <= 10){
            DC_X = 3;
            DC_Y = 3;
            tiempo = 1350000;
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
            tiempo = 1350000;
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
    
    cout << angulo <<" grados";
    cout << " PWM X: " << ServoX.getValue();
    cout << "  PWM Y: " << ServoY.getValue() << endl;
    usleep(espera);
    ServoX.setRunState(stop);    
    ServoY.setRunState(stop);

    return;
}


