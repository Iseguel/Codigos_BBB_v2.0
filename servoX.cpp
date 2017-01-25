#include <iostream>
#include "BlackPWM.h"
#include <unistd.h>

using namespace BlackLib;
using namespace std;
 
int main (void) {

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
    
  for(int i = 0; i<3 ;i++){
    //servo X
    ServoX.setRunState(run);
    DC_X = 3;
    for (int i = 0; i<90; i++)
        {
        DC_X = DC_X + 0.1;    
        ServoX.setDutyPercent (DC_X);
        usleep(100000);
        cout << "PWM X: " << ServoX.getValue() << endl;
        }
    
    for (int i = 0; i<90; i++)
        {
        DC_X = DC_X - 0.1;
        ServoX.setDutyPercent (DC_X);
        usleep(100000);    
        cout << "Valor del PWM X: " << ServoX.getValue() << endl;
        }
    ServoX.setRunState(stop);
 
    //servo Y
    ServoY.setRunState(run);
        
    DC_Y = 3;    
    for (int i = 0; i<90; i++)
        {
        DC_Y = DC_Y + 0.1;
        ServoY.setDutyPercent (DC_Y);
        usleep(100000);            
        cout << "PWM Y: " << ServoY.getValue() << endl;
        }
    
    for (int i = 0; i<90; i++)
        {
        DC_Y = DC_Y - 0.1;
        ServoY.setDutyPercent (DC_Y);
        usleep(100000);    
        cout << "PWM Y: " << ServoY.getValue() << endl;
        }   
    ServoY.setRunState(stop);
    sleep(1);
    }

    //ServoY.setRunState(stop);
    //ServoX.setRunState(stop);
}

