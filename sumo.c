

#include <project.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Motor.h"
#include "Ultra.h"
#include "Reflectance.h"
#include "Gyro.h"
#include "Accel_magnet.h"
#include "LSM303D.h"
#include "IR.h"
#include "Beep.h"
#include "mqtt_sender.h"
#include <time.h>
#include <sys/time.h>
#include "serial1.h"
#include <unistd.h>
#include <stdlib.h>
#include <math.h> 

//sumo
void zmain(void)
{
    bool IRCheck = true;
    bool lineCheckActive = true;
    uint8 button;
    button = SW1_Read();
    long startTime , spinStart, spinEnd;
    bool endPress=false;
    
    //reflectance
    struct sensors_ ref;
    struct sensors_ dig;
    reflectance_start();
    reflectance_set_threshold(15000, 14400, 14300, 14100, 14300, 15500); // set center sensor threshold to 11000 and others to 9000
   
    vTaskDelay(1000);//task to make sure hardware initalize correctly
    int linesReached = 0;
    
    struct accData_ data = {0,0,0};
    
    motor_start();
    LSM303D_Start();
    IR_Start();
    Ultra_Start(); 
       
    // Hardware start functions 
    while(button==1){
        button = SW1_Read();
    }
    motor_forward(100,100);
    
    while(endPress==false){
        reflectance_read(&ref);
        reflectance_digital(&dig);
        if(lineCheckActive){
            //Line checker
            if(dig.l3==1 && dig.r3==1){
                linesReached++;
                lineCheckActive=false;
            }
        }
        if (linesReached == 1&& IRCheck==true) {
            //Code activates on first line and has IR waiting
            motor_forward(0,10);
            print_mqtt("Zumo047/ready" ,"line");
            startTime = xTaskGetTickCount();
            print_mqtt("Zumo047/start time" ,"%ld", startTime);
            IR_flush();
            IR_wait();
            IRCheck = false;
            spinStart = xTaskGetTickCount();
            motor_point_turn(160,160,130,1);
            motor_forward(160,1000);
            int d = Ultra_GetDistance();
            spinEnd=0;
            while(d>30 && spinEnd < 5000){
                //spin attack, sensor and timer based
                motor_point_turn(200,200,50,0);
                d = Ultra_GetDistance();
                spinEnd = xTaskGetTickCount() - spinStart;
                if(d<=30){
                    motor_forward(240,200);
                }
            }
            spinEnd=0;
            spinStart= xTaskGetTickCount();
        }
        if (IRCheck==true){
            //continues forward if hasn't hit first line
            motor_forward(100,10);
        }
        if(IRCheck==false){
            //line turner and rammer code based on reflectors and ultrasound
            int d = Ultra_GetDistance();
            reflectance_read(&ref);
            reflectance_digital(&dig);
            spinEnd = xTaskGetTickCount() - spinStart;
            if(dig.l2==1||dig.l1==1||dig.r1==1||dig.r2==1){
                motor_backward(190,150);
                motor_point_turn(200,200,500,1);
                motor_forward(210,10);
            } else if(dig.l3==1){
                motor_backward(190,150);
                motor_point_turn(200,200,300,1);
                motor_forward(210,10);
            } else if(dig.r3==1){
                motor_backward(190,150);
                motor_point_turn(200,200,300,0);
                motor_forward(210,10);
            } else if(d<10){
                //ram attack
                motor_forward(255,50);
            } else if(spinEnd>10000){
                //Spin attack
                spinEnd=0;
                d = Ultra_GetDistance();
                spinStart=xTaskGetTickCount();
                while(d>30 && spinEnd < 3000){
                    motor_point_turn(200,200,50,0);
                    d = Ultra_GetDistance();
                    spinEnd=xTaskGetTickCount() - spinStart;
                    if(d<=30){
                        motor_forward(240,200);
                    }
                }
                spinStart=xTaskGetTickCount();
                motor_forward(210,10);
            }
                
            //acclerator values 
            struct accData_ previousData;
            previousData = data;
            LSM303D_Read_Acc(&data);
        
            if (data.accX - previousData.accX > 9000 ){
                //hit detection and hit direction
                long hit = xTaskGetTickCount()-startTime;
                double direction = atan((data.accX - previousData.accX) / (data.accY-previousData.accY));
                direction = direction*(180/M_PI);
                direction=direction+90;
                if(data.accX>0&&data.accY<0){
                    direction=direction+90;
                }else if(data.accX>0&&data.accY>0){
                    direction = direction+180;
                }else if(data.accX<0&&data.accY>0){
                    direction = direction+270;
                }
                print_mqtt("Zumo047/hit" ,"%ld, %.2lf", hit, direction);
                //Movement reaction to hit
                if(direction>180){
                    motor_turn(60,220,300);
                    motor_forward(220, 50);
                }else {
                    motor_turn(220,60,300);
                    motor_forward(220, 50);          
                }
            }
            if(SW1_Read()==0){
                //Stops loop when button is pressed and bool is set
                endPress=true;
            }

        }
    }
    //ending process 
    motor_forward(0,10);
    motor_stop();
    long lap = xTaskGetTickCount();
    print_mqtt("Zumo047/stoptime" ,"%ld", lap);
    long run = xTaskGetTickCount() - startTime;
    print_mqtt("Zumo047/runtime" ,"%ld", run);
}