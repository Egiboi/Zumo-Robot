#include <project.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Motor.h"
#include "Reflectance.h"
#include "IR.h"
#include "mqtt_sender.h"


//Zumo line follower Team Stone - Jetro, Daniel, Jalmari
void zmain(void)
{
    //0: middle sensors are on the line, 1: both middle sensors are outside the line
    int outsideLine = 0;
    //previous turning direction, used as a fallback when the sensors can't see the line. 0 (left) or 1 (right)
    int lastDir = 0;
    //line counter
    int linesReached = 0;
    //the end of the track
    const int endLine = 3;

    //target power (motor speed when the robot is not turning)
    const int tp = 220;
    //error proportion multiplier
    const int kp = 50;
    //derivative multiplier smoothes the turning curve
    const int kd = 60;
    //calculated motor speed correction when the robot is turning
    int speedCorrection = 0;
    //error during the previous iteration
    long lastError = 0;

    //true: listening for IR commands
    bool IRCheck = true;
    //makes sure that the crossed lines are counted just once
    bool lineCheckActive = true;
    //lap start time (timestamp, milliseconds)
    long lapStartTime;
    uint8 button = SW1_Read();

    //reflectance
    struct sensors_ ref;
    struct sensors_ dig;
    reflectance_start();

    const int blackValue = 24000; //max sensor value
    const int whiteValue = 5500;
    int sensorRange = blackValue - whiteValue;
    int midThreshold = (blackValue + whiteValue) / 2;
    reflectance_set_threshold(15000, midThreshold, midThreshold, midThreshold, midThreshold, 15500);

    motor_start();
    IR_Start();

    //looping until the button is pressed
    while (button == 1)
    {
        button = SW1_Read();
    }

    motor_forward(80, 1);
    while (linesReached < endLine)
    {
        if (linesReached == 1 && IRCheck == true)
        {
            print_mqtt("Zumo047/ready", "line");
            IR_flush();
            IR_wait();
            IRCheck = false;
            lapStartTime = xTaskGetTickCount();
            print_mqtt("Zumo047/start", "%ld", lapStartTime);
        }

        reflectance_read(&ref);
        reflectance_digital(&dig);

        //checking when the inner sensors get outside the line and return
        if (outsideLine == 1 && dig.l1 == 1 && dig.r1 == 1)
        {
            print_mqtt("Zumo047/line", "%ld", xTaskGetTickCount());
            outsideLine = 0;
        }
        else if (outsideLine == 0 && dig.l1 == 0 && dig.r1 == 0)
        {
            print_mqtt("Zumo047/miss", "%ld", xTaskGetTickCount());
            outsideLine = 1;
        }

        //counting the crossed lines (outer sensors on the line)
        if (lineCheckActive == false)
        {
            if (dig.l3 == 0 && dig.r3 == 0)
                lineCheckActive = true;
        }

        if (lineCheckActive && dig.l3 == 1 && dig.r3 == 1)
        {
            linesReached += 1;
            lineCheckActive = false;
            if (linesReached < endLine)
            {
                motor_forward(0, 0);
            }
        }

        if (IRCheck == false)
        {
            //counting the total error (deviation from the aimed direction)
            float error_l2 = 1.8f * (ref.l2 - whiteValue);
            float error_l1 = 1 * (ref.l1 - whiteValue);
            float error_r1 = 1 * (ref.r1 - whiteValue);
            float error_r2 = 1.8f * (ref.r2 - whiteValue);
            float error = (error_l2 + error_l1 - error_r1 - error_r2) / (sensorRange);

            //keeping track of the last known turn direction
            lastDir = (error > 0) ? 0 : 1;

            //calculating the correction to the motor speeds
            speedCorrection = (int)(kp * error + kd * (error - lastError));
            lastError = error;

            //adjusting and clamping the motor speed values (0-255)
            int leftSpeed = tp - speedCorrection;
            int rightSpeed = tp + speedCorrection;

            if (leftSpeed < 0)
            {
                leftSpeed = 0;
            }
            if (rightSpeed < 0)
            {
                rightSpeed = 0;
            }

            int leftOver = 0;
            int rightOver = 0;

            if (leftSpeed > 255)
            {
                leftOver = (leftSpeed - 255);
                leftSpeed = 255;
                rightSpeed -= (2 * leftOver);
                if (rightSpeed < 0)
                {
                    rightSpeed = 0;
                }
            }
            else if (rightSpeed > 255)
            {
                rightOver = (rightSpeed - 255);
                rightSpeed = 255;
                leftSpeed -= (2 * rightOver);
                if (leftSpeed < 0)
                {
                    leftSpeed = 0;
                }
            }

            //setting the left and right motor speeds based on calculated error value and adjustments
            //max turn when all of the inner 4 sensors are outside the line (until finding the line again)
            if (dig.l2 == 1 || dig.l1 == 1 || dig.r1 == 1 || dig.r2 == 1)
            {
                motor_turn(leftSpeed, rightSpeed, 10);
            }
            else
            {
                while (!(dig.l1 == 1 || dig.r1 == 1 || dig.l2 == 1 || dig.r2 == 1))
                {
                    reflectance_read(&ref);
                    reflectance_digital(&dig);
                    if (lastDir == 0)
                    {
                        motor_turn(0, 255, 2);
                    }
                    else
                    {
                        motor_turn(255, 0, 2);
                    }
                }
            }
        }
    }
    motor_stop();
    print_mqtt("Zumo047/stop", "%ld", xTaskGetTickCount());
    long lapTime = xTaskGetTickCount() - lapStartTime;
    print_mqtt("Zumo047/time", "%ld", lapTime);
}
