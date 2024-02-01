//Written by ESP Group 4 2022-2023
#pragma once
#include "mbed.h"

//All customization should be done in ESP4DEFINITIONS.h file.

//SECURITY
#define MAXPWM 0.3                  //Used by test functions to limit max PWM duty cycle

//1 is Right - 2 is Left through the whole code notation

//Motor Control Pin Declerations
#define PWMpin1 PC_9
#define PWMpin2 PC_8
#define modepin1 PB_9
#define modepin2 PC_5
#define directionpin1 PB_8
#define directionpin2 PC_6
#define enablepin PC_2

//Motor Control Variable initializations
#define pwmPeriod 0.005             //period in seconds
#define modeselect 0                //0 - unipolar 1 - bipolar      DO NOT CHANGE
#define pwmSensitivity 0.02         //Used for manual control - [0,1] - 0 no sensitivity, 1 max sensitivity Reccomend: [0.05-0.1]
#define turnSensitivity 0.3         //Used for manual control - [0,1] - 0 no sensitivity, 1 max sensitivity Reccomend: [0.1-0.4]
//#define speedControllerPeriod 0.0005 //Speed controller interrupt is called every 'speedControllerPeriod' seconds

//Encoder Pin Declerations
#define channelA1 PC_10
#define channelB1 PC_12
#define channelA2 PC_11
#define channelB2 PD_2

//Encoder Variable initializations
#define wheel_diameter 0.0776               //in meters
#define gear_ratio 0.06666                  //output speed / input speed
#define CPR 1024                            //X4 encoding: 1024 pulses per revolution
#define distance_btw_wheels 0.0186          //in meters
#define PI 3.141592
#define encoderSampleTime 0.0005            //in seconds
#define FilterConstant 0.3                  //Increase to avoid more noise, but add delay [0, 1] - Reccomend: 0.3

//Bluetooth Pin Declerations
#define RXpin PA_12
#define TXpin PA_11

//Bluetooth Variable initializations
#define datasendfrequency 0.5               //Data is being sent by microcontroller every 'datasendfrequency' seconds

#define bluetoothEnd 'q'
#define speedup 'w'
#define slowdown 's'
#define breakkey ' '
#define turnright 'd'
#define turnleft 'a'
#define changedirection 'x'


//Line-Sensors Pin Declerations
    //Output LED
#define out_sensorL3 PB_12
#define out_sensorL2 PB_2
#define out_sensorL1 PB_1
#define out_sensorR1 PB_15
#define out_sensorR2 PB_14
#define out_sensorR3 PB_13
    //Input Transistor
#define in_sensorL3 PA_1
#define in_sensorL2 PA_0
#define in_sensorL1 PB_0
#define in_sensorR1 PA_4
#define in_sensorR2 PC_1
#define in_sensorR3 PC_4

//Line-Sensors Variable initializations 
#define sensor_read_period 0.0005            //Sensors get reading every 'sensor_read_period' seconds
#define sensor_threshold 0.55               //Threshold to avoid noise and ambient light
#define sensor_distance 0.014               //in meters, distance between each sensor