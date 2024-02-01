//Written by ESP Group 4 2022-2023
#pragma once
#include "mbed.h"
#include "ESP4MOTOR.h"

int main(){   

    Motor Mur;

    //Mur.bluetoothControl();           //Used for manual control (via Bluetooth)
    Mur.autoLineFollow();               //Follows line automatically
    
    while(1){


    }
    
    return 0;
}