//Written by ESP Group 4 2022-2023
#pragma once
#include "mbed.h"
#include "ESP4DEFINITIONS.h"
#include "QEI.h"

class encoder{
    //When this class is initiated 2 encoder readings are taken and converted to velocities.
    public:
    encoder(){
        //Encoder object setup
        enc1 = new QEI(channelA1, channelB1, NC, CPR, QEI::X4_ENCODING);
        enc2 = new QEI(channelA2, channelB2, NC, CPR, QEI::X4_ENCODING);     
        //Encoder pulses reset
        enc1->reset();
        enc2->reset();

        //Main interrupt to get encoder readings
        encoderSampler.attach(callback(this, &encoder::updater), encoderSampleTime);
    }

    void updater(){
        //Public function to update all values - Called by interrupt
        velocityCalc();
    }

    //Functions to read any individual value related to encoders
    float getTICKRATE1() const{
        return filteredTR1; }
    float getTICKRATE2() const{
        return filteredTR2; }
    float getVELOCITY1() const{
        return velocity1; }
    float getVELOCITY2() const{
        return velocity2; }
    float getTOTALVELOCITY() const{
        return totalV; }
    float getANGULARVELOCITY() const{
        return angularV; }
    float getTRANSLATIONALVELOCITY() const{
        return transV; }
    
    protected:
    float tickRate1, tickRate2, velocity1, velocity2, totalV, transV, angularV;     //Calculated variables
    QEI *enc1, *enc2;                                                               //Encoder objects
    Ticker encoderSampler;

    //Filter variables
    float filteredTR1, filteredTR2, tickRate1old, tickRate2old;
    float filteredValue;


    float smoothing_filter(float inputValue, float &filteredValueOld) {
        //Function to filter noise from the encoder reading using the previous reading -> f(t) = f(t-1)(a) + f(t)(1-a)
        filteredValue  =   FilterConstant  *   inputValue    +   (1-FilterConstant)  *   filteredValueOld;
        filteredValueOld  =   filteredValue;

        return filteredValue;
    }

    void tickRateCalc(){
        //Function to calculate tickrate values using the pulse reading from encoders. The function is called every 'encoderSampleTime' seconds         
        tickRate1 = static_cast<float>(-1 * enc1->getPulses());     //Pulse reading1
        tickRate2 = static_cast<float>(enc2->getPulses());          //Pulse reading2

        enc1->reset();
        enc2->reset();

        tickRate1 = (tickRate1 / encoderSampleTime);                //Tick rate reading1
        tickRate2 = (tickRate2 / encoderSampleTime);                //Tick rate reading1

        filteredTR1 = smoothing_filter(tickRate1, tickRate1old);    //Filtered tick rate1
        filteredTR2 = smoothing_filter(tickRate2, tickRate2old);    //Filtered tick rate2

        
    }

    void velocityCalc(){
        //Function to calculate velocity of wheels and total velocities. Updates all values.
        tickRateCalc();     //First calls the tickRateCalc() function to update tickrates

        velocity1 = (filteredTR1 / CPR) * (wheel_diameter * PI);        //Velocity 1
        velocity2 = (filteredTR2 / CPR) * (wheel_diameter * PI);        //Velocity2

        transV = (velocity1 + velocity2) / 2;                           //Translational velocity
        angularV = (velocity1 - velocity2) / distance_btw_wheels;       //Angular velocity - Right dominant - Turn left dominant
        totalV = (velocity1 + velocity2);                               //Total velocity
    }
};