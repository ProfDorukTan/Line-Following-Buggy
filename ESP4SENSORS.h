//Written by ESP Group 4 2022-2023
#pragma once
#include "mbed.h"
#include "ESP4DEFINITIONS.h"

class sensor{
    //This class is written to control the line sensors, take reading from line sensors
        //and approximate the white line's relative position to the middle of the buggy using line sensor measurements
    public:
    sensor(){
        //Outputs for LED control
         o_sL3 = new DigitalOut(out_sensorL3);
         o_sL2 = new DigitalOut(out_sensorL2);
         o_sL1 = new DigitalOut(out_sensorL1);
         o_sR1 = new DigitalOut(out_sensorR1);
         o_sR2 = new DigitalOut(out_sensorR2);
         o_sR3 = new DigitalOut(out_sensorR3);
        //Inputs for phototransistor readings
         i_sL3 = new AnalogIn(in_sensorL3);
         i_sL2 = new AnalogIn(in_sensorL2);
         i_sL1 = new AnalogIn(in_sensorL1);
         i_sR1 = new AnalogIn(in_sensorR1);
         i_sR2 = new AnalogIn(in_sensorR2);
         i_sR3 = new AnalogIn(in_sensorR3);

        //Read sensors every 'sensor_read_period' seconds. The interrupt also calculates line position
        //sensorSampler.attach(callback(this, &sensor::Read_Sensors), sensor_read_period);

        //Initiate all LEDs to ON
         setL3(1);
         setL2(1);
         setL1(1);
         setR1(1);
         setR2(1);
         setR3(1);

        //Weight sensors according to their relative distance to the mid point - The array is used for line position calculation
        sensorweight[0] = -1 * ((sensor_distance / 2) + (sensor_distance * 2));
        sensorweight[1] = -1 * ((sensor_distance / 2) + (sensor_distance * 1));
        sensorweight[2] = 0;
        sensorweight[3] = 0;
        sensorweight[4] = +1 * ((sensor_distance / 2) + (sensor_distance * 1));
        sensorweight[5] = +1 * ((sensor_distance / 2) + (sensor_distance * 2));

        LineBreakCount = 0;
    }

    //Functions to turn on/off the LEDs
    void setL3(int num){
        if(num == 0 || num == 1){
        o_sL3->write(num); }}
    void setL2(int num){
        if(num == 0 || num == 1){
        o_sL2->write(num); }}
    void setL1(int num){
        if(num == 0 || num == 1){
        o_sL1->write(num); }}
    void setR1(int num){
        if(num == 0 || num == 1){
        o_sR1->write(num); }}
    void setR2(int num){
        if(num == 0 || num == 1){
        o_sR2->write(num); }}
    void setR3(int num){
        if(num == 0 || num == 1){
        o_sR3->write(num); }}
    
    //Functions to take the input from phototransistors
    float getL3() const{
        return sensorReadings[0]; }
    float getL2() const{
        return sensorReadings[1]; }
    float getL1() const{
        return sensorReadings[2]; }
    float getR1() const{
        return sensorReadings[3]; }
    float getR2() const{
        return sensorReadings[4]; }
    float getR3() const{
        return sensorReadings[5]; }
    //Function to return line's position
    float getLinePos() const{
        return LinePos;
    }

    protected:
    DigitalOut *o_sL3, *o_sL2, *o_sL1, *o_sR1, *o_sR2, *o_sR3;  //LEDs
    AnalogIn *i_sL3, *i_sL2, *i_sL1, *i_sR1, *i_sR2, *i_sR3;    //Phototransistors
    float sensorReadings[6];    //Storing sensor readings
    float sensors[6];           //Noise filtered sensor readings
    float sensorweight[6];      //Sensor's weights according to their position
    float LinePos;              //Line's position relative to the middle of the buggy (Left: negative - Right: positive)
    float totalSensorReading;
    int LineBreakCount;

    Ticker sensorSampler;

    void Read_Sensors(){
        //This function updates values of all 6 sensors between 0 - 1 
        sensorReadings[2] = (i_sL1->read());    //L1
        sensorReadings[3] = (i_sR1->read());    //R1

        sensorReadings[1] = (i_sL2->read());    //L2
        sensorReadings[4] = (i_sR2->read());    //R2

        sensorReadings[0] = (i_sL3->read());    //L3
        sensorReadings[5] = (i_sR3->read());    //R3

        Line_Pos();     //Immediately calling the function that calculates line position
    }

    void Line_Pos(){
        //This function approximates the line's relative position to the buggy 
        LinePos = 0;
        totalSensorReading = 0;

        //Avoid noise in sensors (ambient light) Any reading below the threshold defined is zeroed
        for (int i = 0; i < 6; i++){
            if (sensorReadings[i] >= sensor_threshold){
                sensors[i] = 1;
            }else{
                sensors[i] = 0;
            }
        }

        //Sensor readings are multiplied by sensor weights
        for (int i = 0; i < 6; i++){
            LinePos = LinePos + (sensors[i] * sensorweight[i]);
            totalSensorReading = totalSensorReading + sensors[i];
        }
        //If all sensors read below threshold ->Line break
        //If any sensor read above threshold ->Line position
        if(totalSensorReading == 0){
            //If no white line is detected
            LineBreakCount++;
            LinePos = 0;
        }else{
            LineBreakCount = 0;
            LinePos = LinePos / totalSensorReading;
        }

    }

};