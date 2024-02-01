//Written by ESP Group 4 2022-2023
#pragma once
#include "mbed.h"
#include "ESP4DEFINITIONS.h"
#include "ESP4ENCODER.h"
#include "ESP4SENSORS.h"

class Motor : public encoder, public sensor{
    //Main class for motor control
    public:
        Motor() : encoder(), sensor(){
            bluetooth = new Serial(TXpin, RXpin);                                                   //Bluetooth serial config
            bluetooth->baud(9600);                                                                  //Set the baud rate to 9600
            BluetoothPrintout.attach(callback(this, &Motor::printout), datasendfrequency);          //Send data to bluetooth every 'datasendfrequency' seconds
                        
            pc = new Serial(USBTX, USBRX);                                                          //USB Serial Config - Can be used for testing
            pc->baud(9600);                                                                         //Set the baud rate to 9600

            //Motor initialization
            pwm1 = new PwmOut(PWMpin1);                             //Motor1 PWM
            pwm2 = new PwmOut(PWMpin2);                             //Motor2 PWM
            
            pwm1->period(pwmPeriod);                                //Set Motor1 pwm period 
            pwm2->period(pwmPeriod);                                //Set Motor2 pwm period 

            pwm1->write(1.0f);                                      //Motor1 initial duty cycle - STOP
            pwm2->write(1.0f);                                      //Motor2 initial duty cycle - STOP

            direction1 = new DigitalOut(directionpin1);             //Direction1 digital
            direction2 = new DigitalOut(directionpin2);             //Direction2 digital

            direction1->write(1);                                   //Direction1 initialization
            direction2->write(1);                                   //Direction2 initialization

            mode1 = new DigitalOut(modepin1);                       //Mode1 digital
            mode2 = new DigitalOut(modepin2);                       //Mode2 digital

            mode1->write(modeselect);                               //Mode1 initialization
            mode2->write(modeselect);                               //Mode2 initialization

            enable = new DigitalOut(enablepin);                     //Enable digital

            enable->write(1);                                       //Enable initialization
        }

    //Functions to read all Motor related Values
    float getPWM1() const{
        return pwm1->read(); }
    float getPWM2() const{
        return pwm2->read(); }
    
    int getDIRECTION1() const{
        return direction1->read(); }
    int getDIRECTION2() const{
        return direction2->read(); }
    
    int getMODE1() const{
        return mode1->read(); }
    int getMODE2() const{
        return mode2->read(); }

    //Functions to change Motor related values -> PWM, Direction, Mode
    void setPWM(float pwmvar){
        //Changes both motor PWMs
        if(pwmvar > MAXPWM){
        pwm1->write(pwmvar);
        pwm2->write(pwmvar);
        } }
    void setPWM1(float pwmvar1){
        //Change motor1 PWM
        if(pwmvar1 > MAXPWM){
        pwm1->write(pwmvar1);
        } }
    void setPWM2(float pwmvar2){
        //Change motor2 PWM
        if(pwmvar2 > MAXPWM){
        pwm2->write(pwmvar2);
        } }
    
    void setDIRECTION(int direction){
        //Changes both motor Directions
        if(direction == 0 || direction == 1){
        direction1->write(direction);
        direction2->write(direction);
        } }
    void setDIRECTION1(float direction){
        //Changes motor1 Direction
        if(direction == 0 || direction == 1){
        direction1->write(direction);
        } }
    void setDIRECTION2(float direction){
        //Changes motor2 Direction
        if(direction == 0 || direction == 1){
        direction2->write(direction);
        } }

    void setMODE(int mode){
        //Changes motors mode - NOT RECCOMENDED
        if(mode == 0 || mode == 1){
        mode1->write(mode);
        mode2->write(mode);
        } }

    //Functions for manual control
    void t_RIGHT(){                          
        //Speeds up left motor for a set amount
        pwm2->write(getPWM2() - (turnSensitivity));
        wait_ms(100);
        setPWM(getPWM1());
    }
    void t_LEFT(){                              
        //Speeds up right motor for a set amount
        pwm1->write(getPWM1() - (turnSensitivity));
        wait_ms(100);
        setPWM(getPWM2());
    }

    //Bluetooth Control  
    void printout(){
    //For debugging purposes, printsout values using serial comms - This Function is called using ticked and used for debugging
            //bluetooth->printf("Kp: %f, error: %f, var: %f\n", Kp, error, var);
            // bluetooth->printf("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f || %.2f %.2f || %.2f, %.2f, %.2f, %.2f, %.2f, %.2f \n", 
            //         getL3(), getL2(), getL1(), getR1(), getR2(), getR3(),
            //                  Kp, Kd,
            //                    sensors[0], sensors[1], sensors[2], sensors[3], sensors[4], sensors[5] );
            //bluetooth->printf("%f, %f\n", getTOTALVELOCITY(), var);
            // bluetooth->printf("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f || line: %f, Break: %d|| Speed: %f, Kp: %.3f, Kd: %.3f\n", 
            //         getL3(), getL2(), getL1(), getR1(), getR2(), getR3(),
            //                 getLinePos(), LineBreakCount, constantSpeed, Kp, Kd);
            bluetooth->printf("Kp: %.4f Kd: %.2f, line: %f, Speed: %.2f => %.2f || LBR: %d || PWM: %f, %f\n", 
                    Kp, Kd, LinePos, getTOTALVELOCITY(), constantSpeed, LineBreakCount, getPWM1(), getPWM2());
            // bluetooth->printf("%f, %f, %f, %f, %f, %f\n", sensors[0], sensors[1], sensors[2], sensors[3], sensors[4], sensors[5]);
            //bluetooth->printf("%.1f, %.1f, %.1f, %.1f, %.1f, %.1f\n", sensorweight[0], sensorweight[1], sensorweight[2], sensorweight[3], sensorweight[4], sensorweight[5]);
            //bluetooth->printf("LinePos: %.2f, Kp: %f, error %f, Var: %f || R: %.2f => %.2f, L: %.2f => %.2f \n", getLinePos(), Kp, error, var, 
                //getPWM1(), ConstantPWM_Right, getPWM2(), ConstantPWM_Left);
            //bluetooth->printf("pwm1: %.3f, pwm2: %.3f\n", getPWM1(), getPWM2());
            //bluetooth->printf("direction1: %d, direction2: %d\n", getDIRECTION1(), getDIRECTION2());
            //bluetooth->printf("mode1: %d, mode2: %d\n", getMODE1(), getMODE2());
            //bluetooth->printf("TR1: %.3f, TR2: %.3f, P1: %d, P2: %d\n", getTICKRATE1(), getTICKRATE2(), enc1->getPulses(), enc2->getPulses());
            //bluetooth->printf("Velocity1: %.3f, Velocity2: %.3f, %f, %f\n", getVELOCITY1(), getVELOCITY2(), TargetSpeed_Right, TargetSpeed_Left);
            //bluetooth->printf("totalV: %f, transV: %f, angularV: %f\n\n", getTOTALVELOCITY(), getTRANSLATIONALVELOCITY(), getANGULARVELOCITY());
    }

    void bluetoothControl() {
        //This function is for manually controlling the buggy using bluetooth
        char data;

        while(1) {
            data = NULL;
            if (bluetooth->readable()) {
                data = bluetooth->getc();
            }
            switch (data) {
                //The motor is called using the ESP4DEFINITIONS.h defined data signals
                case bluetoothEnd:
                    //To end bluetooth manual control
                    setMODE(0);
                    setPWM(1);
                    bluetooth->printf("\033c");
                    bluetooth->printf("Bluetooth Control Cancelled\n");
                    return;
                
                case speedup:
                    setPWM(getPWM1() - pwmSensitivity);
                    break;

                case slowdown:
                    setPWM(getPWM1() + pwmSensitivity);
                    break;

                case breakkey:
                    setPWM(1);
                    break;

                case turnleft:
                    t_LEFT();
                    break;

                case turnright:
                    t_RIGHT();
                    break;

                case changedirection:
                    setDIRECTION(!getDIRECTION1());
                    break;
                    
                default:
                    break;
            }
        }
    }

    void autoLineFollow(){
        //This function is for auto line following buggy
        setPWM(1);
        Kp = 15.15;               //initial Kp
        Kd = 1.38;               //initial Kd
        Kp_speed = 0.001;
        constantSpeed = 0;      //Constant speed is used by speed control algorithm
        char data;              //Data that is sent by bluetooth
        while(1){
            data = NULL;
            if (bluetooth->readable()) {
                data = bluetooth->getc();
            }
            switch (data){
                case 'p':
                    //Used for PID Tuning, changes the Kp
                    Kp += 0.1;
                    break;

                case 'l':
                    //Used for PID Tuning, changes the Kp
                    if(Kp >= 0){
                        Kp -= 0.1;
                    }
                    break;
                case 'r':
                    //Used for PID Tuning, changes the Kd
                    Kd += 0.01;
                    break;

                case 'o':
                    //Used for PID Tuning, changes the Kd
                    if(Kd >= 0){
                        Kd -= 0.01;
                    }
                    break;

                case speedup:
                    //Speed control -> Speed increase
                    constantSpeed += 0.7;
                    break;

                case slowdown:
                    //Speed control -> Speed decrease
                    if (constantSpeed >= 0.3) {
                        constantSpeed -= 0.3;
                    }else{
                        constantSpeed = 0;
                    }
                    break;

                case breakkey:
                    //Speed control -> STOP
                    constantSpeed = 0;
                    setPWM(1);
                    break;

                case 't':
                    //Turn interrupt - Polling
                    //Turn Around
                    setPWM(1);
                    setDIRECTION1(0);

                    while(LineBreakCount < 2000){
                        //Turn Around until no line is detected
                        sensor::Read_Sensors();
                        setPWM1(0.65);
                        setPWM2(0.65);
                    }
                    while(LineBreakCount != 0){
                        //Turn Around until a line is detected
                        sensor::Read_Sensors();

                        setPWM1(0.65);
                        setPWM2(0.65);
                    }
                    while (getLinePos() > 0.02 || getLinePos() < -0.02){
                        //Turn Around until the line is in the middle
                        sensor::Read_Sensors();
                        setPWM1(0.75);
                        setPWM2(0.75);  
                    }

                    setPWM(1);
                    setDIRECTION1(1);
                    
                    break;


                default:
                    //Default case is if no signal is sent --> LINE FOLLOW USING PID
                    sensor::Read_Sensors();     //READ SENSORS

                    prev_error = error;
                    error = getLinePos();       //Sets the line's position using sensors

                    if(getTOTALVELOCITY() > 0.001 && LineBreakCount > 4000){
                        //Line end detection. If LineBreakCount < 10000 it is a line break not line end
                        setPWM(1);
                        constantSpeed = 0;
                    }
                    if (constantSpeed != 0){
                        //PD CONTROL is buggy is moving
                        dt = timer.read();
                        var = (Kp * error) + (Kd * (error - prev_error)/dt);         //PD

                    }else{
                        var = 0;
                    }         
                    timer.reset();                  
                    timer.start();
                    //Change motor speeds according to PID output. If line is mid sends 'constantSpeed'
                    TargetSpeed_Left = ((constantSpeed/2) + (var));             
                    TargetSpeed_Right = ((constantSpeed/2) + (-1 * var));

                    speedController();

                    break;
            }
        }
    }

    void speedController(){
        //Basic speed controller - Checks speed, compares with target speed - speeds up or slows down
 
        error_speed_right =  TargetSpeed_Right - getVELOCITY1();
        error_speed_left = TargetSpeed_Left - getVELOCITY2();
        integral_speed_right += error_speed_right * dt;
        integral_speed_left += error_speed_left * dt;

        if (TargetSpeed_Right != 0 && TargetSpeed_Left != 0) {
            var_speed_right = (Kp_speed*error_speed_right) + (0.001*integral_speed_right);
            var_speed_left = (Kp_speed*error_speed_left) + (0.001*integral_speed_left);

            setPWM1(getPWM1() - (var_speed_right));
            setPWM2(getPWM2() - (var_speed_left));
        }else{
            setPWM1(1);
            setPWM2(1);
        }



    }


    private:
        Serial *bluetooth, *pc;
        PwmOut *pwm1, *pwm2;
        DigitalOut *direction1, *direction2, *mode1, *mode2, *enable;
        
        Ticker BluetoothPrintout;
        Timer timer;

        float error, prev_error;                                        //Errors for control algorithm
        float constantSpeed, TargetSpeed_Right, TargetSpeed_Left;       //Speed control vars
        float Kp, Kd, var, dt;                         //PID variables  
        float Kp_speed, error_speed_right, error_speed_left, var_speed_right, var_speed_left, integral_speed_right, integral_speed_left;
};