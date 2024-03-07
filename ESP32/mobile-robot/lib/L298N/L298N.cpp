#include "L298N.h"

L298N::L298N(uint8_t enable_A,uint8_t enable_B,uint8_t in1_A,uint8_t in2_A,uint8_t in1_B,uint8_t in2_B, uint8_t resolution=8)
            :enable_A(enable_A),enable_B(enable_B),in1_A(in1_A),in2_A(in2_A),in1_B(in1_B),in2_B(in2_B), resolution(resolution)

{
    pwm_A=0;
    pwm_B=0;
    motor_A_status = STOPPED;
    motor_B_status = STOPPED;
    setUpPinMode();
}

void L298N::setUpPinMode(){
    pinMode(enable_A,OUTPUT);
    pinMode(enable_B,OUTPUT);

    pinMode(in1_A,OUTPUT);
    pinMode(in2_A,OUTPUT);
    pinMode(in1_B,OUTPUT);
    pinMode(in2_B,OUTPUT); 

    ledcSetup(0, 1000, resolution);
    ledcSetup(1, 1000, resolution);

    ledcAttachPin(enable_A,0);
    ledcAttachPin(enable_B,1);
}

void L298N::setSpeed_A(uint16_t pwm){
    int resolution_limit = 1 << resolution;
    if(pwm > resolution_limit){
        pwm = resolution_limit;
    }else if(pwm < 0){
        pwm = 0;
    }
    pwm_A = pwm;
}
void L298N::setSpeed_B(uint16_t pwm){
    int resolution_limit = 1 << resolution;
    if(pwm > resolution_limit){
        pwm = resolution_limit;
    }else if(pwm < 0){
        pwm = 0;
    }
    pwm_B = pwm;
}
uint16_t L298N::getSpeedA(){
    return pwm_A;
}
uint16_t L298N::getSpeedB(){
    return pwm_B;
}
void L298N::moveForward_A(){
    digitalWrite(in1_A, HIGH);
    digitalWrite(in2_A, LOW);

    motor_A_status = FORWARD;
}

void L298N::moveForward_B(){
    digitalWrite(in1_B,HIGH);
    digitalWrite(in2_B, LOW);

    motor_B_status = FORWARD;
}

void L298N::moveBackward_A(){
    digitalWrite(in1_A, LOW);
    digitalWrite(in2_A, HIGH);

    motor_A_status = BACKWARD;
}

void L298N::moveBackward_B(){
    digitalWrite(in1_B,LOW);
    digitalWrite(in2_B, HIGH);

    motor_B_status = BACKWARD;
}

void L298N::updateSpeed_A(uint16_t pwm){
    setSpeed_A(pwm);
    ledcWrite(0, pwm_A);
}

void L298N::updateSpeed_B(uint16_t pwm){
    setSpeed_B(pwm);
    ledcWrite(1, pwm_A);
}

void L298N::stop_A(){
    digitalWrite(in1_A,LOW);
    digitalWrite(in2_A,LOW);

    motor_A_status = STOPPED;
}

void L298N::stop_B(){
    digitalWrite(in1_B,LOW);
    digitalWrite(in2_B,LOW);

    motor_B_status = STOPPED;
}

uint8_t L298N::getMotorAStatus(){
    return motor_A_status;
}

uint8_t L298N::getMotorBStatus(){
    return motor_B_status;
}