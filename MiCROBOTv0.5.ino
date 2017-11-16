/*

MiCROBOTv0.5
Julkifli Awang Besar
2nd C0d3
PID for R0b0tr4c3r

Refer to Fritzing Schematic.
***ROBONEO2017****

*/
#include <QTRSensors.h>
#define NUM_SENSORS 6 
#define TIMEOUT 2500 
#define EMITTER_PIN 2 

#define rightMotor1 8 
#define rightMotor2 9 
#define rightMotorPWM 3 

#define leftMotor1 11 
#define leftMotor2 12 
#define leftMotorPWM 5 

#define motorPower 10 
#define rightBaseSpeed 100
#define leftBaseSpeed 100


QTRSensorsRC qtrrc((unsigned char[]) { A0, A1, A2, A3, A4, A5} , NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS]; 

void setup() { 

  pinMode(rightMotor1, OUTPUT); 
  pinMode(rightMotor2, OUTPUT); 
  pinMode(rightMotorPWM, OUTPUT); 
  pinMode(leftMotor1, OUTPUT); 
  pinMode(leftMotor2, OUTPUT); 
  pinMode(leftMotorPWM, OUTPUT); 
  pinMode(motorPower, OUTPUT); 
  
  int i; 
  for (int i = 0; i < 150; i++) 
    { 
    qtrrc.calibrate(); 
    digitalWrite(13, HIGH); delay(20); 
    
    } 
  
    digitalWrite(motorPower, LOW);
    digitalWrite(13, LOW); 
    delay(2000); 
    
  } 
    
    
    int lastError = 0; 
    float Kp = 0.08; 
    float Kd = 0.05; 
    float Ki = 0.001; 
    int rightMotorSpeed = 0; 
    int leftMotorSpeed = 0; 
    int integral = 0;

    void loop() { 
      unsigned int sensors[6]; 
      int position = qtrrc.readLine(sensors); 
      int error = position - 2500; 
      //integral += error; 
      
      int motorSpeed = Kp * error + Kd * (error - lastError)+ Ki * integral; 
      lastError = error;

      rightMotorSpeed = rightBaseSpeed + motorSpeed ; 
      leftMotorSpeed = leftBaseSpeed - motorSpeed ; 
      rightMotorSpeed = constrain(rightMotorSpeed, 0, 255); 
      leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);


      digitalWrite(motorPower, HIGH); 
      digitalWrite(rightMotor1, HIGH); 
      digitalWrite(rightMotor2, LOW); 
      analogWrite(rightMotorPWM, rightMotorSpeed); 
      digitalWrite(leftMotor1, HIGH); 
      digitalWrite(leftMotor2, LOW); 
      analogWrite(leftMotorPWM, leftMotorSpeed); 
      
  } 
      
      
 //************************ BASE CODE***********\\﻿


