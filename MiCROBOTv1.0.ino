/*
MiCROBOT V1

Julkifli Awang Besar
2nd C0d3
PID for R0b0tr4c3r

Refer to Fritzing Schematic.
***ROBONEO2017****

*/
#include <QTRSensors.h>
#define NUM_SENSORS 6 
#define TIMEOUT 2500 
#define EMITTER_PIN 11 

#define rightMotor1 3
#define rightMotor2 4 
#define rightMotorPWM 6 

#define leftMotor1 7 
#define leftMotor2 8
#define leftMotorPWM 5 

#define motorPower 9


#define rightBaseSpeed 120
#define leftBaseSpeed 120


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
    float Kp = 0.01; 
    float Kd = 3.00; 
    float Ki = 0.000; 
    int rightMotorSpeed = 0; 
    int leftMotorSpeed = 0; 
    int integral = 0;

    void loop() { 
      unsigned int sensors[6]; 
      int position = qtrrc.readLine(sensors); 
      
      //for white Line
      //unsigned int position = qtrrc.readLine(sensorValues, QTR_EMITTERS_ON, 1);
      
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


