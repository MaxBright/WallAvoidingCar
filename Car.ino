/**Arduino self-driving car
  Copyright 2014 Max Bright. All Rights Reserved.
  
  Requires Adafruit Motorshield and NewPing libraries
  */
  
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <NewPing.h>
#include "utility/Adafruit_PWMServoDriver.h"

const int motorPinR =  5;   
const int motorPinL =  6;  
const int distanceLightPin = 10;
const int pollLightPin = 12;
const int buttonPin = 3;
const int powerButtonPin = 13;
const int distancePin = 7;
const int sensorPin = 7;

int powerButtonCheckCycle = 100;
unsigned long currentTime = 0;
unsigned long previousTime = 0;

char carState = 's';

int checkCycles[1];
unsigned long cyclePreviousTime[1];

bool firstLoop = true;
bool poweredOn = true;
bool powerButtonHeld = false;

bool checkCycle(int);
void threadedDelay(long, long);
bool checkPowerButton();

void startDriving(int);
void stopDriving();
void startTurn(int, char);

char turnDirection = 'r';
int forwardDriveCount = 0;

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *motor1;
Adafruit_DCMotor *motor2;
Adafruit_DCMotor *motor3;
Adafruit_DCMotor *motor4;

NewPing sensor = NewPing(sensorPin, sensorPin, 300);

void setup()   {                
  pinMode(motorPinR, OUTPUT); 
  pinMode(motorPinL, OUTPUT);  
  pinMode(distanceLightPin, OUTPUT);
  pinMode(pollLightPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  pinMode(powerButtonPin, INPUT);
  
  Serial.begin(9600);
  
  
  motor1 = AFMS.getMotor(1);
  motor2 = AFMS.getMotor(2);
  motor3 = AFMS.getMotor(3);
  motor4 = AFMS.getMotor(4);
  AFMS.begin();
  
  //Power button cycle
  checkCycles[0] = 100;
  cyclePreviousTime[0] = 0;
  
}

bool checkCycle(int cycleIndex)
{
  bool result;
  
  currentTime = millis();
  
  if (cyclePreviousTime[cycleIndex] + checkCycles[cycleIndex] <= currentTime)
  {
    cyclePreviousTime[cycleIndex] = currentTime;
    result = true;
  }
  else
  {
    result = false;
  }
  
  return result;
}

void threadedDelay(long delayTime, long initialTime = -1)
{
  long initialDelay;
  long loopStartTime;
  long elapsedTime;
  
  //This ensures program operates on an exact clock, in case certain
  //parts of the code take a long time to run while a function should
  //maintain the same total duration
  if (initialTime >= 0)
  {
    initialDelay = millis() - initialTime;
    
    delayTime = delayTime - initialDelay;
  }
  
  loopStartTime = millis();
  elapsedTime = 0;
  while (elapsedTime < delayTime)
  {
    if (checkCycle(0))
    {
      poweredOn = checkPowerButton();
    }
    delay(1);
    elapsedTime = millis() - loopStartTime;
  }
}

bool checkPowerButton()
{
  if (digitalRead(powerButtonPin) == LOW && !powerButtonHeld)
  {
    //Serial.println("powerButtonPin == LOW");
    if (!poweredOn)
    {
      poweredOn = true;
    }
    else
    {
      poweredOn = false;
    }
    powerButtonHeld = true;
  }
  else if (digitalRead(powerButtonPin) == HIGH)
  {
    //Serial.println("powerButtonPin == HIGH");
    powerButtonHeld = false;
  }
  return poweredOn;
}


bool checkDistance(int minRange = 15)
{
  bool result;
  
  long duration, cm;
  
  cm = sensor.ping_cm();
  
  Serial.println(cm);
  
  return (cm <= minRange && cm != 0);
}

void turn(char rot, int duration = 500)
{
  if (rot == 'r')
  {
    digitalWrite(motorPinL, HIGH);
    delay(duration);
    digitalWrite(motorPinL, LOW);
  }
  else if (rot == 'l')
  {
    digitalWrite(motorPinR, HIGH);
    delay(duration);
    digitalWrite(motorPinR, LOW);
  }
}

void startDriving(int motorSpeed = 255)
{
  //Serial.println("Called StartDriving");
  motor1->setSpeed(motorSpeed);
  motor1->run(FORWARD);
  motor2->setSpeed(motorSpeed);
  motor2->run(FORWARD);
  motor3->setSpeed(motorSpeed);
  motor3->run(FORWARD);
  motor4->setSpeed(motorSpeed);
  motor4->run(FORWARD);
  carState = 'f';
}

void startReversing(int motorSpeed = 255)
{
  motor1->setSpeed(motorSpeed);
  motor1->run(BACKWARD);
  motor2->setSpeed(motorSpeed);
  motor2->run(BACKWARD);
  motor3->setSpeed(motorSpeed);
  motor3->run(BACKWARD);
  motor4->setSpeed(motorSpeed);
  motor4->run(BACKWARD);
  carState = 'r';
}

void stopDriving()
{
  //Serial.println("Called StopDriving");
  motor1->run(RELEASE);
  motor2->run(RELEASE);
  motor3->run(RELEASE);
  motor4->run(RELEASE);
  carState = 's';
}

void startFastTurn(int motorSpeed = 127, float factor = 0.5, char dir = 'r')
{
  //Serial.println("Called StartFastTurn");
  float motorSpeedProduct = motorSpeed*factor;
  
  if (motorSpeedProduct > 255)
  {
    Serial.println("Warning: called startFastTurn with a factor too large");
    motorSpeedProduct = 255;
  }
  else if (motorSpeed * factor < 0)
  {
    Serial.println("Warning: called StartFastTurn with a factor too small");
    motorSpeedProduct = 0;
  }
  
  if (dir == 'r')
  {
    motor1->setSpeed(motorSpeed);
    motor2->setSpeed(motorSpeed*factor);
    motor3->setSpeed(motorSpeed*factor);
    motor4->setSpeed(motorSpeed);
  }
  else
  {
    motor1->setSpeed(motorSpeed*factor);
    motor2->setSpeed(motorSpeed);
    motor3->setSpeed(motorSpeed);
    motor4->setSpeed(motorSpeed*factor);
  }
  
  motor1->run(FORWARD);
  motor2->run(FORWARD);
  motor3->run(FORWARD);
  motor4->run(FORWARD);
  
  carState = 'm';
}

void startTurn(int motorSpeed = 127, char dir = 'r')
{
  //Serial.println("Called StartTurn");
  
  motor1->setSpeed(motorSpeed);
  motor2->setSpeed(motorSpeed);
  motor3->setSpeed(motorSpeed);
  motor4->setSpeed(motorSpeed);
  
  if (dir == 'r')
  {
    motor1->run(FORWARD);
    motor2->run(BACKWARD);
    motor3->run(BACKWARD);
    motor4->run(FORWARD);
  }
  else
  {
    motor1->run(BACKWARD);
    motor2->run(FORWARD);
    motor3->run(FORWARD);
    motor4->run(BACKWARD);
  }
  
  
  carState = 't';
}

char updateTurnDirection(int forwardDriveCount, int driveLength, char initialValue)
{
  Serial.println("Called UpdateTurnDirection");
  if (forwardDriveCount > driveLength)
  {
    if (initialValue == 'r')
      return 'l';
    else
      return 'r';
  }
}

void loop()                     
{
  unsigned long debugStartTime;
  unsigned long distanceCheckpoint;
  unsigned long distanceTime;
  unsigned long betweenTime;
  unsigned long debugCurrentTime;
  
  unsigned long initialTime;
  
  initialTime = millis();
  
  debugStartTime = millis(); 
  
  
  if (firstLoop)
  {
    delay(500);
    poweredOn = checkPowerButton();
    firstLoop = false;
  }
    
  if (!poweredOn)
  {
    if (carState == 's')
    {
      startDriving(255);
      carState = 'f';
    }
    
    bool throwaway;
    if (checkDistance(20))
    {
      if (carState != 'r')
      {
        turnDirection = updateTurnDirection(forwardDriveCount, 7, turnDirection);
        stopDriving();
        threadedDelay(100);
        startReversing(127);
      }
      forwardDriveCount = 0;
    }
    else if(checkDistance(60))
    {
      Serial.println("Within 75");
      
      if (carState != 't')
      {
        turnDirection = updateTurnDirection(forwardDriveCount, 7, turnDirection);
        digitalWrite(distanceLightPin, HIGH);
        stopDriving();
        threadedDelay(100);
        startTurn(255, turnDirection);
        carState = 't';
      }
      
      
      forwardDriveCount = 0;
    }
    else if (checkDistance(150))
    {
      if (carState != 'm')
      {
        turnDirection = updateTurnDirection(forwardDriveCount, 7, turnDirection);
        digitalWrite(distanceLightPin, HIGH);
        if (carState != 'f')
        {
          stopDriving();
          threadedDelay(100);
        }
        startFastTurn(255, 0.5, turnDirection);
        carState = 'm';
      }
      
      forwardDriveCount = 0;
    }
    else
    {
      digitalWrite(distanceLightPin, LOW);
      if (carState != 'f')
      {
        if (carState == 't' || carState == 'r')
        {
          stopDriving();
          threadedDelay(100);
        }
        startDriving(255);
        carState = 'f';
        
      }
      forwardDriveCount += 1;
      
    }
    
    distanceCheckpoint = millis();

    threadedDelay(200, initialTime);
    
    debugCurrentTime = millis();
    distanceTime = distanceCheckpoint - debugStartTime;
    betweenTime = debugCurrentTime - debugStartTime;

  }
  else
  {
    if (carState != 's')
      stopDriving();
    digitalWrite(distanceLightPin, HIGH);
    threadedDelay(1000);
    digitalWrite(distanceLightPin, LOW);
  }
  
}




