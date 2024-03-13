
#include "Vector.h"
#include <Wire.h>
#include <Zumo32U4.h>
#include <Zumo32U4IMU.h>
#include <string.h>
#include <Timer.h>

const uint16_t maxSpeed = 250;
const uint16_t defaultSpeedPos = 100;
const uint16_t defaultSpeedNeg = -100;

const uint16_t threshold= 375;
uint16_t counter = 0;

Zumo32U4Buzzer buzzer;
Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;
Zumo32U4IMU intertialSensors;
Zumo32U4OLED display;


int storageArray[50]; 
uint32_t timeStorageArray[50];
Vector<int> turnMemory;
Vector<uint32_t> turnMemoryTime;

Timer timer(MILLIS);
uint32_t prevTime;

#define NUM_SENSORS 3

unsigned int lineSensorValues[NUM_SENSORS];
unsigned int prevLineSensorValues[NUM_SENSORS];


void setup() {
  // THIS Serial.begin ALLOWS US TO DEBUG
  Serial.begin(9600);

  turnMemory.setStorage(storageArray);
  turnMemoryTime.setStorage(timeStorageArray);

  lineSensors.initThreeSensors();

  //Song to ensure that the zumo has been flashed and is ready
  buzzer.playFrequency(440, 100, 15);

  buttonA.waitForButton();
  calibrateSensors();
  timer.start();
}

void loop() {

  lineSensors.read(lineSensorValues);
  prevTime = timer.read();
  if(counter >=3)
    revertFromMemory();
  
  if(lineSensorValues[0] > threshold && lineSensorValues[1] > threshold){
    turnMemoryTime.push_back(prevTime);
    turnMemory.push_back(0);
    turnRight();
  }

  if(lineSensorValues[2] > threshold && lineSensorValues[1] > threshold){
    turnMemoryTime.push_back(timer.read());
    turnMemory.push_back(1);
    turnLeft();
  }

  if(lineSensorValues[0] > threshold && lineSensorValues[1] <= threshold){
    bearRight();
  }

  if(lineSensorValues[2] > threshold && lineSensorValues[1] <= threshold){
    bearLeft();
  }

  motors.setSpeeds(defaultSpeedPos,defaultSpeedPos);
}

void turn180(){
    motors.setSpeeds(defaultSpeedNeg,defaultSpeedNeg);
    delay(300);
    motors.setSpeeds(0,0);
    delay(300);
    motors.setSpeeds(defaultSpeedNeg, defaultSpeedPos);
    delay(2000); // how long to turn
    counter++;
}

void bearLeft(){
  motors.setSpeeds(defaultSpeedNeg, defaultSpeedPos);
  delay(100); // how long to bear
}


void bearRight(){
  motors.setSpeeds(defaultSpeedPos, defaultSpeedNeg);
  delay(100); // how long to bear
}


void turnLeft(){
    motors.setSpeeds(defaultSpeedNeg,defaultSpeedNeg);
    delay(300);
    motors.setSpeeds(0,0);
    delay(300);
    motors.setSpeeds(defaultSpeedNeg, defaultSpeedPos);
    delay(750); // how long to turn
    counter++;
}

void turnRight(){
    motors.setSpeeds(defaultSpeedNeg,defaultSpeedNeg);
    delay(300);
    motors.setSpeeds(0,0);
    delay(300);
    motors.setSpeeds(defaultSpeedPos, defaultSpeedNeg);
    delay(750); // how long to turn
    counter++;
}

void calibrateSensors()
{

  // Wait 1 second and then begin automatic sensor calibration
  // by rotating in place to sweep the sensors over the line
  delay(1000);
  for(uint16_t i = 0; i < 120; i++)
  {
    if (i > 30 && i <= 90)
    {
      motors.setSpeeds(defaultSpeedNeg, defaultSpeedPos);
    }
    else
    {
      motors.setSpeeds(defaultSpeedPos, defaultSpeedNeg);
    }

    lineSensors.calibrate();
  }
  motors.setSpeeds(0, 0);
}

void revertFromMemory(){
  do{
  motors.setSpeeds(0,0);
  motors.setSpeeds(defaultSpeedNeg, defaultSpeedNeg);
  delay(turnMemoryTime[turnMemoryTime.size() - 1]);
  turnMemoryTime.pop_back();
  switch(turnMemory[turnMemory.size()]){
    case 0:
      turnLeft();
      break;
    case 1:
      turnRight();
      break;
  }
  turnMemory.pop_back();
  }while(!turnMemory.empty());
}
