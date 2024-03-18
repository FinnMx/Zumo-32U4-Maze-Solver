#include "Vector.h"
#include <Wire.h>
#include <Zumo32U4.h>
#include <Zumo32U4IMU.h>
#include <string.h>
#include <Timer.h>

const uint16_t maxSpeed = 250;
const uint16_t defaultSpeedPos = 200;
const uint16_t defaultSpeedNeg = -200;

const uint16_t threshold= 375;
uint16_t counter = 0;

Zumo32U4Buzzer buzzer;
Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;
Zumo32U4IMU imu;
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

bool finished = false;



//==========================================================================
// TURN SENSOR TEST!!!!!!!!!

// Turnsensor.h provides functions for configuring the
// Zumo 32U4's gyro, calibrating it, and using it to
// measure how much the robot has turned about its Z axis.
//
// This file should be included once in your sketch,
// somewhere after you define objects named buttonA,
// display, and imu.

// This constant represents a turn of 45 degrees.
const int32_t turnAngle45 = 0x20000000;

// This constant represents a turn of 90 degrees.
const int32_t turnAngle90 = turnAngle45 * 2;

// This constant represents a turn of approximately 1 degree.
const int32_t turnAngle1 = (turnAngle45 + 22) / 45;

/* turnAngle is a 32-bit unsigned integer representing the amount
the robot has turned since the last time turnSensorReset was
called.  This is computed solely using the Z axis of the gyro, so
it could be inaccurate if the robot is rotated about the X or Y
axes.

Our convention is that a value of 0x20000000 represents a 45
degree counter-clockwise rotation.  This means that a uint32_t
can represent any angle between 0 degrees and 360 degrees.  If
you cast it to a signed 32-bit integer by writing
(int32_t)turnAngle, that integer can represent any angle between
-180 degrees and 180 degrees. */
uint32_t turnAngle = 0;
uint32_t targetTurnAngle = 0;

// turnRate is the current angular rate of the gyro, in units of
// 0.07 degrees per second.
int16_t turnRate;

// This is the average reading obtained from the gyro's Z axis
// during calibration.
int16_t gyroOffset;

// This variable helps us keep track of how much time has passed
// between readings of the gyro.
uint16_t gyroLastUpdate = 0;


// This should be called to set the starting point for measuring
// a turn.  After calling this, turnAngle will be 0.
void turnSensorReset()
{
  gyroLastUpdate = micros();
  turnAngle = 0;
}

// Read the gyro and update the angle.  This should be called as
// frequently as possible while using the gyro to do turns.
void turnSensorUpdate()
{
  // Read the measurements from the gyro.
  imu.readGyro();
  turnRate = imu.g.z - gyroOffset;

  // Figure out how much time has passed since the last update (dt)
  uint16_t m = micros();
  uint16_t dt = m - gyroLastUpdate;
  gyroLastUpdate = m;

  // Multiply dt by turnRate in order to get an estimation of how
  // much the robot has turned since the last update.
  // (angular change = angular velocity * time)
  int32_t d = (int32_t)turnRate * dt;

  // The units of d are gyro digits times microseconds.  We need
  // to convert those to the units of turnAngle, where 2^29 units
  // represents 45 degrees.  The conversion from gyro digits to
  // degrees per second (dps) is determined by the sensitivity of
  // the gyro: 0.07 degrees per second per digit.
  //
  // (0.07 dps/digit) * (1/1000000 s/us) * (2^29/45 unit/degree)
  // = 14680064/17578125 unit/(digit*us)
  turnAngle += (int64_t)d * 14680064 / 17578125;
}

/* This should be called in setup() to enable and calibrate the
gyro.  It uses the display, yellow LED, and button A.  While the
display shows "Gyro cal", you should be careful to hold the robot
still.

The digital zero-rate level of the gyro can be as high as
25 degrees per second, and this calibration helps us correct for
that. */
void turnSensorSetup()
{
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.configureForTurnSensing();

  display.clear();
  display.print(F("Gyro cal"));

  // Turn on the yellow LED in case the display is not available.
  ledYellow(1);

  // Delay to give the user time to remove their finger.
  delay(500);

  // Calibrate the gyro.
  int32_t total = 0;
  for (uint16_t i = 0; i < 1024; i++)
  {
    // Wait for new data to be available, then read it.
    while(!imu.gyroDataReady()) {}
    imu.readGyro();

    // Add the Z axis reading to the total.
    total += imu.g.z;
  }
  ledYellow(0);
  gyroOffset = total / 1024;

  // Display the angle (in degrees from -180 to 180) until the
  // user presses A.
  display.clear();
  turnSensorReset();
  while (!buttonA.getSingleDebouncedRelease())
  {
    turnSensorUpdate();
    display.gotoXY(0, 0);
    display.print((((int32_t)turnAngle >> 16) * 360) >> 16);
    display.print(F("   "));
  }
  display.clear();
  targetTurnAngle = turnAngle;
}


//======================================================================
//TEST OVER



void setup() {
  // THIS Serial.begin ALLOWS US TO DEBUG
  Serial.begin(9600);

  turnMemory.setStorage(storageArray, 0);
  turnMemoryTime.setStorage(timeStorageArray, 0);

  lineSensors.initThreeSensors();

  //Song to ensure that the zumo has been flashed and is ready
  buzzer.playFrequency(440, 100, 7);

  buttonA.waitForButton();
  turnSensorSetup();
  calibrateSensors();

  timer.start();
}

void loop() {
  if(finished){ motors.setSpeeds(0,0); return 0; }
  Serial.println(turnAngle);

  lineSensors.read(lineSensorValues);
  prevTime = timer.read();

  
  if(counter >= 3){
    motors.setSpeeds(0, 0);
    revertFromMemory();
    return 0;
  }
  
  //checkTurnAngleSame();
  
  if(lineSensorValues[0] > threshold && lineSensorValues[1] > threshold){
      Serial.println("right turn");
      targetTurnAngle += -turnAngle45;
    turnMemoryTime.push_back(prevTime);
    turnMemory.push_back(0);
    reverse();
    turnRight();
  }else if(lineSensorValues[2] > threshold && lineSensorValues[1] > threshold){
    Serial.println("left turn");
    targetTurnAngle += turnAngle45;
    turnMemoryTime.push_back(prevTime);
    turnMemory.push_back(1);
    reverse();
    turnLeft();
  }
/*
  if(lineSensorValues[2] > threshold && lineSensorValues[1] <= threshold){
    delay(20);
    if(lineSensorValues[1] > threshold){ return; }
    turnMemoryTime.push_back(prevTime);
    turnMemory.push_back(2);
    bearLeft();
  }

    if(lineSensorValues[0] > threshold && lineSensorValues[1] <= threshold){
    delay(20);
    if(lineSensorValues[1] > threshold){ return; }
    turnMemoryTime.push_back(prevTime);
    turnMemory.push_back(3);
    bearRight();
  }
  */

  motors.setSpeeds(defaultSpeedPos,defaultSpeedPos);
}

void reverse(){
    motors.setSpeeds(defaultSpeedNeg,defaultSpeedNeg);
    delay(300);
}

void shiftForwards(){
    motors.setSpeeds(defaultSpeedPos,defaultSpeedPos);
    delay(300);
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
  counter++;
}

void bearRight(){
  motors.setSpeeds(defaultSpeedPos, defaultSpeedNeg);
  delay(100); // how long to bear
  counter++;
}

void turnLeft(){
    turnSensorReset();
    motors.setSpeeds(0,0);
    delay(300);
  motors.setSpeeds(defaultSpeedNeg, defaultSpeedPos);
  while((int32_t)turnAngle < turnAngle45 * 2)
  {
    turnSensorUpdate();
  }
  motors.setSpeeds(0,0);
    counter++;
}

void turnRight(){
    turnSensorReset();
    motors.setSpeeds(0,0);
    delay(300);
  motors.setSpeeds(defaultSpeedPos, defaultSpeedNeg);
  while((int32_t)turnAngle > -turnAngle45 * 2)
  {
    turnSensorUpdate();
  }
  motors.setSpeeds(0,0);
    counter++;
}

void calibrateSensors()
{

  // Delay so the robot does not move while the user is still
  // touching the button.
  delay(1000);

  // We use the gyro to turn so that we don't turn more than
  // necessary, and so that if there are issues with the gyro
  // then you will know before actually starting the robot.

  turnSensorReset();

  // Turn to the left 90 degrees.
  motors.setSpeeds(defaultSpeedNeg, defaultSpeedPos);
  while((int32_t)turnAngle < turnAngle45 * 2)
  {
    lineSensors.calibrate();
    turnSensorUpdate();
  }

  // Turn to the right 90 degrees.
  motors.setSpeeds(defaultSpeedPos, defaultSpeedNeg);
  while((int32_t)turnAngle > -turnAngle45 * 2)
  {
    lineSensors.calibrate();
    turnSensorUpdate();
  }

  // Turn back to center using the gyro.
  motors.setSpeeds(defaultSpeedNeg, defaultSpeedPos);
  while((int32_t)turnAngle < 0)
  {
    lineSensors.calibrate();
    turnSensorUpdate();
  }

  // Stop the motors.
  motors.setSpeeds(0, 0);
}

void revertFromMemory(){
  while(turnMemory.size() != 0){
    switch(turnMemory[turnMemory.size() - 1]){
    case 0: // left turn
      turnLeft();
      shiftForwards();
      break;
    case 1: // right turn
      turnRight();
      shiftForwards();
      break;
    case 2:
    bearRight();
    motors.setSpeeds(0,0);
      break;
    case 3:
    bearLeft();
    motors.setSpeeds(0,0);
    break;
    default:
      motors.setSpeeds(0,0);
      break;
    }
  motors.setSpeeds(defaultSpeedNeg, defaultSpeedNeg);
  //  Serial.println(getRevertedDelayTime());
  delay(getRevertedDelayTime());
  motors.setSpeeds(0,0);

  turnMemoryTime.pop_back();
  turnMemory.pop_back();
<<<<<<< Updated upstream
  //Serial.println(turnMemory.size());
=======
>>>>>>> Stashed changes
  }
  finished = true;
}

uint32_t getRevertedDelayTime(){
  if(turnMemoryTime[turnMemoryTime.size() - 2] > 60000){
    if(turnMemoryTime[turnMemoryTime.size() - 1] > 60000){
      return 0;
    } 
    else{
      Serial.println(turnMemoryTime[turnMemoryTime.size() -1]);
      return turnMemoryTime[turnMemoryTime.size() -1];
    }
  }
  Serial.println(turnMemoryTime[turnMemoryTime.size() -1] -  turnMemoryTime[turnMemoryTime.size() - 2]);
  return turnMemoryTime[turnMemoryTime.size() -1] -  turnMemoryTime[turnMemoryTime.size() - 2];
  /*
  switch(turnMemoryTime.size()){
    case -1:
    //need to reset the storage here
      return 0;
    break;
    default:
      return turnMemoryTime[turnMemoryTime.size() - 1] - turnMemoryTime[turnMemoryTime.size() - 2];
    break;
  }
<<<<<<< Updated upstream
  //Serial.println("TEST \n" + (turnMemoryTime[turnMemoryTime.size() - 1]) - (turnMemoryTime[turnMemoryTime.size() - 2]));
}
=======
  */
}
>>>>>>> Stashed changes
