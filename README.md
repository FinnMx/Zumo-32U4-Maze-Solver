# Zumo 32U4 Maze Solver
## Description

This repo consists of a single arduino .INO file. Inside the file you will find an arduino script designed to navigate a Zumo 32U4 around a maze by detecting the boundaries of the maze and avoiding it. This script is fairly basic as the limitations of the Zumos we had access to meant we couldn't fine tune an efficient approach. See the flowchart listed below to get a simple understanding of the robots event loop.

## Dependencies
### Libraries
- [Zumo32U4 Library ](https://github.com/pololu/zumo-32u4-arduino-library)
- [Vector Library](https://github.com/janelia-arduino/Vector)
- [Timer Library](https://github.com/sstaub/Timer)

### Editor
- [Arduino IDE](https://www.arduino.cc/en/software)

## References to code Snippets

- The Gyroscope / IMU setup routine is copied from the [Zumo 32U4 example .ino files](https://github.com/pololu/zumo-32u4-arduino-library/tree/master/examples).

## Event Loop Flowchart

![Zumo Loop drawio (1)](https://github.com/FinnMx/Zumo-32U4-Maze-Solver/assets/93927783/ce5053e6-fd61-4719-b3df-e10e4a56d528)


## Overview of key functions

### Setup
```C++
void setup() {
  Serial.begin(9600);

  turnMemory.setStorage(storageArray, 0);
  turnMemoryTime.setStorage(timeStorageArray, 0);

  lineSensors.initThreeSensors();
  proximitySensors.initThreeSensors();
  encoders.init();

  //Song to ensure that the zumo has been flashed and is ready
  buzzer.playFrequency(440, 100, 7);

  while (!buttonB.getSingleDebouncedRelease())
  {
    if(buttonA.getSingleDebouncedRelease()){
      targetHouses++;
      if(targetHouses > 2){
        targetHouses = 1;
      }

      for(uint8_t i = 0; i < targetHouses; i++){
        delay(200);
        buzzer.playFrequency(440, 100, 10);
      }
    }
  }

  buzzer.playFrequency(440, 500, 7);

  buttonA.waitForButton();
  turnSensorSetup();
  calibrateSensors();

  calibrateMotorSpeeds();

  timer.start();
}
```
This is a default arduino function that is ran before our main "loop". It initilisies our turnMemory Vectors so we can store our robots movements around the maze. We also call the default .init() functions for each zumo class object we have made. We then wait for the users input (pressign the A button) before calibrating our sensors.

### Loop
```C++
void loop() {
  
  turnSensorUpdate();
  proximitySensors.read();
  int16_t proxReadingL = proximitySensors.countsFrontWithLeftLeds();
  int16_t proxReadingR = proximitySensors.countsFrontWithRightLeds();

  lineSensors.read(lineSensorValues);
  prevTime = timer.read();

  if(proxReadingL >= 6 && proxReadingR >= 6) {  detectedHouses++; encounteredAHouse(prevTime); timer.stop(); timer.start(); }
  if(finished){ motors.setSpeeds(0,0); revertFromMemory(); return 0; }
  
  if(lineSensorValues[0] > threshold && lineSensorValues[1] > threshold && lineSensorValues[2] < threshold){
  counterL = 0;
  counterR++;
  if((counterR % 3) == 0){
    counterR = 0;
    turnMemoryTime.push_back(prevTime);
    turnMemory.push_back(1);
    reverse();
    turnLeft();
    timer.stop();
    timer.start();
  } else{
      turnMemoryTime.push_back(prevTime);
      turnMemory.push_back(0);
      reverse();
      turnRight();
      timer.stop();
      timer.start();
  }
  }
  else if((lineSensorValues[2] > threshold && lineSensorValues[1] > threshold && lineSensorValues[0] < threshold)
  || (lineSensorValues[2] > threshold && lineSensorValues[1] > threshold && lineSensorValues[0] > threshold)){
    counterR = 0;
    counterL++;
    if((counterL % 3) == 0){
       buzzer.playFrequency(440, 100, 15);
      counterL = 0;
      turnMemoryTime.push_back(prevTime);
      turnMemory.push_back(0);
      reverse();
      turnRight();
      timer.stop();
      timer.start();
    } else{
    turnMemoryTime.push_back(prevTime);
    turnMemory.push_back(1);
    reverse();
    turnLeft();
    timer.stop();
    timer.start();
    }
  }

  if(lineSensorValues[2] > threshold && lineSensorValues[1] <= threshold){
    delay(100);
    lineSensors.read(lineSensorValues);
    if(lineSensorValues[1] < threshold){
      turnMemoryTime.push_back(prevTime);
      turnMemory.push_back(2);
      bearLeft();
      timer.stop();
      timer.start();
    }
  }

  if(lineSensorValues[0] > threshold && lineSensorValues[1] <= threshold){
    delay(100);
    lineSensors.read(lineSensorValues);
    if(lineSensorValues[1] < threshold){
      turnMemoryTime.push_back(prevTime);
      turnMemory.push_back(3);
      bearRight();
      timer.stop();
      timer.start();
    }
  }
  
  motors.setSpeeds(leftPWMMotorSpeed,rightPWMMotorSpeed);
    
}
```
This loop function may seem rather long and cumbersome, but it is quite simple when broken down. On each loop we take a new reading from the line sensors, check if there is a object infront of us via the proximity sensors, and note the current time elapsed. We then have 4 if clauses, checking if any of our line sensors have passed a certain threshold. If they have we then call a certain movement function and push what we did into the turnMemory vector. This loop will run until we have found our house.

### Revert From Memory
```C++
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
    case 4:
    turn180();
    motors.setSpeeds(0,0);
    default:
      motors.setSpeeds(0,0);
      break;
    }
  motors.setSpeeds(-leftPWMMotorSpeed, -rightPWMMotorSpeed);
  delay(getRevertedDelayTime());
  motors.setSpeeds(0,0);

  turnMemoryTime.pop_back();
  turnMemory.pop_back();
  }
}

```

This function is called once we have found our house(s). It simpily reverses the turnMemoryVector by popping values off of it. The values popped off are the values recorded while trying to locate the house, so we do the opposite of each turn. We then call "getRevertedDelayTime()" to wait for the elapsed time that passed between turns recorded.
