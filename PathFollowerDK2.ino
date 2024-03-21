#include <List.hpp>
#include <Wire.h>
#include <Zumo32U4.h>

// Different types of speeds so that I can seperate my 180-degree turns from my ZUMOs normal driving speed.
const uint16_t maxSpeed = 160;
const uint16_t max180 = 300;

#define NUM_SENSORS 5
unsigned int lineSensorValues[NUM_SENSORS];
unsigned char white_line = 1;


// This is the delay where I control how long the 180 / turns should take.
unsigned int turnDelay = 300;
unsigned int delay180 = 770;
unsigned int turnSpeed = 150;

// The baseline line-threshold of 1000 where I manipulate how sensitive my sensors should react to the line
const uint16_t lineThreshold = 1000;
// Essential variable so that I can control the space I give the ZUMO after hitting a black line.
unsigned int reverseSpeed = 70;
// Needed so that the ZUMO understands when to stop searching for houses and to return to base.
uint8_t objDetected = 0;
// Controls the speed at which the ZUMO scans the areas for houses.
const int calibrationSpeed = 250;

// The timers are used so that the difference can be calculated to see how many seconds has elapsed between scanning and driving.
unsigned long startTimer;
unsigned long lastTimer = 0;

// The highest number so that the sensors dont detect houses from far away.
const uint8_t sensorThreshold = 6;

// These lists are essential for returning back to base. Here is stored the direction of which the turns take, and the angles.
List <String> path_direction;
List <int> gyro_angles;


Zumo32U4Buzzer buzzer; // Used for sounds; Detects one or more houses
Zumo32U4ProximitySensors proxSensors; // Used to detect the houses
Zumo32U4LineSensors lineSensors; // Detect the black lines and obstacles (dashed lines)
Zumo32U4Motors motors; // Used to control the movement of the ZUMO
Zumo32U4ButtonA buttonA; // Start the ZUMO, initiating the Calibration phase.
Zumo32U4LCD display; 
Zumo32U4IMU imu; // Used along with TurnSensor to calculate the angle of each turn.
#include "TurnSensor.h"


const char fugue[] PROGMEM =
  "! O5 L16 agafaea dac+adaea fa<aa<bac#a dac#adaea f"
  "O6 dcd<b-d<ad<g d<f+d<gd<ad<b- d<dd<ed<f+d<g d<f+d<gd<ad"
  "L8 MS <b-d<b-d MLe-<ge-<g MSc<ac<a ML d<fd<f O5 MS b-gb-g"
  "ML >c#e>c#e MS afaf ML gc#gc# MS fdfd ML e<b-e<b-"
  "O6 L16ragafaea dac#adaea fa<aa<bac#a dac#adaea faeadaca"
  "<b-acadg<b-g egdgcg<b-g <ag<b-gcf<af dfcf<b-f<af"
  "<gf<af<b-e<ge c#e<b-e<ae<ge <fe<ge<ad<fd"
  "O5 e>ee>ef>df>d b->c#b->c#a>df>d e>ee>ef>df>d"
  "e>d>c#>db>d>c#b >c#agaegfe f O6 dc#dfdc#<b c#4";

// Adjust the sensors and their values.
void calibrateSensors()
{

  delay(1000);
  for(uint16_t i = 0; i < 120; i++)
  {
    if (i > 30 && i <= 90)
    {
      motors.setSpeeds(-200, 200);
    }
    else
    {
      motors.setSpeeds(200, -200);
    }

    lineSensors.calibrate();
  }
  motors.setSpeeds(0, 0);
}
// Initializes the necessary sensors, starts the calibration and the timer.
void setup() {
  turnSensorSetup();
  proxSensors.initFrontSensor();
  lineSensors.initFiveSensors();
  buttonA.waitForButton();
  calibrateSensors();
  lineSensors.readCalibrated(lineSensorValues);
  startTimer = millis();
  

}

void loop() {
  startTimer = millis();

  // Reads the line sensors to determine whether we hit a black line or not
  lineSensors.readLine(lineSensorValues,QTR_EMITTERS_ON,white_line);
  lineSensors.read(lineSensorValues);

  // This delay is needed to aid the dead-end detection.
  delay(30);

  // If the ZUMO detects two houses, attempt to go back to base, then play a tune and stop the ZUMO.
  if (objDetected == 2)
  {
    ReturnToStart();
    buzzer.play("!L16 V12 cdefgab>cbagfedc");
    delay(200);
    motors.setSpeeds(0,0);
    return;
  }
  // Start scanning for houses every n milliseconds
  if ((unsigned long)(startTimer - lastTimer) >= 6000)
  {
    Scan();
  }
  // If further left or right sensors trigger at the same time (deadend), 180 degree turn.
  if((lineSensorValues[0] > (lineThreshold-200)) && (lineSensorValues[4] > (lineThreshold-200)))
  {
    turn180();
  }
  // If the middle sensor is triggered, reverse and turn left.
  else if ((lineSensorValues[2] > (lineThreshold-400)))
  {
    reverseSpeed = reverseSpeed + 30;  // Reversing here is enhanced to further prevent overrunning the lines.
    turnLeft(false);
    reverseSpeed = reverseSpeed - 30;
  }
  // Turn left or right if either sensor is triggered.
  else if (lineSensorValues[0] > lineThreshold)
  {
    turnRight(false);
  }

  else if((lineSensorValues[4] > lineThreshold))
  {
    turnLeft(false);
  }
  else
  {
    motors.setSpeeds(maxSpeed,maxSpeed);
  }

}
// Function for the robot to reverse and turn right
void turnRight(bool returnStart)
{
  if(returnStart) //Boolean is necessary for when to turn to either find a house or return back to base.
  {
    uint16_t count = path_direction.getSize(); // Gets the size of the path_direction list.
    turnSensorReset(); //Resets the gyro to 0 so that it can turn accurately based on the gyro_angle.
    turnSensorUpdate();
    motors.setSpeeds(-reverseSpeed,-reverseSpeed); // Reverse to give the ZUMO space.
    delay(500);
    while( ((((int32_t)turnAngle >> 16) * 360) >> 16) != -(gyro_angles[count-1]) ) // While the ZUMO's angle is NOT the angle in the gyro_angles list ...
    { //-(gyro_angles[count-1]) we flip this into a negative due to going the opposite direction.
      Serial.println(((((int32_t)turnAngle >> 16) * 360) >> 16));
      turnSensorUpdate();
      motors.setSpeeds(turnSpeed+60,-turnSpeed+60); // Keep turning untill it hits the right degrees.
    }
    gyro_angles.removeLast(); // Removes that angle from the list.
    motors.setSpeeds(0, 0);
    delay(500);
    turnSensorReset(); // Resets the gyro sensors again.
  }
  else
  {
    motors.setSpeeds(-reverseSpeed,-reverseSpeed);
    delay(500);
    turnSensorReset();
    motors.setSpeeds(turnSpeed,-turnSpeed);

    uint32_t start = micros();
    unsigned long ms = turnDelay;

    // This while loop is necessary to track the angles whilst it makes the turn.
    // This is essentially the delay() function, but needed to create it manually to get the angles updated during the delay where the turn is made.
    while (ms > 0) {
        yield();
        while ( ms > 0 && (micros() - start) >= 1000) {
          turnSensorUpdate();
          ms--;
          start += 1000;
        }
    }   

    path_direction.add("RIGHT"); // Adds onto the list for the ReturnToStart function.
    gyro_angles.add(((((int32_t)turnAngle >> 16) * 360) >> 16)); // Adds the angle it made to make the turn.
    turnSensorReset(); // Resets the gyro sensors
  }
}
// Function for the robot to reverse and turn left
void turnLeft(bool returnStart)
{
  if(returnStart)
  {
    uint16_t count = path_direction.getSize();
    turnSensorReset();
    turnSensorUpdate();
    motors.setSpeeds(-reverseSpeed,-reverseSpeed);
    delay(500);
    while( ((((int32_t)turnAngle >> 16) * 360) >> 16) != -(gyro_angles[count-1]) )
    {
      Serial.println(((((int32_t)turnAngle >> 16) * 360) >> 16));
      turnSensorUpdate();
      motors.setSpeeds(-turnSpeed+20,turnSpeed+20);
    }
    gyro_angles.removeLast();
    motors.setSpeeds(0, 0);
    delay(500);
    turnSensorReset();
  }
  else
  {
    motors.setSpeeds(-reverseSpeed,-reverseSpeed);
    delay(500);
    turnSensorReset();
    motors.setSpeeds(-turnSpeed,turnSpeed);

    uint32_t start = micros();
    unsigned long ms = turnDelay;

    while (ms > 0) {
        yield();
        while ( ms > 0 && (micros() - start) >= 1000) {
          turnSensorUpdate();
          ms--;
          start += 1000;
        }
    }   
    
    path_direction.add("LEFT");
    gyro_angles.add(((((int32_t)turnAngle >> 16) * 360) >> 16));
    turnSensorReset();
  }




  //delay(turnDelay);
}
// Function for the robot to turn and do a 180; required for dead-ends.
void turn180()
{
  motors.setSpeeds(-max180,max180);
  delay(delay180);
  motors.setSpeeds(0,0);
  delay(300);
}
// Function to scan the area for houses. 
void Scan()
{
  motors.setSpeeds(0, 0);
  delay(100);
//  Uses the same for loop as the calibration function.
  for(uint16_t i = 0; i < 120; i++)
  {
    if (i > 30 && i <= 90)
    {
      motors.setSpeeds(-300, 300);
    }
    else
    {
      motors.setSpeeds(300, -300);
    }

    proxSensors.read();

    uint8_t leftValue = proxSensors.countsFrontWithLeftLeds();
    uint8_t rightValue = proxSensors.countsFrontWithRightLeds();
    // Uses the values of each sensor to detect if either has found a house.
    bool objectSeen = leftValue >= sensorThreshold || rightValue >= sensorThreshold;
    if (objectSeen)
    {
      Serial.println("OBJ FOUND");
      objDetected++;
      buzzer.playFrequency(240, 200, 15); // Play a sound to show a house has been found.
      motors.setSpeeds(0,0);
      if (objDetected == 2)
        {
          buzzer.playFrequency(440, 200, 15);
          motors.setSpeeds(0,0);
          delay(2000);
          turn180(); // Once two houses has been found, turn around and break the loop. 
          break;
        }
      delay(2000);
      turn180();
      break;
    }
    else if (leftValue >= (sensorThreshold-1) || rightValue >= (sensorThreshold-1)) // If either sensor has found an object, start adjustScan
    {
      Serial.println(leftValue);
      Serial.println(rightValue);
      
      motors.setSpeeds(0,0);
      delay(500);
      adjustScan(leftValue, rightValue);
      lastTimer = startTimer;
      break;
    }
  }
  lastTimer = startTimer;
}

// This helps the robot adjust its position so that it is facing the house.
bool adjustScan (uint8_t leftValue, uint8_t rightValue)
{
  bool objectSeen2;
  delay(500);
  Serial.println("ADJUST SCAN");
  for(uint16_t i = 0; i < 120; i++)
  {
    if (i > 30 && i <= 90)
    {
      motors.setSpeeds(-calibrationSpeed, calibrationSpeed);
    }
    else
    {
      motors.setSpeeds(calibrationSpeed, -calibrationSpeed);
    }

    proxSensors.read();

    uint8_t leftValue = proxSensors.countsFrontWithLeftLeds();
    uint8_t rightValue = proxSensors.countsFrontWithRightLeds();

    bool objectSeen3 = (leftValue == (sensorThreshold) && rightValue == (sensorThreshold));
    bool objectSeen2 = (leftValue >= (sensorThreshold-1) && rightValue >= (sensorThreshold-1));

    if(objectSeen3) // if objectSeen3 detects a house, with both values being at least 6, count that as a house being detected.
    {
      buzzer.playFrequency(240, 200, 15);
      motors.setSpeeds(0,0);
      delay(200);
      objDetected++;
      turn180();
      break;
    }

    else if(objectSeen2)   
    {
      break;
    }  
  }
  return objectSeen2;
}
// Function to go back to the starting base.
void ReturnToStart()
{
  uint16_t count = path_direction.getSize();
  do // While path_direction still has directions ... 
  {
    lineSensors.readLine(lineSensorValues,QTR_EMITTERS_ON,white_line);
    lineSensors.read(lineSensorValues); 
    motors.setSpeeds(maxSpeed,maxSpeed);
    if((lineSensorValues[0] > (lineThreshold-200)) || (lineSensorValues[4] > (lineThreshold-200)) || (lineSensorValues[2] > (lineThreshold-400)) ) // When either sensor is detected; Turn either left or right.
    {
      if(path_direction[count-1] == "LEFT")
      {
        turnRight(true);
        count--;
        path_direction.removeLast();

      }
      else
      {
        turnLeft(true);
        count--;
        path_direction.removeLast();
      }
    }
  }while(count != 0);
  motors.setSpeeds(0,0);
}