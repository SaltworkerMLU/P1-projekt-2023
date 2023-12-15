#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4IMU imu;
Zumo32U4OLED display;
Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4ProximitySensors proxSensors;

// variables for gyro
uint32_t turnAngle = 0;
int16_t turnRate;
int16_t gyroOffset;
uint16_t gyroLastUpdate = 0;

float boundsY = 98.7;
float boundsX = 76;
int state = 0;
float lastPosition[3];
int pathCount = 1;
int pathPart = 0;
uint8_t value[6];
int speed = 100;
int turnDirection = 0;

//Prints two values on the screen
void screen(float line1, float line2) {
  display.clear();
  display.print(line1);
  display.gotoXY(0, 1);
  display.print(line2);
}

//stops
void stop() {
  motors.setSpeeds(0, 0);
}

//drives forward
void forward(int spdL = speed, int spdR = speed) {
  float power = (int16_t)readBatteryMillivolts();                                                                 // uint16_t -> int16_t -> float
  motors.setSpeeds(1.06 * spdL * pow(5000, 1.25) / pow(power, 1.25), spdR * pow(5000, 1.25) / pow(power, 1.25));  //scaled by 1.06 to account for drag of left motor and by the power level of the battery
}

/* 
Gyro setup and convenience functions '
*/
void turnSensorSetup() {
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.configureForTurnSensing();

  display.clear();
  display.print(F("Gyro cal"));

  // Turn on the yellow LED in case the LCD is not available.
  ledYellow(1);

  // Delay to give the user time to remove their finger.
  delay(500);

  // Calibrate the gyro.
  int32_t total = 0;
  for (uint16_t i = 0; i < 1024; i++) {
    // Wait for new data to be available, then read it.
    while (!imu.gyroDataReady()) {}
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
}

// This should be called to set the starting point for measuring
// a turn.  After calling this, turnAngle will be 0.
void turnSensorReset() {
  gyroLastUpdate = micros();
  turnAngle = 0;
}

// Read the gyro and update the angle.  This should be called as
// frequently as possible while using the gyro to do turns.
void turnSensorUpdate() {
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

uint32_t getTurnAngleInDegrees() {
  turnSensorUpdate();
  // do some math and pointer magic to turn angle in seconds to angle in degree
  return (((uint32_t)turnAngle >> 16) * 360) >> 16;
}

//reads all poximity sensors and saves the value to an array
void getProximity() {
  proxSensors.read();

  value[0] = proxSensors.countsLeftWithLeftLeds();
  value[1] = proxSensors.countsLeftWithRightLeds();
  value[2] = proxSensors.countsFrontWithLeftLeds();
  value[3] = proxSensors.countsFrontWithRightLeds();
  value[4] = proxSensors.countsRightWithLeftLeds();
  value[5] = proxSensors.countsRightWithRightLeds();
}

//struct for collecting and processing data from the encoders
struct encoderData {
  float distance[2];  //array with distance driven
  float velocity[2];  //array with current velocity
  long passedTime[2];
  float passedDistance[2];
  long deltaEncoders[2];  //saves encoder counts in type long to avoid overflow

  //function for finding driven distance
  void getDistance() {
    deltaEncoders[0] += encoders.getCountsAndResetLeft();  //saves encoder counts in type long to avoid overflow
    deltaEncoders[1] += encoders.getCountsAndResetRight();
    distance[0] = 8 * acos(0.0) * deltaEncoders[0] / 900;  //calculates distance driven from encoder counts
    distance[1] = 8 * acos(0.0) * deltaEncoders[1] / 900;
  }

  //function for calculating velocity of both motors
  void getVelocity() {
    getDistance();

    //calculates velocity of left motor
    long dt = micros() - passedTime[0];
    passedTime[0] = micros();
    velocity[0] = 1000000 * (distance[0] - passedDistance[0]) / dt;  //distance since last update divided by time since last update (times 1000000 to convert from cm/μs to cm/s)
    passedDistance[0] = distance[0];

    //calculates velocity of right motor
    dt = micros() - passedTime[1];
    passedTime[1] = micros();
    velocity[1] = 1000000 * (distance[1] - passedDistance[1]) / dt;
    passedDistance[1] = distance[1];
  }
} encoderData;

//struct for anything related to kinematics
struct kinematics {
  float currentPosition[3] = { 7, 6.4, 0 };  //start position of the zumo
  int r = 2;                                 //radius of the zumos wheels
  int s = 85;                                //distance between the belts of the zumo
  long excecutedTime = micros();
  int backwardStage = 0;

  //function for finding the position of the zumo in the virtual coordinate system
  void forwardKinematics() {
    encoderData.getVelocity();
    long dt = micros() - excecutedTime;
    excecutedTime = micros();
    currentPosition[0] += 0.95 * (encoderData.velocity[0] + encoderData.velocity[1]) * cos(currentPosition[2] * PI / 180) * dt / (2 * 1000000);  //function (9) of the kinematics section
    currentPosition[1] += 0.95 * (encoderData.velocity[0] + encoderData.velocity[1]) * sin(currentPosition[2] * PI / 180) * dt / (2 * 1000000);
    currentPosition[2] = getTurnAngleInDegrees();  //zumo angle = the angle read by the gyro (adds a little over time to  gyro drift)
  }

  //function for turning to a specific angle in relation to the virtual coordinate system
  void turnByAngle(float angle) {
    float turnAngle = angle - currentPosition[2];  //calculates difference between desired angle and the zumos current angle

    //finds which direction the zumo needs to turn for the shortest turn to desired angle
    int dir = -1;
    if (turnAngle < 0) {
      turnAngle += 360;
    }
    if (turnAngle <= 180) {
      dir = 1;
    }

    //the zumo turns until difference between desired angle and current angle of the zumo is less than 1
    while (turnAngle >= 1) {
      forward(dir * -speed, dir * speed);  //turn in chosen direction
      forwardKinematics();                 //updates position of the zumo

      //updates difference in angle
      turnAngle = angle - currentPosition[2];
      if (turnAngle < 0) {
        turnAngle += 360;
      }
      screen(currentPosition[0], currentPosition[1]);
    }
    stop();
  }

  //function for finding the angle between the zumos current position and the desired position and turns the zumo so its facing the desired point
  void findTargetAngle(float x_d, float y_d) {
    forwardKinematics();             //updates zumo position
    x_d = x_d - currentPosition[0];  //finds difference between current position and desired position
    y_d = y_d - currentPosition[1];
    float teta = (atan2(y_d, x_d)) / PI * 180;  //calculates angle between the two points

    //Turns angle into 360 instead of -180 to 180
    if (teta < 0) {
      teta += 360;
    }

    turnByAngle(teta);  // turns to face desired coordinates
  }

  //function for driving the zumo to desired coordinates
  void driveStraight(float x_d, float y_d) {
    forwardKinematics();                //updates zumo position
    float xStart = currentPosition[0];  //saves starting position of the drive
    float yStart = currentPosition[1];
    float distanceTotal = sqrt((x_d - xStart) * (x_d - xStart) + (y_d - yStart) * (y_d - yStart));                                                               //calculates distance from start position to desired position
    float distanceDriven = sqrt((currentPosition[0] - xStart) * (currentPosition[0] - xStart) + (currentPosition[1] - yStart) * (currentPosition[1] - yStart));  //calculates distance from start position to zumo

    //drives until distance from start to zumo is the same as distance from start to desired position
    while (distanceTotal - distanceDriven > 0) {
      forward();
      forwardKinematics();  //continuously updates zumo position and calculates distance from start position
      distanceDriven = sqrt((currentPosition[0] - xStart) * (currentPosition[0] - xStart) + (currentPosition[1] - yStart) * (currentPosition[1] - yStart));
      screen(currentPosition[0], currentPosition[1]);
    }
    stop();
  }

  //function for driving zumo to a specific position and angle in the virtual coordinate system
  void backwardKinematics(float x_d, float y_d, float angle_d) {
    forwardKinematics();
    findTargetAngle(x_d, y_d);  //finds the necessary angle for the zumo to be facing the desired point, and turns the zumo to face it
    driveStraight(x_d, y_d);    //drives forward until the zumo is at the desired coordinates
    turnByAngle(angle_d);       //turns the zumo to the desired angle
  }
} kinematics;

//function for patrolling the area with the zumo
void patrol() {
  //function runs as long as the zumo is within bounds y
  while (kinematics.currentPosition[1] < boundsY && state == 0) {
    kinematics.forwardKinematics();

    //if the proximitysensors detects a tree, the zumo saves its current position, breaks the patrol loop and moves on to the next state in the main loop
    if (treeDetected()) {
      stop();
      lastPosition[0] = kinematics.currentPosition[0];
      lastPosition[1] = kinematics.currentPosition[1];
      lastPosition[2] = kinematics.currentPosition[2];
      state++;
    }

    //here the zumo follows the predefined path
    else {
      switch (pathPart) {

        //the zumo drives until it hits the bounds and then turn to angle 90
        case 0:
          if (kinematics.currentPosition[0] < boundsX - 5) {
            forward();
            screen(kinematics.currentPosition[0], kinematics.currentPosition[1]);
          } else {
            kinematics.turnByAngle(90);
            pathPart++;
          }
          break;

        //The zumo drives for 17cm and then turns to angle 180
        case 1:
          if (kinematics.currentPosition[1] < pathCount * 17.2 + 6.4) {  //add 6.4 to account for the starting coordinates
            forward();
            screen(kinematics.currentPosition[0], kinematics.currentPosition[1]);
          } else {
            kinematics.turnByAngle(180);
            pathCount++;  //count how many times the zumo has driven in the y direction, to find the end coordinate of the next part
            pathPart++;
          }
          break;

        //same as case 0 but towards x=0 instead
        case 2:
          if (kinematics.currentPosition[0] > 8.4) {
            forward();
            screen(kinematics.currentPosition[0], kinematics.currentPosition[1]);
          } else {
            kinematics.turnByAngle(90);
            screen(kinematics.currentPosition[0], kinematics.currentPosition[1]);
            pathPart++;
          }
          break;
        //drive the next part in the y direction, turn to angle 0 and restart the patrol loop
        case 3:
          if (kinematics.currentPosition[1] < pathCount * 17.2 + 6.4) {
            forward();
            screen(kinematics.currentPosition[0], kinematics.currentPosition[1]);
          } else {
            kinematics.turnByAngle(0);
            pathCount++;
            pathPart = 0;
          }
          break;
      }
    }
  }
  stop();  //stop when the loop is finished (when the zumo reaches the bounds in the y direction)
  kinematics.forwardKinematics();
  if (kinematics.currentPosition[1] > boundsY) {
    kinematics.backwardKinematics(7, 6.4, 0);
    state = 3;
  }
}

//function for removing trees from the area
void removeTree() {
  int orientation = 90;  //target orientation for the zumo before turning to push the tree (could be optimised so that the zumo always pushes the tree the shortest possible distance)
  long dt;
  //The zumo drives in a curve around the tree, until it reaches the target orientation
  float turnAngle = 1;
  kinematics.forwardKinematics();

  dt = millis();
  while (millis() - dt < 300) {
    kinematics.forwardKinematics();
    forward();
  }

  while (turnAngle >= 1) {
    forward(100 - turnDirection * 60, 100 + turnDirection * 60);  //turn direction is found in the treeDetected() function
    kinematics.forwardKinematics();
    turnAngle = orientation - kinematics.currentPosition[2];
    if (turnAngle < 0) {
      turnAngle += 360;
    }
    screen(kinematics.currentPosition[0], kinematics.currentPosition[1]);
  }

  //the zumo turns 90 degrees in the turn direction from above, in order to face towards the tree
  stop();
  orientation += 90 * turnDirection;
  kinematics.turnByAngle(orientation);

  //the zumo drives forwards with the tree until it reaches a boundry line
  kinematics.forwardKinematics();
  while (5 < kinematics.currentPosition[0] && kinematics.currentPosition[0] < boundsX - 5 && 5 < kinematics.currentPosition[1] && kinematics.currentPosition[1] < boundsY - 5) {
    kinematics.forwardKinematics();
    forward();
    screen(kinematics.currentPosition[0], kinematics.currentPosition[1]);
  }
  stop();
  state++;  //move on to the next part of the main loop and end the removeTree loop
}

//function for detecting trees and setting turn direction
int treeDetected() {
  getProximity();  //reads proximity sensors

  //if left proximity sensor is above the threshold turnDirection is 1, and the function is true
  if (value[0] > 8) {
    turnDirection = 1;
    return true;
  }

  //Same but with right proximity sensor and turnDirection -1
  else if (value[5] > 8) {
    turnDirection = -1;
    return true;
  }

  //if neither side detects a tree, patrol loop continues
  else {
    return false;
  }
}

void setup() {
  Serial.begin(9600);  //setup serial connection

  turnSensorSetup();  //setup gyro

  //setup proximity sensors
  int brightnessLevels[20];
  for (int i = 0; i < 20; i++) {
    brightnessLevels[i] = 2 * i + 10;
  }
  proxSensors.setBrightnessLevels(brightnessLevels, 20);
  proxSensors.initThreeSensors();
  proxSensors.setPulseOffTimeUs(0);
  proxSensors.setPulseOnTimeUs(0);
  imu.configureForTurnSensing();
}

//main loop
void loop() {
  switch (state) {
    case 0:
      patrol();  //the zumo patrols the area until it sees a tree on either side
      break;
    case 1:
      removeTree();  //the zumo finds the tree and pushes it out of the area
      break;
    case 2:
      kinematics.backwardKinematics(lastPosition[0], lastPosition[1], lastPosition[2]);  //the zumo returns to its position before from before removing the tree, and goes back to patrol loop
      state = 0;
      break;
    case 3:
      stop();
      display.clear();
      display.print("Done!");
      delay(1000);
      break;
  }
}
