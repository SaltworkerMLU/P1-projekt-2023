#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4OLED display;
Zumo32U4Encoders encoders;
Zumo32U4IMU imu;
Zumo32U4Buzzer buzzer;
Zumo32U4ButtonA buttonA;

// variables for gyro
uint32_t turnAngle = 0;
int16_t turnRate;
int16_t gyroOffset;
uint16_t gyroLastUpdate = 0;

float boundsY = 95.7;
float boundsX = 76;
int state = 0;
float lastPosition[3];
int pathCount = 1;
int pathPart = 0;
int bitch = 0;

void screen(float line1, float line2) {
  display.clear();
  display.print(line1);
  display.gotoXY(0, 1);
  display.print(line2);
}

void stop() {
  motors.setSpeeds(0, 0);
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


struct encoderData {
  float distance[2];
  float velocity[2];
  long passedTime[2];
  float passedDistance[2];

  void getDistance() {
    distance[0] = 8 * acos(0.0) * encoders.getCountsLeft() / 900;
    distance[1] = 8 * acos(0.0) * encoders.getCountsRight() / 900;
  }

  void getVelocity() {
    getDistance();

    long dt = micros() - passedTime[0];
    passedTime[0] = micros();
    velocity[0] = 1000000 * (distance[0] - passedDistance[0]) / dt;
    passedDistance[0] = distance[0];

    dt = micros() - passedTime[1];
    passedTime[1] = micros();
    velocity[1] = 1000000 * (distance[1] - passedDistance[1]) / dt;
    passedDistance[1] = distance[1];
  }
} encoderData;


struct kinematics {
  float currentPosition[3] = { 7, 6.4, 0 };
  int r = 2;
  int s = 85;
  long excecutedTime = micros();
  int backwardStage = 0;

  void forwardKinematics(float v1, float v2, float Offset) {
    encoderData.getVelocity();
    long dt = micros() - excecutedTime;
    excecutedTime = micros();
    currentPosition[0] += Offset * (v1 + v2) * cos(currentPosition[2] * PI / 180) * dt / (2 * 1000000);  //funktion (9)
    currentPosition[1] += Offset * (v1 + v2) * sin(currentPosition[2] * PI / 180) * dt / (2 * 1000000);
    currentPosition[2] = getTurnAngleInDegrees();
    if (currentPosition[2] < 0) {
      currentPosition[2] = currentPosition[2] + 360;
    }
    if (currentPosition[2] > 360) {
      currentPosition[2] = currentPosition[2] - 360;
    }
  }

  void turnByAngle(float angle) {
    float turnAngle = angle - currentPosition[2];
    int dir = -1;
    if (turnAngle < 0) {
      turnAngle += 360;
    }
    if (turnAngle <= 180) {
      dir = 1;
    }
    while (turnAngle >= 1) {
      motors.setSpeeds(dir * -106, dir * 100);
      forwardKinematics(encoderData.velocity[0], encoderData.velocity[1], 0.95);
      turnAngle = angle - currentPosition[2];
      if (turnAngle < 0) {
        turnAngle += 360;
      }
      screen(currentPosition[0], currentPosition[1]);
    }
    stop();
  }

  void findTargetAngle(float x_d, float y_d) {
    x_d = x_d - currentPosition[0];
    y_d = y_d - currentPosition[1];
    forwardKinematics(encoderData.velocity[0], encoderData.velocity[1], 0.95);
    float teta = (atan2(y_d, x_d)) / PI * 180;
    int dir = -1;
    if (teta < 0) {
      teta += 360;
    }
    turnByAngle(teta);
  }

  void driveStraight(float x_d, float y_d) {
    forwardKinematics(encoderData.velocity[0], encoderData.velocity[1], 0.95);
    float xStart = currentPosition[0];
    float yStart = currentPosition[1];
    float distanceTotal = sqrt((x_d - xStart) * (x_d - xStart) + (y_d - yStart) * (y_d - yStart));
    float distanceDriven = sqrt((currentPosition[0] - xStart) * (currentPosition[0] - xStart) + (currentPosition[1] - yStart) * (currentPosition[1] - yStart));

    while (distanceTotal - distanceDriven > 0) {
      motors.setSpeeds(106, 100);
      forwardKinematics(encoderData.velocity[0], encoderData.velocity[1], 0.95);
      distanceDriven = sqrt((currentPosition[0] - xStart) * (currentPosition[0] - xStart) + (currentPosition[1] - yStart) * (currentPosition[1] - yStart));
      screen(currentPosition[0], currentPosition[1]);
    }
    stop();
    /*
    float xCurrent = x_d - currentPosition[0];
    float yCurrent = y_d - currentPosition[1];
    float difference = ((xCurrent + yCurrent)*(xCurrent + yCurrent));
    while (difference>1){
      motors.setSpeeds(106, 100);
      forwardKinematics(encoderData.velocity[0], encoderData.velocity[1], 0.95);
      xCurrent = x_d - currentPosition[0];
      yCurrent = y_d - currentPosition[1];
      difference = ((xCurrent + yCurrent)*(xCurrent + yCurrent));
      screen(lastPosition[0],lastPosition[1]);
    }
    stop();
    */
  }

  void backwardKinematics(float x_d, float y_d, float angle_d) {
    bool run = true;
    while (run) {
      forwardKinematics(encoderData.velocity[0], encoderData.velocity[1], 0.95);
      switch (backwardStage) {
        case 0:
          findTargetAngle(x_d, y_d);
          backwardStage++;
          break;
        case 1:
          driveStraight(x_d, y_d);
          backwardStage++;
          break;
        case 2:
          turnByAngle(angle_d);
          backwardStage++;
          break;
        case 3:
          backwardStage = 0;
          run = false;
      }
    }
  }

} kinematics;


void patrol() {
  while (kinematics.currentPosition[1] < boundsY && state == 0) {
    kinematics.forwardKinematics(encoderData.velocity[0], encoderData.velocity[1], 0.95);
    if (treeDetected()) {
      stop();
      state++;
    } else {
      switch (pathPart) {
        case 0:
          if (kinematics.currentPosition[0] < boundsX - 6.4) {
            motors.setSpeeds(106, 100);
            screen(kinematics.currentPosition[0], kinematics.currentPosition[1]);
          } else {
            kinematics.turnByAngle(90);
            pathPart++;
          }
          break;
        case 1:
          if (kinematics.currentPosition[1] < pathCount * 17.2) {
            motors.setSpeeds(106, 100);
            screen(kinematics.currentPosition[0], kinematics.currentPosition[1]);
          } else {
            kinematics.turnByAngle(180);

            pathCount++;
            pathPart++;
          }
          break;
        case 2:
          if (kinematics.currentPosition[0] > 6.4) {
            motors.setSpeeds(106, 100);
            screen(kinematics.currentPosition[0], kinematics.currentPosition[1]);
          } else {
            kinematics.turnByAngle(90);
            screen(kinematics.currentPosition[0], kinematics.currentPosition[1]);
            pathPart++;
          }
          break;
        case 3:
          if (kinematics.currentPosition[1] < pathCount * 17.2) {
            motors.setSpeeds(106, 100);
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
  stop();
}

void removeTree() {
  kinematics.backwardKinematics(40, 60, 0);
  stop();
}


bool treeDetected() {
  switch (bitch) {
    case 0:
      if (millis() > 12000) {
        bitch++;
        return true;
      } else {
        return false;
      }
      break;
    case 1:
      return false;
      break;
  }
}



void setup() {
  Serial.begin(9600);
  turnSensorSetup();
}



void loop() {
  switch (state) {
    case 0:
      patrol();
      break;
    case 1:
      stop();
      lastPosition[0] = kinematics.currentPosition[0];
      lastPosition[1] = kinematics.currentPosition[1];
      lastPosition[2] = kinematics.currentPosition[2];
      state++;
    case 2:
      removeTree();
      state++;
      break;
    case 3:
      kinematics.backwardKinematics(lastPosition[0], lastPosition[1], lastPosition[2]);
      state = 0;
      break;
  }
}
