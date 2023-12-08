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

float boundsX = 95.7;
float boundsY = 76;
int state;
float lastPosition[3];

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

  void turnByAngle(float turnAngle, float angle, int dir) {
    while (turnAngle >= 1) {
      motors.setSpeeds(dir * -106, dir * 100);
      forwardKinematics(encoderData.velocity[0], encoderData.velocity[1], 0.95);
      turnAngle = angle - currentPosition[2];
      if (turnAngle < 0) {
        turnAngle += 360;
      }
      screen(turnAngle, currentPosition[2]);
    }
    stop();
  }

  void findTargetAngle(float x_d, float y_d) {
    forwardKinematics(encoderData.velocity[0], encoderData.velocity[1], 0.95);
    float teta = (atan2(y_d, x_d)) / PI * 180;
    int dir = -1;
    if (teta < 0) {
      teta += 360;
    }

    float turnAngle = teta - currentPosition[2];
    if (turnAngle < 0) {
      turnAngle += 360;
    }

    if (turnAngle <= 180) {
      dir = 1;
    }

    turnByAngle(turnAngle, teta, dir);
  }


  void driveStraight(float x_d, float y_d) {
    forwardKinematics(encoderData.velocity[0], encoderData.velocity[1], 0.95);

    if (currentPosition[0] < x_d) {
      while (currentPosition[0] < x_d) {
        motors.setSpeeds(106, 100);
        forwardKinematics(encoderData.velocity[0], encoderData.velocity[1], 0.95);
        screen(x_d, currentPosition[0]);
      }
      stop();
    }

    else if (currentPosition[0] > x_d) {
      while (currentPosition[0] > x_d) {
        motors.setSpeeds(106, 100);
        forwardKinematics(encoderData.velocity[0], encoderData.velocity[1], 0.95);
        screen(x_d, currentPosition[0]);
      }
      stop();
    }
  }

  void findDesiredAngle(float angle_d) {
    float turnAngle = angle_d - currentPosition[2];
    int dir = -1;
    if (turnAngle < 0) {
      turnAngle += 360;
    }
    if (turnAngle <= 180) {
      dir = 1;
    }
    turnByAngle(turnAngle, angle_d, dir);
  }

  void backwardKinematics(float x_d, float y_d, float angle_d) {
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
        findDesiredAngle(angle_d);
        backwardStage++;
        break;
    }
  }
} kinematics;


void patrol() {
  if (treeDetected()) {
    stop();
    lastPosition[0] = kinematics.currentPosition[0];
    state++;
  } else {
    movement();
  }
}

void removeTree() {
  //turn to tree
  //move to tree
  //push tree out of bounds
}

void movement() {
}

bool treeDetected() {
  if (true) {
    return true;
  }
}



void setup() {
  Serial.begin(9600);
  turnSensorSetup();
}

void loop() {
  kinematics.backwardKinematics(38, 24, 180);

  /*
  switch (state){
    case 0: 
      patrol();
      break;
    case 1:
      removeTree();
      break;
    case 2:
      kinematics.backwardKinematics(lastPosition[0], lastPosition[1], lastPosition[2]);
      break;
  }
  */
}
