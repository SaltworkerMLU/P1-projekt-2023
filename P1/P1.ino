#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4OLED display;
Zumo32U4Encoders encoders;
Zumo32U4IMU imu;
Zumo32U4Buzzer buzzer;
Zumo32U4ButtonA buttonA;

float boundsX;
float boundsY;
int state;
float lastPosition[3];

void screen(float line1, float line2) {
  display.clear();
  display.print(line1);
  display.gotoXY(0, 1);
  display.print(line2);
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
  float currentPosition[3] = { 0, 0, 0 };
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
    currentPosition[2] += Offset * 50 * (v2 - v1) * dt / (s / 10 * 1000000);
    if (currentPosition[2] < 0) {
      currentPosition[2] = currentPosition[2] + 360;
    }
    if (currentPosition[2] > 360) {
      currentPosition[2] = currentPosition[2] - 360;
    }
  }

  void findTargetAngle(float x_d, float y_d) {
    forwardKinematics(encoderData.velocity[0], encoderData.velocity[1], 0.95);
    float teta = (atan2(y_d, x_d)) / PI * 180;
    if (teta < 0) {
      teta += 360;
    }

    float turnAngle = teta - currentPosition[2];
    if (turnAngle < 0) {
      turnAngle += 360;
    }

    if (turnAngle <= 180) {
      while (turnAngle > 1) {
        motors.setSpeeds(-100, 100);
        forwardKinematics(encoderData.velocity[0], encoderData.velocity[1], 0.95);
        turnAngle = teta - currentPosition[2];
        if (turnAngle < 0) {
          turnAngle += 360;
        }
        screen(turnAngle, currentPosition[2]);
      }
      motors.setSpeeds(0, 0);
    }

    else if (turnAngle > 180) {
      while (turnAngle > 1) {
        motors.setSpeeds(100, -100);
        forwardKinematics(encoderData.velocity[0], encoderData.velocity[1], 0.95);
        turnAngle = teta - currentPosition[2];
        if (turnAngle < 0) {
          turnAngle += 360;
        }
        screen(turnAngle, currentPosition[2]);
      }
      motors.setSpeeds(0, 0);
    }
    screen(turnAngle, currentPosition[2]);
  }

  void driveStraight(float x_d, float y_d) {
    forwardKinematics(encoderData.velocity[0], encoderData.velocity[1], 0.95);

    if (currentPosition[0] < x_d) {
      while (currentPosition[0] < x_d) {
        motors.setSpeeds(100, 100);
        forwardKinematics(encoderData.velocity[0], encoderData.velocity[1], 0.95);
        screen(x_d, currentPosition[0]);
      }
      motors.setSpeeds(0, 0);
    }

    else if (currentPosition[0] > x_d) {
      while (currentPosition[0] > x_d) {
        motors.setSpeeds(100, 100);
        forwardKinematics(encoderData.velocity[0], encoderData.velocity[1], 0.95);
        screen(x_d, currentPosition[0]);
      }
      motors.setSpeeds(0, 0);
    }
  }

  void findDesiredAngle(float angle_d) {
    float turnAngle = angle_d - currentPosition[2];
    if (turnAngle < 0) {
      turnAngle += 360;
    }

    if (turnAngle <= 180) {
      while (turnAngle > 1) {
        motors.setSpeeds(-100, 100);
        forwardKinematics(encoderData.velocity[0], encoderData.velocity[1], 0.95);
        turnAngle = angle_d - currentPosition[2];
        if (turnAngle < 0) {
          turnAngle += 360;
        }
        screen(turnAngle, currentPosition[2]);
      }
      motors.setSpeeds(0, 0);
    }

    else if (turnAngle > 180) {
      while (turnAngle > 1) {
        motors.setSpeeds(100, -100);
        forwardKinematics(encoderData.velocity[0], encoderData.velocity[1], 0.95);
        turnAngle = angle_d - currentPosition[2];
        if (turnAngle < 0) {
          turnAngle += 360;
        }
        screen(turnAngle, currentPosition[2]);
      }
      motors.setSpeeds(0, 0);
    }
    screen(turnAngle, currentPosition[2]);
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

void stop() {
  motors.setSpeeds(0, 0);
}

void setup() {
  // put your setup code here, to run once:
}

void loop() {
  kinematics.backwardKinematics(10, 10, 0);

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
