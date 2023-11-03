#include <Zumo32U4.h>
Zumo32U4Encoders motorEncoders;

/*! This struct represents the encoders, aka. the motor turns, based on wheel circumference, measurer */
struct encoders {
  float distance[2];    // Motor distance in centimeters cm. distance[0] = left motor; distance[1] = right motor
  float velocity[2];    // Motor velocity in centimeters per second cm/s. velocity[0] = left motor; velocity[1] = right motor
  long oldTime[2];      // Stores past time in micros() after the respective motor velocity has been updated
  float oldDistance[2]; // Stores past distance after the respective motor velocity has been updated

  /*! Updates distance[2] and calculates distance in cm from the "ticks" of each encoder */
  void getDistance() { 
    distance[0] = 8 * acos(0.0) * motorEncoders.getCountsLeft() / 900; // [...] = 8 * acos(0.0) * count / 900... CPR=900 table value: https://www.pololu.com/docs/0J63/3.4
    distance[1] = 8 * acos(0.0) * motorEncoders.getCountsRight() / 900; // If distance does not measure up, adjust using class variable CPR.
  }

  /*! Updates velocity[2] and calculates velocity in cm/s. Also calculates distance[2] */
  void getVelocity() {
  getDistance(); // distance[2] is needed to get velocity = distance / time

  // Left motor velocity is calcluated
  long dt = micros() - oldTime[0]; // Get difference of time in microseconds micros() = new time; encoderTime = past time
  velocity[0] = 1000000 * (distance[0] - oldDistance[0])/dt; // 1s = 1000000µs [a million microseconds]. Velocity (cm/µs) is converted to (cm/s).
  oldDistance[0] = distance[0]; // Update past distance
  oldTime[0] = micros(); // Update past time

  // Right motor velocity is calculated using same principle
  dt = micros() - oldTime[1];
  velocity[1] = 1000000 * (distance[1] - oldDistance[1])/dt;
  oldDistance[1] = distance[1];
  oldTime[1] = micros();
  }
} encoders;

/*! This struct represents the kinematic modelling of the Zumo */
struct kinematics {
  float P_Zumo[3]= {0, 0, 0}; // x; y; theta θ
  int r = 20 / 2; // Zumo wheel radius in millimeters. To simplify forwardKinematics(float, float), it has been halved
  int width = 98 + 27; // Zumo width in millimeters (distance from wheel to wheel). An offset has been added to it

  /*! Arduino implementation of forward kinematic differential drive model. 
   *  To get precise calcluations, do not use delay() alongside this function.
   *  \param theta_l angular velocity of left motor
   *  \param theta_r angular velocity of right motor */
  void forwardKinematics(float theta_l, float theta_r) {
    P_Zumo[0] += (theta_l+theta_r)*r*cos(P_Zumo[2]*PI/180) * (encoders.oldTime[1] - encoders.oldTime[0])/1000000;  // Function cos() uses radians, thus degree to radian conversion needed ...
    P_Zumo[1] += (theta_l+theta_r)*r*sin(P_Zumo[2]*PI/180) * (encoders.oldTime[1] - encoders.oldTime[0])/1000000;  // ... formula: radian = degree * PI / 180
    P_Zumo[2] += (theta_l-theta_r)*r/(2*PI*width*2);
    if (P_Zumo[2] < 0) { P_Zumo[2] = P_Zumo[2] + 360; }   // Angles do not go in the negatives. If this would be the case, add 360 to degrees
    if (P_Zumo[2] > 360) { P_Zumo[2] = P_Zumo[2] - 360; } // Angles do not surpass 360 degrees. If this would be the case, subtract 360 to degrees
  }
} kinematics;

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  encoders.getVelocity(); // Extracts encoder "ticks" and converts it to distance (cm) and then velocity (cm/s)
  encoders.velocity[0] = encoders.velocity[0] / (kinematics.r/10); // Converts encoder velocity to angular velocity...
  encoders.velocity[1] = encoders.velocity[1] / (kinematics.r/10); // ... formula: v = r * ω <=> v/r = ω, where r = 1.2cm
  kinematics.forwardKinematics(encoders.velocity[0], encoders.velocity[1]); // Input: ω (radians/s)
  Serial.println((String)"X:\t" + kinematics.P_Zumo[0] + 
                     "\t\tY:\t" + kinematics.P_Zumo[1] + 
                     "\t\tθ:\t" + kinematics.P_Zumo[2]);
}