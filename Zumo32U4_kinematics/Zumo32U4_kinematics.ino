#include <Zumo32U4.h>
Zumo32U4Encoders motorEncoders;
Zumo32U4Motors motors;

/*! This struct represents the encoders, aka. the motor turns, based on wheel circumference, measurer */
struct encoders {
  float distance[2];    // Motor distance in centimeters cm. distance[0] = left motor; distance[1] = right motor
  float velocity[2];    // Motor velocity in centimeters per second cm/s. velocity[0] = left motor; velocity[1] = right motor
  long pastTime[2];      // Stores past time in micros() after the respective motor velocity has been updated
  float pastDistance[2]; // Stores past distance after the respective motor velocity has been updated

  /*! Updates distance[2] and uses encoder "clicks" to calculate distance travelled by each motor in unit cm */
  void getDistance() { 
    distance[0] = 8 * acos(0.0) * motorEncoders.getCountsLeft() / 900;
    distance[1] = 8 * acos(0.0) * motorEncoders.getCountsRight() / 900;
  }

  /*! Updates velocity[2] and calculates velocity in cm/s. Also calculates distance[2] */
  void getVelocity() {
  getDistance(); // distance[2] is needed to get velocity = distance / time

  // Left motor velocity is calcluated
  long dt = micros() - pastTime[0]; // Get difference of time in microseconds micros() = new time; encoderTime = past time
  pastTime[0] = micros(); // Update past time
  velocity[0] = 1000000 * (distance[0] - pastDistance[0])/dt; // 1s = 1000000µs [a million microseconds]. Velocity (cm/µs) is converted to (cm/s).
  pastDistance[0] = distance[0]; // Update past distance

  // Right motor velocity is calculated using same principle
  dt = micros() - pastTime[1];
  pastTime[1] = micros();
  velocity[1] = 1000000 * (distance[1] - pastDistance[1])/dt;
  pastDistance[1] = distance[1];
  }
} encoders;

/*! This struct represents the kinematic modelling of the Zumo */
struct kinematics {
  float P_Zumo[3]= {0, 0, 0}; // Position of Zumo in 2D plane. x; y; theta θ
  int r = 2; // Zumo wheel radius in centimeters. To simplify forwardKinematics(float, float), it has been halved
  int s = 85; // Distance between the two continuous belts on the Zumo in millimeters. Belt width = 13mm; Zumo width = 98mm: 98mm - 13mm = 85mm
  long executedTime = micros(); // Time in microseconds since forwardKinematics(float, float) has been executed (or program start if never executed)

  /*! Arduino implementation of forward kinematic differential drive model. 
   *  This function updates the virtual position of the Zumo defined as float P_Zumo[3].
   *  To get precise calcluations, do not use delay() alongside this function.
   *  \param v_1 Linear velocity of left motor
   *  \param v_2 Linear velocity of right motor
   *  \param coordinateOffset Offset of P_Zumo[0], P_Zumo[1] and P_Zumo[2] in relation to real world measurements (calibration) */
  void forwardKinematics(float v_1, float v_2, float coordinateOffset) {
    encoders.getVelocity(); // Every time forwardKinematics(float, float, float) executes, the velocity found using each encoder must be found.
    long dt = micros() - executedTime;
    executedTime = micros(); // Update executedTime
    P_Zumo[0] += coordinateOffset*(v_1 + v_2)*cos(P_Zumo[2]*PI/180) * dt/(2*1000000);  // Function cos() uses radians, thus degree to radian conversion needed ...
    P_Zumo[1] += coordinateOffset*(v_1 + v_2)*sin(P_Zumo[2]*PI/180) * dt/(2*1000000);  // ... formula: radian = degree * PI / 180
    P_Zumo[2] += coordinateOffset*50*(v_1 - v_2) * dt / (s/10*1000000); // Equation: 0.5 * s^-1 * (v_1 - v_2) * Δt <=> 0.5 * (v_1 - v_2) * dt / (s*1000000)...  Multiply equation by 100 due to offset
    if (P_Zumo[2] < 0) { P_Zumo[2] = P_Zumo[2] + 360; }   // Angles do not go in the negatives. If this would be the case, add 360 to degrees
    if (P_Zumo[2] > 360) { P_Zumo[2] = P_Zumo[2] - 360; } // Angles do not surpass 360 degrees. If this would be the case, subtract 360 to degrees
  }

  /*! Simply drives Zumo in straight line until {length}cm has been traversed from start position by Zumo.
   *  \param v Linear velocity of Zumo whilst function is running 
   *  \param time Time function must run in milliseconds.
   *  \param offset Take into account any kinds of offset in calculations. */
  void driveStraight(float v, int time, float offset=1){
    motors.setSpeeds(offset*v*s/10, offset*v*s/10); // Note how {speed} directly translates into motor speeds by using the formula omega = v * w
    long startTime = millis();
    long nowTime = startTime; // Set current time to start time before beginning loop
    while(nowTime < startTime + time) {
      forwardKinematics(encoders.velocity[0], encoders.velocity[1], 0.95); // Updates robot position in relation to plane
      nowTime = millis(); // Update current time
      Serial.println((String)"X:\t" + P_Zumo[0] + 
                       "cm\t\tY:\t" + P_Zumo[1] + 
                       "cm\t\tθ:\t" + P_Zumo[2] + "°");
      }
    motors.setSpeeds(0, 0);
  }

  /*! Simply rotates Zumo until it has rotated by {degrees} from its start position.
   *  \param omega Angular velocity of Zumo whilst function is running 
   *  \param time Time function must run in milliseconds.
   *  \param offset Take into account any kinds of offset in calculations. */
  void driveRotation(float omega, int time, float offset = 1) {
    motors.setSpeeds(offset*omega, -offset*omega);
    long startTime = millis();
    long nowTime = startTime; // Set current time to start time before beginning loop
    while(nowTime < startTime + time) {
      forwardKinematics(encoders.velocity[0], encoders.velocity[1], 0.95); // Updates robot position in relation to plane
      nowTime = millis(); // Update current time
      Serial.println((String)"X:\t" + P_Zumo[0] + 
                       "cm\t\tY:\t" + P_Zumo[1] + 
                       "cm\t\tθ:\t" + P_Zumo[2] + "°");
      }
    motors.setSpeeds(0, 0);
  }

  /*! Arduino implementation of the Unicycle method. Consider it the equivalent to tank steering the Zumo.
   *  \param v Linear velocity of Zumo whilst function is running 
   *  \param omega Angular velocity of Zumo whilst function is running 
   *  \param time Time function must run in milliseconds.
   *  \param angularOffset Take into account any kinds of offset in calculating anglular velocity. */
  void unicycleModel(float v, float omega, int time, float angularOffset=1) {
    float v_1 = v*s/10 + angularOffset*omega;
    float v_2 = v*s/10 - angularOffset*omega;
    motors.setSpeeds(v_1, v_2);
    long startTime = millis();
    long nowTime = startTime; // Set current time to start time before beginning loop
    while (nowTime < startTime + time) {
      forwardKinematics(encoders.velocity[0], encoders.velocity[1], 0.95); // Updates robot position in relation to plane
      nowTime = millis(); // Update current time
      Serial.println((String)"X:\t" + P_Zumo[0] + 
                       "cm\t\tY:\t" + P_Zumo[1] + 
                       "cm\t\tθ:\t" + P_Zumo[2] + "°");
    }
    motors.setSpeeds(0, 0);
  }
} kinematics;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(1000);
  kinematics.driveStraight(20, 1000);
  kinematics.driveRotation(90, 1000);
  kinematics.unicycleModel(20, 90, 1000, 0.75);
}

void loop() {
  // put your main code here, to run repeatedly:
  kinematics.forwardKinematics(encoders.velocity[0], encoders.velocity[1], 0.95); // Updates robot position in relation to plane
  Serial.println((String)"X:\t" + kinematics.P_Zumo[0] + 
                       "cm\t\tY:\t" + kinematics.P_Zumo[1] + 
                       "cm\t\tθ:\t" + kinematics.P_Zumo[2] + "°");
}
