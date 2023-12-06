#include <Zumo32U4.h> // Includes downloaded library located in sketchbook location
Zumo32U4Encoders motorEncoders; // Class object

float distance[2];  // Motor distance in centimeters cm. distance[0] = left motor; distance[1] = right motor

void getDistance() { // Updates distance[2] and uses encoder "clicks" to calculate distance travelled by each motor in unit cm
  distance[0] = 8 * acos(0.0) * motorEncoders.getCountsLeft() / 900;
  distance[1] = 8 * acos(0.0) * motorEncoders.getCountsRight() / 900;
}

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  getDistance();
  Serial.println((String)"Left:\t" + distance[0] + "\t\tRight:\t" + distance[1]);
}
