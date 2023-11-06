#include <Zumo32U4.h> // Includes downloaded library located in sketchbook location
Zumo32U4Motors motors; // Class object

int8_t speed=100; // Configured speed of each Zumo motor
int8_t interval = 3000; // Milliseconds

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  motors.setSpeeds(speed, speed);
  delay(interval);
  motors.setSpeeds(0, 0);
  delay(interval);
}