#include <Zumo32U4.h> // Includes downloaded library located in sketchbook location
Zumo32U4ProximitySensors proximitySensors; // Class object

struct proximity {
  uint8_t value[6];                   // The array where proximity sensor values are stored.
  static const uint8_t levels = 1;    // static is needed when const is in struct. Range: 0 - 255
  uint16_t brightnessLevels[levels];  // By default, each element in array is equal to 0
  long executeTime;                   // Stores time in microseconds when getProximity() begins

  void init() {
    for (int i = 0; i < levels%256; i++) { brightnessLevels[i] = 10; } // Where range of brightnessLevels[] is 1 - 420
    proximitySensors.initThreeSensors();
    proximitySensors.setBrightnessLevels(brightnessLevels, levels%256); // Modulus 256 ensures elements are not overwritten
  }

  void getProximity() {
    executeTime = micros();
    proximitySensors.read();
    value[0] = proximitySensors.countsLeftWithLeftLeds();
    value[1] = proximitySensors.countsLeftWithRightLeds();

    value[2] = proximitySensors.countsFrontWithLeftLeds();
    value[3] = proximitySensors.countsFrontWithRightLeds();

    value[4] = proximitySensors.countsRightWithLeftLeds();
    value[5] = proximitySensors.countsRightWithRightLeds();
    Serial.print((String)"VALUES:\t" + value[0] + "\t" + value[1] + "\t\t" + value[2] + "\t" + 
                                         value[3] + "\t\t" + value[4] + "\t" + value[5]);
    Serial.println((String)"\t\t\t\tExecution time in ms:\t" + (micros() - executeTime)/1000); // 
  }
} proximity;

void setup() {
  // put your setup code here, to run once:
  proximity.init();
}

void loop() {
  // put your main code here, to run repeatedly:
  proximity.getProximity();
}