#include "Arduino.h"
#include "Wire.h"
#include "DFRobot_VL53L0X.h"
DFRobot_VL53L0X sensor;
void setup() {
 //initialize serial communication at 9600 bits per second:
 Serial.begin(115200);
 //join i2c bus (address optional for master)
 Wire.begin();
 //Set I2C sub-device address
 sensor.begin(0x50);
 //Set to Back-to-back mode and high precision mode
 sensor.setMode(sensor.eContinuous,sensor.eHigh);
 //Laser rangefinder begins to work
 sensor.start();
}
void loop()
{
 //Get the distance
 Serial.print("Distance: ");Serial.println(sensor.getDistance());
 //The delay is added to demonstrate the effect, and if you do not add the delay,
 //it will not affect the measurement accuracy
 delay(500);
}
