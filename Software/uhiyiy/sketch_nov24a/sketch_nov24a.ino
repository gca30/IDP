#include "Arduino.h"
#include "Wire.h"
#include "DFRobot_VL53L0X.h"
#include "Adafruit_MotorShield.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor* motor1 = AFMS.getMotor(4);
Adafruit_DCMotor* motor2 = AFMS.getMotor(1);
DFRobot_VL53L0X sensor;


enum Movement {
    STOPPED,
    R_LEFT,
    R_RIGHT,
    M_FORWARD,
    M_BACKWARD
};

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
  motor1->setSpeed(150);
  motor1->run(RELEASE);
  motor2->setSpeed(150);
  motor2->run(RELEASE);
}


static inline void setMovement(Movement m) {
    // f = f
    // l = r
    switch(m) {
        case STOPPED:
            motor1->run(RELEASE);
            motor2->run(RELEASE);
            break;
        case R_LEFT:

            motor1->setSpeed(150);
            motor2->setSpeed(150);
            motor1->run(BACKWARD);
            motor2->run(FORWARD);

            break;
        case R_RIGHT:
            motor1->setSpeed(150);
            motor2->setSpeed(150);
            motor1->run(FORWARD);
            motor2->run(BACKWARD);

            break;
        case M_FORWARD:
            motor1->setSpeed(200);
            motor2->setSpeed(200);
            motor1->run(FORWARD);
            motor2->run(FORWARD);

            break;
        case M_BACKWARD:
            motor1->setSpeed(200);
            motor2->setSpeed(200);
            motor1->run(BACKWARD);
            motor2->run(BACKWARD);
            break;
    }
}

#define MEDIAN_SIZE 13
float time_of_flight() {
    static float dists[MEDIAN_SIZE];
    static float sorted[MEDIAN_SIZE];
    float flight =  sensor.getDistance();
    
    for(int i = 0; i < MEDIAN_SIZE-1; i++)
        dists[i] = dists[i+1];
    dists[MEDIAN_SIZE-1] = flight;
    for(int i = 0; i < MEDIAN_SIZE; i++)
        sorted[i] = dists[i];
    
    for(int i = 1; i < MEDIAN_SIZE; i++) {
        int j = i;
        while(j > 0 && sorted[j-1] > sorted[j]) {
            float temp = sorted[j];
            sorted[j] = sorted[j-1];
            sorted[j-1] = temp;
            j--;
        }
    }
    return sorted[MEDIAN_SIZE/2];
  }

float distance;
float past_distance;

void loop() 
{
  distance = time_of_flight();
  if (distance < past_distance) {
    setMovement(R_RIGHT);
    } else if (distance > past_distance) {
      setMovement(R_RIGHT);
      } else {
        setMovement(M_FORWARD);
      }
  past_distance = distance;
 //Get the distance
 Serial.print("Distance: ");Serial.println(sensor.getDistance());
 //The delay is added to demonstrate the effect, and if you do not add the delay,
 //it will not affect the measurement accuracy
 delay(500);
}
