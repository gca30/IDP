
#include <Adafruit_MotorShield.h>

const int ledPin = 2;
const int inputPin = 1;
#define MAX_RANG (520)
#define ADC_SOLUTION (1023.0)

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor* motor1 = AFMS.getMotor(1);
Adafruit_DCMotor* motor2 = AFMS.getMotor(3);

int state=0, prevb=LOW;
void setup() {
    Serial.begin(9600);           // set up Serial library at 9600 bps
    if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
      Serial.println("Could not find Motor Shield. Check wiring.");
      while (1);
    }
    pinMode(inputPin, INPUT);
    Serial.println("Starting!!!!1!");
    motor1->setSpeed(150);
    motor1->run(FORWARD);
    motor1->run(RELEASE);
    motor2->setSpeed(150);
    motor2->run(FORWARD);
    motor2->run(RELEASE);
    
}

void loop() {
    int button = digitalRead(inputPin);
    if(prevb == HIGH && button == LOW) {
      state = (state+1)%5;
      Serial.print("Switched to state ");
      Serial.println(state);
      switch(state) {
        case 0:
          // RIGHT
          motor1->run(BACKWARD);
          motor2->run(BACKWARD);
          break;
        case 1:
          // LEFT
          motor1->run(FORWARD);
          motor2->run(FORWARD);
          break;
        case 2:
          // BACKWARD
          motor1->run(FORWARD);
          motor2->run(BACKWARD);
          break;
        case 3:
          // FORWARD
          motor1->run(BACKWARD);
          motor2->run(FORWARD);
          break;
        case 4:
          // STOP
          motor1->run(RELEASE);
          motor2->run(RELEASE);
          break;
      }
    }
    prevb = button;
    delay(1);
}

/*
 * SNIPPEETS
 */
/*

// READING FROM THE IR SENSOR
#define MAX_RANG (520)
#define ADC_SOLUTION (1023.0)
int sensityPin = A0;
float sensity_t = analogRead(sensityPin);
float dist_t = sensity_t * MAX_RANG / ADC_SOLUTION;

*/
