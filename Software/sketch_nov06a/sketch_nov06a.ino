/*
IDP
David Paterson
Line Sensor Module Example Code V1
Move the line sensor across the black and white line, monitor on serial 
*/
#include <Adafruit_MotorShield.h>

const int redButtonPin = 2;
const int frontLeftLSP = 4; // LSP = line sensor pin
const int frontRightLSP = 3;
const int axleLeftLSP = 6;
const int axleRightLSP = 5;
const int ultrasonicPin = A0;
const int tofPin = 999;

const int movetime = 1450;
const int rotatetime = 1300;

bool active = false;
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor* motor1 = AFMS.getMotor(1);
Adafruit_DCMotor* motor2 = AFMS.getMotor(3);

enum Movement {
  STOPPED,
  R_LEFT,
  R_RIGHT,
  M_FORWARD,
  M_BACKWARD
};
static inline void setMovement(Movement m) {
    switch(m) {
      case STOPPED:
        motor1->run(RELEASE);
        motor2->run(RELEASE);
        break;
      case R_LEFT:
        motor1->run(FORWARD);
        motor2->run(FORWARD);
        break;
      case R_RIGHT:
        motor1->run(BACKWARD);
        motor2->run(BACKWARD);
        break;
      case M_FORWARD:
        motor1->run(BACKWARD);
        motor2->run(FORWARD);
        break;
      case M_BACKWARD:
        motor1->run(FORWARD);
        motor2->run(BACKWARD);
        break;
    }
}

void setup() {
    Serial.begin(9600); // Init the serial port
    if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
        Serial.println("Could not find Motor Shield. Check wiring.");
        while (1);
    }
    Serial.println("Starting!!!!1!");
    motor1->setSpeed(150);
    motor1->run(RELEASE);
    motor2->setSpeed(150);
    motor2->run(RELEASE);
      
    pinMode(frontLeftLSP, INPUT); 
    pinMode(frontRightLSP, INPUT);
    pinMode(axleLeftLSP, INPUT); 
    pinMode(axleRightLSP, INPUT);
    pinMode(redButtonPin, INPUT);
}


int prevb = LOW;
void loop() {

    int button = digitalRead(redButtonPin);
    if(button != prevb) {
      prevb = button;
      if(button == HIGH)
        active = !active;
    }

    if(!active) {
      delay(10);
      return;
    }
    
   int frontLeft = digitalRead(frontLeftLSP);
   int frontRight = digitalRead(frontRightLSP);
   int axleLeft = digitalRead(axleLeftLSP);
   int axleRight = digitalRead(axleRightLSP);
  
   if (frontLeft == 0 && frontRight == 1)
      setMovement(R_RIGHT);
   else if (frontLeft == 1 && frontRight == 0)
      setMovement(R_LEFT);
   else if (frontLeft == 1 && frontRight == 1)
      setMovement(M_FORWARD);
   else if(frontLeft == 0 && frontRight == 0) {
      setMovement(M_FORWARD);
      delay(movetime);
      setMovement(R_RIGHT);
      delay(rotatetime);
   }
  delay(10);
}


/*
#include <Adafruit_MotorShield.h>

const int ledPin = 2;
const int redButtonPin = 1;

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
    pinMode(redButtonPin, INPUT);
    Serial.println("Starting!!!!1!");
    motor1->setSpeed(150);
    motor1->run(FORWARD);
    motor1->run(RELEASE);
    motor2->setSpeed(150);
    motor2->run(FORWARD);
    motor2->run(RELEASE);
    
}

void loop() {
    int button = digitalRead(redButtonPin);
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

// READING FROM THE Ultrasonic SENSOR
#define MAX_RANG (520)
#define ADC_SOLUTION (1023.0)
int sensityPin = A0;
float sensity_t = analogRead(sensityPin);
float dist_t = sensity_t * MAX_RANG / ADC_SOLUTION;

*/
