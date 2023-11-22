#include <Adafruit_MotorShield.h>

// ----------------
// VARIABLES AND CONSTANTS
// ----------------

// SENSOR PINS
const int redButtonPin = 2;
const int frontLeftLSP = 4; // LSP = line sensor pin
const int frontRightLSP = 6;
const int axleLeftLSP = 5;
const int axleRightLSP = 3;
const int ultrasonicPin = A0;
const int redLEDPin = 999;
const int greenLEDPin = 999;
const int blueLEDPin = 999;
const int tofPin = 999;
const int magneticSensorPin = 7;
const int errorLEDPin = 999;

// SENSOR VALUES
int frontLeft, frontRight, axleLeft, axleRight;
float ultrasonic = 500.0, prevUltrasonic;
int magnetic = LOW;
int button = LOW, prevButton = LOW;

// HARD CODED DISTANCES
// const int movetime = 1450; ~ length of the car
// const int rotatetime = 1300; ~ 90 degrees

// STATE
char points[] = "x\
ooooo\
ooooo\
xooox\
xoxox";

enum State {
    INACTIVE, // completely stopped
    INITIAL, // the initial state of the car, in which it needs to move forward until it hits the line
    LINE_FOLLOWING, // we line follow until one of the back sensor is white
    CHANGE_DIRECTION, // we change our direction at a junction until the desiredDir is hit
    DEPOSIT, // depositing of the cube from the starting position
    PICKUP_DELAY, // delay when the cube is picked up
    VIBE_CHECK
};

enum Direction {
    NORTH = -5,
    SOUTH = 5,
    WEST = -1,
    EAST = 1
};

enum CubeState {
    NO_CUBE,
    CUBE_NEXT_JUNCTION,
    NOT_MAGNETIC,
    MAGNETIC
};

enum TaskState: int {
    FIRST_CUBE = 0,
    SECOND_CUBE = 1,
    SCANNING_FIRST = 2,
    SCANNING_SECOND = 3
};

enum Movement {
    STOPPED,
    R_LEFT,
    R_RIGHT,
    M_FORWARD,
    M_BACKWARD
};

void setState(State newState);
State state = INACTIVE;
TaskState taskState = FIRST_CUBE;
CubeState cubeState = NO_CUBE;


int positon = 18;
Direction directn = NORTH;
bool backwards = false;

Direction desiredDir = NORTH;
State prevState = INITIAL;
int delayCounter = 1000;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor* motor1 = AFMS.getMotor(4);
Adafruit_DCMotor* motor2 = AFMS.getMotor(1);



// ----------------
// HELPER FUNCTIONS
// ----------------

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
float getUltrasonicDistance() {
    static float dists[MEDIAN_SIZE];
    static float sorted[MEDIAN_SIZE];
    float sensity_t = analogRead(ultrasonicPin);
    float dist_t = sensity_t * 520 / 1023.0;
    
    for(int i = 0; i < MEDIAN_SIZE-1; i++)
        dists[i] = dists[i+1];
    dists[MEDIAN_SIZE-1] = dist_t;
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
#undef MEDIAN_SIZE

void setCubeState(CubeState cb) {
    if(cb == cubeState)
        return;
    cubeState = cb;
    switch (cubeState) {
        case NO_CUBE:
        case CUBE_NEXT_JUNCTION:
            digitalWrite(redLEDPin, LOW);
            digitalWrite(greenLEDPin, LOW);
            break;
        case NOT_MAGNETIC:
            digitalWrite(redLEDPin, LOW);
            digitalWrite(greenLEDPin, HIGH);
            break;
        case MAGNETIC:
            digitalWrite(redLEDPin, HIGH);
            digitalWrite(greenLEDPin, LOW);
            break;
    }
}



// ----------------
// ALGORITHM FOR THE ROBOT
// ----------------

Direction getDesiredDirection() {
    /*if(cubeState == NO_CUBE) {
        switch(positon) {
        case 1:
            return EAST;
            break;
        case 2:
            if(directn == WEST)
                return WEST;
            else if(directn == EAST)
                return SOUTH;
            break; 
        case 3:
            if(directn == NORTH) {
                if(points[10] == 'e') // visited lower right side
                    return EAST;
                return WEST;
            }
            else if(directn == WEST)
            break; 
        case 4:
            if(directn == NORTH)
                return EAST;
            else if(directn == WEST)
                return WEST;
            break; 
        case 5:
            return WEST;
            break; 
        case 6:
            return EAST;
            break; 
        case 7:
            if(directn == EAST)
                return EAST;
            else if(directn == SOUTH)
                return WEST;
            break; 
        case 8:
            if(directn == EAST)
                return EAST;
            else if(directn == NORTH) {
                if(points[6] == 'e') // checked whole right side
                    return EAST;
                if(points[1] = 'e' && points[7] != 'e') // checked only upper right side
                    return WEST;
                return NORTH;
            }
            else if(directn == SOUTH)
                return SOUTH;
            break; 
        case 9:
            if(directn == EAST)
                return EAST;
            else if(directn == WEST)
                return NORTH;
            break; 
        case 10:
            return WEST;
            break; 
        default:
            return directon;
        }
    }*/ 
    if(cubeState == NO_CUBE) {
        switch(positon) {
        case 1:
            return EAST;
        case 2:
            if(directn == NORTH)
                return WEST;
            else if(directn == EAST)
                return EAST;
            break;
        case 3:
            return EAST;
        case 4:
            if(directn == EAST)
                return EAST;
            else if(directn == WEST)
                return SOUTH;
            break;
        case 5:
            return WEST;
        case 6:
            return EAST;
        case 7:
            if(directn == WEST)
                return WEST;
            else if(directn == EAST)
                return NORTH;
            break;
        case 8:
            if(directn == NORTH)
                return WEST;
            else if(directn == EAST)
                return EAST;
            else if(directn == WEST)
                return NORTH;
            else if(directn == SOUTH)
                return SOUTH;
            break;
        case 9:
            if(directn == SOUTH)
                return EAST;
            else if(directn == WEST)
                return WEST;
            break;
        case 10:
            return WEST;
        case 13:
            return NORTH;
        default:
            return directn;
        }
    }
    else {
        switch(positon % 5) {
          case 1: case 2:
              return EAST;
          case 3:
              return SOUTH;
          case 4: case 0:
              return WEST;
        }
        return SOUTH;
    }

    // unreachable 
    // digitalWrite(errorLEDPin, HIGH);
    setState(INACTIVE);
    return NORTH;
}

CubeState csAtLastJunction = NO_CUBE;

void updateOnJunction() {
    positon += directn;
    desiredDir = getDesiredDirection();
    Serial.print("pos = ");
    Serial.print(positon);
    Serial.print(", dir = ");
    Serial.print(directn);
    Serial.print(", desired dir = ");
    Serial.print(desiredDir);
    if(cubeState == CUBE_NEXT_JUNCTION)
      setCubeState(magnetic ? MAGNETIC : NOT_MAGNETIC);
    if(cubeState != NO_CUBE)
        Serial.print(", with cube");
    Serial.println("");
    if(csAtLastJunction == NO_CUBE)
        points[positon] = 'e'; // now the position we are in is empty
    csAtLastJunction = cubeState;
}



// ----------------
// SPECIFIC STATE FUNCTIONS
// ----------------

// LINE FOLLOWING STATE

bool shouldChangeState = false;
bool outOfJunction;

void lineFollowingSetup() {
    outOfJunction = false;
}

void lineFollowingLoop() {
    
    if (frontLeft == 0 && frontRight == 1)
        setMovement(R_RIGHT);
    else if (frontLeft == 1 && frontRight == 0)
        setMovement(R_LEFT);
    else if (frontLeft == 1 && frontRight == 1)
        setMovement(M_FORWARD);
    else if(frontLeft == 0 && frontRight == 0)
        setMovement(M_FORWARD);

    // we can start in a junction and we must first ignore it
    if(axleRight == 0 && axleLeft == 0)
        outOfJunction = true;
    if(outOfJunction && (axleRight == 1 || axleLeft == 1)) {
        updateOnJunction();
        if(0 < positon + directn && positon + directn < 10 && points[positon + directn] != 'e' && cubeState == NO_CUBE) {
            setState(VIBE_CHECK);
            return;
        }
        if(positon == 13 && directn == SOUTH) {
            setState(DEPOSIT);
            return;
        }
        setState(CHANGE_DIRECTION);
    }
}

// CHANGE DIRECTION STATE

int rotationIterations;

void changeDirectionSetup() {
    if(directn == desiredDir) {
        setState(LINE_FOLLOWING);
        return;
    }
    outOfJunction = false;
    rotationIterations = directn == -desiredDir ? 2 : 1;
    if(
        (directn == NORTH && desiredDir == WEST) ||
        (directn == WEST && desiredDir == SOUTH) ||
        (directn == SOUTH && desiredDir == EAST) ||
        (directn == EAST && desiredDir == NORTH)
    )
        setMovement(R_LEFT);
    else if(directn == -desiredDir && (positon-1)%5 > 2) // special case when turing at point 6
        setMovement(R_LEFT);
    else
        setMovement(R_RIGHT);
}

void changeDirectionLoop() {

    // it can only exit the loop if it has cleared the line
    if(frontLeft == LOW && frontRight == LOW)
      outOfJunction = true;

    if(frontLeft != HIGH || frontRight != HIGH)
        return; //loop back
    if(!outOfJunction)
        return;
    if(rotationIterations > 1) {
        rotationIterations--;
        outOfJunction = false;  
        return;
    }
    directn = desiredDir;
    if(0 < positon + directn && positon + directn < 10 && points[positon + directn] != 'e' && cubeState == NO_CUBE)
        setState(VIBE_CHECK);
    else
        setState(LINE_FOLLOWING);
}

// VIBE CHECK DELAY

int vibeCheckDelay;

void vibeCheckSetup() {
    setMovement(STOPPED);
    vibeCheckDelay = 200;
}

void vibeCheckLoop() {
    Serial.println(ultrasonic);
    if(vibeCheckDelay > 0) {
        vibeCheckDelay--;
        return;
    }
    if(ultrasonic > 30.0 && ultrasonic < 65.0) {
        setCubeState(CUBE_NEXT_JUNCTION);
        Serial.println("CUBE DETECTED HERE");
    }
    else
        points[positon + directn] = 'e';
    setState(CHANGE_DIRECTION);
}

// INITIAL STATE

void initialSetup() {
  positon = 18;
}

void initialLoop() {
    // initial wait time to get ultrasonic sensor median readings
    static int counter = 13;
    if(counter > 0) {
        counter--;
        if(counter == 0)
            setMovement(M_FORWARD);
        return;
    }
    // move forward until we hit the line
    if(frontLeft == HIGH && frontRight == HIGH)
        setState(LINE_FOLLOWING);
}


// DELAY STATE

void pickupDelaySetup() {
    setMovement(STOPPED);
    delayCounter = 1000;
}

void pickupDelayLoop() {
    if(delayCounter == 0) {
        setState(prevState);
        return;
    }
    delayCounter--;
}

// INACTIVE STATE

void inactiveSetup() {
    setMovement(STOPPED);
}

void inactiveLoop() {
  ;;
}

// DEPOSIT STATE


const int depositRotate = 310;
const int depositForward = 1200;
const int depositSmallForward = 140;
int depositProgress = 0;
int depositTimer = 0;
CubeState initialCubeState = NO_CUBE;

void depositSetup() {
    depositProgress = 0;  
    initialCubeState = cubeState;
}

void depositLoop() {
    if(depositTimer > 0) {
        depositTimer--;
        return;
    }
    switch(depositProgress) {
        case 0:
            setMovement(M_FORWARD);
            depositTimer = depositSmallForward;
            break;
        case 1:
            setMovement(initialCubeState == MAGNETIC ? R_RIGHT : R_LEFT);
            depositTimer = depositRotate;
            break;
        case 2:
            setMovement(M_FORWARD);
            depositTimer = depositForward;
            break;
        case 3:
            setMovement(M_BACKWARD);
            setCubeState(NO_CUBE);
            depositTimer = depositForward;
            break;
        case 4:
            setMovement(initialCubeState == MAGNETIC ? R_RIGHT : R_LEFT);
            depositTimer = depositRotate;
            break;
        case 5:
            taskState = taskState + 1;
            setState(INITIAL);
            return;
    }
    depositProgress++;
}

// VIBE CHECK STATE



// ----------------
// STATE MANAGEMENT
// ----------------

// STATE SETUP
void setState(State newState) {
    if(newState == state)
        return;
    prevState = state;
    state = newState;
    // do setup for the new state
    switch(state) {
        case LINE_FOLLOWING:
            lineFollowingSetup();
            break;
        case CHANGE_DIRECTION:
            changeDirectionSetup();
            break;
        case DEPOSIT:
            depositSetup();
            break;
        case INACTIVE:
            inactiveSetup();
            break;
        case PICKUP_DELAY:
            pickupDelaySetup();
            break;
        case INITIAL:
            initialSetup();
            break;
        case VIBE_CHECK:
            vibeCheckSetup();
            break;
        default:
            break;
    }
}

void stateSensors() {
    // Button sets inactive state
    if(prevButton == HIGH && button == LOW)
        setState(state == INACTIVE ? prevState : INACTIVE);

    // Ultrasonic sensor changes LED state
    /*
    if(cubeState == NO_CUBE)
        Serial.println(ultrasonic);
    if(
        (frontRight == HIGH || frontLeft == HIGH) && 
        state != INACTIVE && state != INITIAL && state != DEPOSIT && cubeState == NO_CUBE &&
        (ultrasonic < 7.0 || ultrasonic > 480.0) && ultrasonic != 0.0
    ) {
        if(magnetic == HIGH) {
            Serial.println("MAGNETIC!!");
            setCubeState(MAGNETIC);
        }
        else {
            Serial.println("NOT MAGNETIC!");
            setCubeState(NOT_MAGNETIC);
        }
        setState(PICKUP_DELAY);
    } */
}

void statesLoop() {
    // ACTION BASED ON STATE
    switch(state) {
        case INACTIVE:
            inactiveLoop();
            break;
        case DEPOSIT:
            depositLoop();
            break;
        case INITIAL:
            initialLoop();
            break;
        case PICKUP_DELAY:
            pickupDelayLoop();
            break;
        case LINE_FOLLOWING:
            lineFollowingLoop();
            break;
        case CHANGE_DIRECTION:
            changeDirectionLoop();
            break;
        case VIBE_CHECK:
            vibeCheckLoop();
            break;
    }
}


// ----------------
// ARDUINO FUNCTIONS
// ----------------

void setup() {
    Serial.begin(9600);
    if (!AFMS.begin()) {
        Serial.println("Could not find Motor Shield. Check wiring.");
        while (1);
    }
    Serial.println("Begin");
    motor1->setSpeed(150);
    motor1->run(RELEASE);
    motor2->setSpeed(150);
    motor2->run(RELEASE);
      
    pinMode(frontLeftLSP, INPUT); 
    pinMode(frontRightLSP, INPUT);
    pinMode(axleLeftLSP, INPUT); 
    pinMode(axleRightLSP, INPUT);
    pinMode(redButtonPin, INPUT);
    pinMode(magneticSensorPin, INPUT);
    // pinMode(ultrasonicPin, INPUT);
    // pinMode(redLEDPin, OUTPUT);
    // pinMode(greenLEDPin, OUTPUT);
    // pinMode(blueLEDPin, OUTPUT);
    // pinMode(errorLEDPin, INPUT);
    // pinMode(tofPin, INPUT);
}

void loop() {

    // SENSOR READING
    prevButton = button;
    button = digitalRead(redButtonPin);
    frontLeft = digitalRead(frontLeftLSP);
    frontRight = digitalRead(frontRightLSP);
    axleLeft = digitalRead(axleLeftLSP);
    axleRight = digitalRead(axleRightLSP);
    ultrasonic = getUltrasonicDistance();
    magnetic = digitalRead(magneticSensorPin);

    stateSensors();
    statesLoop();      
    delay(5);
}
