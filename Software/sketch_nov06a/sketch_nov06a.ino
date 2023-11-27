#include <Adafruit_MotorShield.h>

// ----------------
// VARIABLES AND CONSTANTS
// ----------------

// SENSOR PINS
const int redButtonPin = 0;
const int frontLeftLSP = 2; // LSP = line sensor pin
const int frontRightLSP = 1;
const int axleLeftLSP = 4;
const int axleRightLSP = 3;
const int ultrasonicPin = A0;
const int redLEDPin = 7;
const int greenLEDPin = 8;
const int blueLEDPin = 6;
const int magneticSensorPin = 5;

// SENSOR VALUES
int frontLeft, frontRight, axleLeft, axleRight;
float ultrasonic = 500.0, prevUltrasonic;
int magnetic = LOW;
int button = LOW, prevButton = LOW;

// HARD CODED DISTANCES
// const int movetime = 1450; ~ length of the car
// const int rotatetime = 1250; ~ 90 degrees

// STATE
char points[] = "x\
ooooo\
ooooo\
xooox\
xoxox";
const int COMMAND_COUNT = 21;
char commands[] = "mcWcmcNcmWcEmcmcScmEc";
int commandsFollowed = 0;
int commandsIndex = 0;

enum State {
    INACTIVE, // completely stopped
    INITIAL, // the initial state of the car, in which it needs to move forward until it hits the line
    LINE_FOLLOWING, // we line follow until one of the back sensor is white
    CHANGE_DIRECTION, // we change our direction at a junction until the desiredDir is hit
    DEPOSIT, // depositing of the cube from the starting position
    PICKUP_DELAY, // delay when the cube is picked up
    VIBE_CHECK,
    DEPOSIT_2
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
    LINE_FIRST_CUBE = 0,
    LINE_SECOND_CUBE = 1,
    SCANNING_FIRST_CUBE = 2,
    SCANNING_SECOND_CUBE = 3,
    DONE = 4
};

enum Movement {
    STOPPED,
    R_LEFT,
    R_RIGHT,
    M_FORWARD,
    M_BACKWARD,
    PIVOTB_RIGHT,
    PIVOTB_LEFT,
    PIVOTF_RIGHT,
    PIVOTF_LEFT
};

void setState(State newState);
State state = INACTIVE;
TaskState taskState = LINE_FIRST_CUBE;
CubeState cubeState = NO_CUBE;


int positon = 18;
Direction directn = NORTH;
bool backwards = false;

Direction desiredDir = NORTH;
State prevState = INITIAL;
int delayCounter = 400;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor* motor1 = AFMS.getMotor(4);
Adafruit_DCMotor* motor2 = AFMS.getMotor(1);
          
void switchState();


// ----------------
// HELPER FUNCTIONS
// ----------------

static inline void setMovement(Movement m) {
    // MOTOR1 = LEFT
    switch(m) {
        case STOPPED:
            motor1->run(RELEASE);
            motor2->run(RELEASE);
            break;
        case R_LEFT:
            motor1->setSpeed(200);
            motor2->setSpeed(200);
            motor1->run(BACKWARD);
            motor2->run(FORWARD);
            break;
        case R_RIGHT:
            motor1->setSpeed(200);
            motor2->setSpeed(200);
            motor1->run(FORWARD);
            motor2->run(BACKWARD);
            break;
        case M_FORWARD:
            motor1->setSpeed(250);
            motor2->setSpeed(250);
            motor1->run(FORWARD);
            motor2->run(FORWARD);
            break;
        case M_BACKWARD:
            motor1->setSpeed(250);
            motor2->setSpeed(250);
            motor1->run(BACKWARD);
            motor2->run(BACKWARD);
            break;
        case PIVOTB_RIGHT:
            motor1->setSpeed(200);
            motor2->setSpeed(200);
            motor1->run(RELEASE);
            motor2->run(BACKWARD);
            break;
        case PIVOTB_LEFT:
            motor1->setSpeed(200);
            motor2->setSpeed(200);
            motor1->run(BACKWARD);
            motor2->run(RELEASE);
            break;
        case PIVOTF_RIGHT:
            motor1->setSpeed(200);
            motor2->setSpeed(200);
            motor1->run(RELEASE);
            motor2->run(FORWARD);
            break;
        case PIVOTF_LEFT:
            motor1->setSpeed(200);
            motor2->setSpeed(200);
            motor1->run(FORWARD);
            motor2->run(RELEASE);
            break;
    }
}

// m1v + m2v = 400u
 


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


Direction backDirection(int point) {
    switch(point % 5) {
        case 1: case 2:
            return EAST;
        case 4: case 0:
            return WEST;
        case 3:
            return SOUTH;
    }
}

bool validNext(int point, Direction facing) {
    if(!(0 < point + facing && point + facing < 11))
        return false;
    if((facing == EAST || facing == WEST) && (point-1)/5 != (point+facing-1)/5)
        return false;
    return true;
}

void admitDefeat() {
    setMovement(R_RIGHT); 
    while(1);
}

Direction rightOf(Direction d) {
    switch(d) {
        case NORTH: return EAST;
        case EAST: return SOUTH;
        case SOUTH: return WEST;
        case WEST: return NORTH;
    }
}

bool hasValidRotJunction(int point, Direction d) {
    if(d == NORTH)
        return !(point == 2 || point == 3 || point == 4);
    if(d == SOUTH)
        return !(point == 6 || point == 7 || point == 9 || point == 10);
    if(d == EAST)
        return !(point == 5 || point == 10);
    if(d == WEST)
        return !(point == 1 || point == 6);
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
        //updateOnJunction();
        positon += directn;
        switchState();
    }
}

// CHANGE DIRECTION STATE

int rotationIterations;

void changeDirectionSetup() {
    if(directn == desiredDir) {
        switchState();
        return;
    }
    outOfJunction = false;
    rotationIterations = directn == -desiredDir ? 2 : 1;
    if(desiredDir == -rightOf(directn))
        setMovement(R_LEFT);
    else if(directn == -desiredDir) {
        if(hasValidRotJunction(positon, rightOf(directn)))
            setMovement(R_RIGHT);
        else
            setMovement(R_LEFT);
    }
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
    switchState();
}

// VIBE CHECK DELAY

int vibeCheckDelay;

void vibeCheckSetup() {
    setMovement(STOPPED);
    vibeCheckDelay = 50;
}

void vibeCheckLoop() {
    Serial.println(ultrasonic);
    if(vibeCheckDelay > 0) {
        vibeCheckDelay--;
        return;
    }
    if(ultrasonic > 25.0 && ultrasonic < 40.0) {
        setCubeState(CUBE_NEXT_JUNCTION);
        Serial.println("CUBE DETECTED HERE");
    }
    else
        points[positon + directn] = 'e';
    switchState();
}

// INITIAL STATE

int initialStateCounter = 100;

void initialSetup() {
    positon = 18;
    directn = NORTH;
    commandsIndex = 0;
    initialStateCounter = 13;
    setMovement(STOPPED);
}

void initialLoop() {
    // initial wait time to get ultrasonic sensor median readings
    if(initialStateCounter > 0) {
        initialStateCounter--;
        if(initialStateCounter == 0)
            setMovement(M_FORWARD);
        return;
    }
    // move forward until we hit the line
    if(frontLeft == HIGH || frontRight == HIGH)
        setState(LINE_FOLLOWING);
}


// DELAY STATE

void pickupDelaySetup() {
    setMovement(STOPPED);
    setCubeState(magnetic ? MAGNETIC : NOT_MAGNETIC);
    delayCounter = 400;
}

void pickupDelayLoop() {
    if(delayCounter > 0) {
        delayCounter--;
        return;
    }
    switchState();
}

// INACTIVE STATE

void inactiveSetup() {
    setMovement(STOPPED);
}

void inactiveLoop() {
    ;;
}

// DEPOSIT STATE

const int depositRotateM = 240; //fine tuning. Initially:353
const int depositRotateNM = 265;
const int depositForward = 850;
const int depositBackward = 450;
const int depositSmallForward = 160;
const int depositSmallBackward = 150;
const int depositFinalBack = 200;
const int depositLineFollowingHell = 200;
int depositProgress = 0;
int depositTimer = 0;
CubeState initialCubeState = NO_CUBE;
// NO
// move forward by a bit
// rotate ~90 degress
// move forward a hardcoded amount
// move back a hardcoded amount
// move back until one of the back sensors is white
// pivot until both are white
// move backward by a bit
// rotate ~90 degrees
// move forward until one of the sensors is white
// pivot until both are white
// go back by a a bit

void depositSetup() {
    depositProgress = 0;
    initialCubeState = cubeState;
}

void depositLoop() {
    if(depositTimer > 0) {
        depositTimer--;
        return;
    }
    switch(depositProgress+1) {
        case 1:
            setMovement(M_FORWARD);
            depositTimer = depositSmallForward;
            break;
        case 2:
            setMovement(initialCubeState == MAGNETIC ? R_RIGHT : R_LEFT);
            depositTimer = initialCubeState == MAGNETIC ? depositRotateM : depositRotateNM;
            break;
        case 3:
            setMovement(M_FORWARD);
            depositTimer = depositForward;
            break;
        case 4:
            setMovement(M_BACKWARD);
            setCubeState(NO_CUBE);
            depositTimer = depositBackward;
            break;
        case 5:
            setMovement(M_BACKWARD);
            depositTimer = 0;
            break;
        case 6:
            if(axleRight == HIGH && axleLeft == HIGH) {
                depositProgress++;
                setMovement(initialCubeState == MAGNETIC ? R_RIGHT : R_LEFT);
                depositTimer = initialCubeState == MAGNETIC ? depositRotateM : depositRotateM;
                return;
            }
            else if(axleRight == HIGH || axleLeft == HIGH) {
                if(axleRight == HIGH)
                    setMovement(PIVOTB_LEFT);
                else
                    setMovement(PIVOTB_RIGHT);
            }
            return;
        case 7:
            directn = NORTH;
            positon = initialCubeState == MAGNETIC ? 17 : 19;
            setState(LINE_FOLLOWING);
            break;
    }
    depositProgress++;
}

// DEPOSIT_2

void deposit2Setup() {
    depositProgress = 0;
    depositTimer = 0;
    setMovement(STOPPED);
}


void deposit2Loop() {
    if(depositTimer > 0) {
        depositTimer--;
        return;
    }
    depositProgress++;
    switch(depositProgress) {
        case 1:
            setMovement(M_BACKWARD);
            depositTimer = depositFinalBack;
            break;
        case 2:
            setMovement(STOPPED);
            digitalWrite(blueLEDPin, HIGH);
            depositTimer = 400;
            break;
        case 3:
            taskState = taskState + 1;
            if(taskState == SCANNING_FIRST_CUBE) {
                setState(INACTIVE);
                digitalWrite(blueLEDPin, HIGH);
            }
            else
                setState(INITIAL);
            break;
    }
}

// ----------------
// STATE MANAGEMENT
// ----------------

// STATE SWITCH
// for the 4 states that will be found in the board
void switchState() {
    if(cubeState == NO_CUBE) {
        if(state == LINE_FOLLOWING && positon == 12) {
            desiredDir = EAST;
            setState(CHANGE_DIRECTION);
            return;
        }
        if(state == LINE_FOLLOWING && positon == 14) {
            desiredDir = WEST;
            setState(CHANGE_DIRECTION);
            return;
        }
        if(state == CHANGE_DIRECTION && (positon == 12 || positon == 14)) {
            setState(LINE_FOLLOWING);
            return;
        }
        if(state == LINE_FOLLOWING && positon == 13 && (directn == EAST || directn == WEST)) {
            desiredDir = NORTH;
            setState(CHANGE_DIRECTION);
            return;
        }
        if(state == CHANGE_DIRECTION && positon == 13 && directn == NORTH) {
            setState(DEPOSIT_2);
            return;
        }
        if(commands[commandsIndex] == 'c' && points[positon + directn] == 'e')
            commandsIndex++;
        else 
            commandsFollowed = commandsIndex;
        if(commandsIndex == COMMAND_COUNT)
            admitDefeat();
        commandsIndex++;
        switch(commands[commandsIndex-1]) {
            case 'm':
                setState(LINE_FOLLOWING);
                return;
            case 'c':
                setState(VIBE_CHECK);
                return;
            case 'W':
                desiredDir = WEST;
                setState(CHANGE_DIRECTION);
                return;
            case 'E':
                desiredDir = EAST;
                setState(CHANGE_DIRECTION);
                return;
            case 'N':
                desiredDir = NORTH;
                setState(CHANGE_DIRECTION);
                return;
            case 'S':
                desiredDir = SOUTH;
                setState(CHANGE_DIRECTION);
                return;
        }
    }
    else if(cubeState == CUBE_NEXT_JUNCTION) {
        if(state == VIBE_CHECK)
            setState(LINE_FOLLOWING);
        else if(state == LINE_FOLLOWING)
            setState(PICKUP_DELAY);
        else admitDefeat(); /* impossible */ 
    }
    else {
        if(state == LINE_FOLLOWING && positon == 13)
            setState(DEPOSIT);
        else if(state == PICKUP_DELAY || state == LINE_FOLLOWING) {
            desiredDir = backDirection(positon);
            if(desiredDir == directn)
                setState(LINE_FOLLOWING);
            else
                setState(CHANGE_DIRECTION);
        }
        else setState(LINE_FOLLOWING);
    }

    /*
    if(state == LINE_FOLLOWING) {
        if(cubeState == CUBE_NEXT_JUNCTION) {
            setState(PICKUP_DELAY);
            return;
        }
        if(positon == 13 && directn == SOUTH) {
            setState(DEPOSIT);
            return;
        }
        if(validNext(positon, directn) && points[positon + directn] != 'e' && cubeState == NO_CUBE) {
            setState(VIBE_CHECK);
            return;
        }
        setState(CHANGE_DIRECTION);
    }
    else if(state == CHANGE_DIRECTION) {
        if(validNext(positon, directn) && points[positon + directn] != 'e' &&  && cubeState == NO_CUBE)
            setState(VIBE_CHECK);
        else
            setState(LINE_FOLLOWING);
    }
    else if(state == VIBE_CHECK) {
        if(cubeState == CUBE_NEXT_JUNCTION);
            setState(LINE_FOLLOWING);
        else
            setState(CHANGE_DIRECTION);
    }
    else if(state == PICKUP_DELAY) {
        desiredDir = backDirection(positon);
        setState(CHANGE_DIRECTION);
    } */
}

// STATE SETUP
void setState(State newState) {
    if(newState != state) {
        prevState = state;
        state = newState;
    }
    // do setup for the new state
    if(state != LINE_FOLLOWING && state != DEPOSIT)
        digitalWrite(blueLEDPin, LOW);
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
        case DEPOSIT_2:
            deposit2Setup();
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

int blueLEDTimer = 0;

void stateSensors() {
    // Button sets inactive state
    if(prevButton == HIGH && button == LOW)
        setState(state == INACTIVE ? prevState : INACTIVE);

    // Ultrasonic sensor changes LED state
    if(state == LINE_FOLLOWING || state == DEPOSIT) {
        if(blueLEDTimer == 0) {
            blueLEDTimer = 50;
            digitalWrite(blueLEDPin, HIGH);
        }
        else if(blueLEDTimer == 25)
            digitalWrite(blueLEDPin, LOW);
        blueLEDTimer--;
    }
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
        case DEPOSIT_2:
            deposit2Loop();
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
    pinMode(ultrasonicPin, INPUT);
    pinMode(redLEDPin, OUTPUT);
    pinMode(greenLEDPin, OUTPUT);
    pinMode(blueLEDPin, OUTPUT);
    // pinMode(errorLEDPin, INPUT);
    // pinMode(tofPin, INPUT);

    //setMovement(R_RIGHT);
    //delay(10000); // 10000 ms / 450 deg / 5 ticks/s * 90 deg = 
    //setMovement(STOPPED);
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
    //Serial.println(ultrasonic);
    stateSensors();
    statesLoop();      
    delay(5);
}
