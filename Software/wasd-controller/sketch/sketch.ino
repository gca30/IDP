#include "WiFiS3.h"
#include <Adafruit_MotorShield.h>

char ssid[] = "M208 Arduino"; 
char pass[] = "weareone";

const int moveSpeed = 150;
const int outerSpeed = 150;
const int innerSpeed = 50;
const int rotSpeed = 100;
int status = WL_IDLE_STATUS;
WiFiServer server(80);
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor* motor1 = AFMS.getMotor(1);
Adafruit_DCMotor* motor2 = AFMS.getMotor(3);

void setup() {
    Serial.begin(9600);
    while(!Serial);
    if(WiFi.status() == WL_NO_MODULE) {
        Serial.println("No WiFi module");
        while(true);
    }
    // WiFi.config(IPAddress(192,48,56,2)); // if we need to override the IP
    status = WiFi.beginAP(ssid, pass);
    if (status != WL_AP_LISTENING) {
        Serial.println("Creating access point failed");
        while (true);
    }

    if (!AFMS.begin()) {
      Serial.println("Could not find Motor Shield. Check wiring.");
      while (1);
    }
    motor1->setSpeed(moveSpeed);
    motor1->run(FORWARD);
    motor1->run(RELEASE);
    motor2->setSpeed(moveSpeed);
    motor2->run(FORWARD);
    motor2->run(RELEASE);
    Serial.println("Motors set up");
    
    delay(10000);
    server.begin();
    Serial.println("Server started");
}

void updateMotors(char data) {
    int forward = 0, turn = 0;
    if(data & 1 << 0) // W
        forward += 1;
    if(data & 1 << 1) // A
        turn -= 1;
    if(data & 1 << 2) // S
        forward -= 1;
    if(data & 1 << 3) // D
        turn += 1;

    // motor2 is on the right and in the correct orientation
    // motor1 is on the left and flipped horizontally
    
    if(forward == 0 && turn == 0) {
        motor1->run(RELEASE);
        motor2->run(RELEASE);
    }
    if(forward == 1 && turn == 0) {
        motor1->setSpeed(moveSpeed);
        motor2->setSpeed(moveSpeed);
        motor1->run(BACKWARD);
        motor2->run(FORWARD);
    }
    else if(forward == -1 && turn == 0) {
        motor1->setSpeed(moveSpeed);
        motor2->setSpeed(moveSpeed);
        motor1->run(FORWARD);
        motor2->run(BACKWARD);
    }
    else if(forward == 0 && turn == 1) {
        // turn right
        motor1->setSpeed(rotSpeed);
        motor2->setSpeed(rotSpeed);
        motor1->run(BACKWARD);
        motor2->run(BACKWARD);
        
    }
    else if(forward == 0 && turn == -1) {
        // turn left
        motor1->setSpeed(rotSpeed);
        motor2->setSpeed(rotSpeed);
        motor1->run(FORWARD);
        motor2->run(FORWARD);
    }
    else if(forward == 1 && turn == 1) {
        // turn right while moving forward
        motor1->setSpeed(outerSpeed);
        motor2->setSpeed(innerSpeed);
        motor1->run(BACKWARD);
        motor2->run(FORWARD);
    }
    else if(forward == 1 && turn == -1) {
        motor1->setSpeed(innerSpeed);
        motor2->setSpeed(outerSpeed);
        motor1->run(BACKWARD);
        motor2->run(FORWARD);
    }
    else if(forward == -1 && turn == 1) {
        // turn right while moving backwards
        motor1->setSpeed(outerSpeed);
        motor2->setSpeed(innerSpeed);
        motor1->run(FORWARD);
        motor2->run(BACKWARD);
    }
    else if(forward == -1 && turn == -1) {
        motor1->setSpeed(innerSpeed);
        motor2->setSpeed(outerSpeed);
        motor1->run(FORWARD);
        motor2->run(BACKWARD);
    }
}

void loop() {
    // Check for new status
    if(status != WiFi.status()) {
        status = WiFi.status();
        if(status == WL_AP_CONNECTED)
            Serial.println("Device connected to AP");
        else
            Serial.println("Device disconnected from AP");
    }
    // Connect to client
    WiFiClient client = server.available();   
    if(client) {
        Serial.println("New client");
        while(client.connected()) {
            if(client.available()) {
                char data = client.read();
                Serial.write(data);
                updateMotors(data);
            }
        } 
    }
    client.stop();
    Serial.println("Client disconnected");
}
