#define MAX_RANG (520)
#define ADC_SOLUTION (1023.0)//ADC accuracy of Arduino UNO is 10bit

int led_red = 3, led_green = 2, led_blue = 3, magnet = 5;
float dist_t, sensity_t, prev_dist = 500;
int ismagnetic = -2, prev_magnetic = -1;

int sensityPin = A0; // select the input pin
void setup() {
 // Serial init
 Serial.begin(9600);
 pinMode(led_red, OUTPUT);
 pinMode(led_green, OUTPUT);
 pinMode(led_blue, OUTPUT);
 pinMode(magnet, INPUT);
}


void ultrasonic() {
// read the value from the sensor:
sensity_t = analogRead(sensityPin);
 // turn the ledPin on
dist_t = sensity_t * MAX_RANG / ADC_SOLUTION;//
if ((dist_t < 6 || dist_t > 518) && (prev_dist < 6 || prev_dist > 518)) {
  //WRITE code for motors to stop 
    ismagnetic = digitalRead(magnet);
   if (ismagnetic == prev_magnetic){
     if (ismagnetic == HIGH) { // check if the input is HIGH
       digitalWrite(led_red, HIGH);
       digitalWrite(led_green, LOW); 
    //enter state code
      } else {
       digitalWrite(led_green, HIGH);
       digitalWrite(led_red, LOW); 
    //enter state code
    }
   } else {
    digitalWrite(led_red, LOW); 
    digitalWrite(led_green, LOW);
    }//if magnetic
} //if distance
    else {
    digitalWrite(led_red, LOW); 
    digitalWrite(led_green, LOW);
  }
  prev_dist = dist_t;
  prev_magnetic = ismagnetic;
  }

void loop() {
ultrasonic();
Serial.print(dist_t,0);
Serial.println("cm");
delay(500);
}
