#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>
#include <Encoder.h>

#define GEARING  34
#define PPR  48

//encoder declarations. Do not change!
// Motor encoder external interrupt pins (no other pins allowed!) Pins 18 and 19 are only available on the Arduino Mega
#define ENCA_L  19       // Encoder A input         Yellow
#define ENCB_L  2        // Encoder B input         Green
#define ENCA_R  18        // Encoder A input         Yellow
#define ENCB_R  3        // Encoder B input         Green

Encoder encoderL(ENCA_L, ENCB_L);
Encoder encoderR(ENCB_R, ENCA_R); //due to wrong wiring

float cpr = GEARING * PPR;
long positionLeft;
long positionRight;

unsigned long tNow;
unsigned long tPrev;
long prevPositionLeft;
long prevPositionRight;

//servo declarations
Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards


// motor pins. Do not change
#define PWMA  5         // Motor A PWM control     Orange
#define AIN2  7         // Motor A input 2         Brown
#define AIN1  4       // Motor A input 1         Green
#define BIN1  10       // Motor B input 1         Yellow
#define BIN2  8       // Motor B input 2         Purple
#define PWMB  6       // Motor B PWM control     White

//global variables

int speed = 100;
int startPos = 92; //neutral posistion servo in degrees


int rotation;
double distance;


void setup() { //leave as is
  
  initMotors(); //function to setup motor
  pinMode( 15, INPUT_PULLUP );
  //set steering in neutral position and wait 2 sec
 // myservo.write(startPos)
  prevPositionRight = encoderR.read();
 // Serial.begin(9600);
 
  
}





void loop() {
  forwardA(speed);
  forwardB(speed);
  positionRight = encoderR.read();

 
  rotation = positionRight/(GEARING*PPR);
  
  distance = positionRight/cpr*3.14*6.86;
  
  if(distance >= 100)
  {speed = 0;
  brakeA;
  brakeB;
  }


 
}





//functions
void forwardA(uint16_t pwm) { //addapted to wiring
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, pwm);
}
void forwardB(uint16_t pwm) { //addapted to wiring
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, pwm);
}
void reverseA(uint16_t pwm) { //addapted to wiring
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, pwm);
}
void reverseB(uint16_t pwm) { //addapted to wiring
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMB, pwm);
}
void brakeA() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
}
void brakeB() {
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

void initMotors() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

}
