#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>
#include <Encoder.h>

#define GEARING  34      // gear ratio
#define PPR  48          // pulses per resolution
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

Servo myservo;
//motor shit
#define PWMA  5        // Motor A PWM control     Orange
#define AIN2  7        // Motor A input 2         Brown
#define AIN1  4        // Motor A input 1         Green
#define BIN1  10       // Motor B input 1         Yellow
#define BIN2  8        // Motor B input 2         Purple
#define PWMB  6        // Motor B PWM control     White

//global variables
int pos = 0;    // variable to store the servo position
int speed = 0;
int startPos = 92; //neutral posistion servo in degrees


void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  initMotors(); //function to setup motor
  pinMode( 15, INPUT_PULLUP );
  //set steering in neutral position and wait 2 sec
  myservo.write(startPos);
  delay(2000);
  //init for rotary speeds
  tPrev = micros();
  prevPositionLeft = encoderL.read();;
  prevPositionRight = encoderR.read();
}



void loop() {
  //read rotary sensor position
  positionLeft = encoderL.read();
  positionRight = encoderR.read();

  //in this example the motorspeed and servo are changed sinusoidal
  speed = 255 * sin(6.28 * millis() / 1000 / 20); //sinusoidal speed change with T = 20s
  pos = 40 * sin(6.28 * millis() / 1000 / 10) + startPos; //servo plusminus 40 degrees from neutralPos

  if (speed > 10) {
    forwardA(abs(speed));
    forwardB(abs(speed));
  }
  else if (speed < -10) {
    reverseA(abs(speed));
    reverseB(abs(speed));
  }

  else {
    brakeA();
    brakeB();
  }
  //set steering position
  myservo.write(pos); // ranging 0-180 degrees
  //calculation of rotation speed of motor
  tNow = micros(); //timestap of calculation
  float dT = (tNow - tPrev) / 1e6; //calculate time difference since last measurement in seconds
  long dPosL = positionLeft - prevPositionLeft; //number of pulses since last measurement
  long dPosR = positionRight - prevPositionRight; //dito

  float VelocityL = dPosL / dT; //actual calculation in pulses per second
  float VelocityR = dPosR / dT;

  //store variable for next run
  tPrev = tNow;
  prevPositionLeft = positionLeft;
  prevPositionRight = positionRight;
  delay(200);
}


//functions no need to change it
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

