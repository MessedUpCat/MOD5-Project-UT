//configurable items
#define SERIALPORT Serial  //define debug output. Serial is via USB, Serial3 is bluetooth
#define BAUDRATE 9600      //9600 for Serial3
#define GEARING 34         //47, 31 motor gearing. Change to your motor
#define PPR 48             //Pulses per revolution. See rotary encode spec


//libraries used
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <Adafruit_Sensor.h>

//other libraries: included in Arduino
#include <Wire.h>
#include <Servo.h>
#include <Encoder.h>


//encoder declarations. Do not change!
// Motor encoder external interrupt pins (no other pins allowed!) Pins 18 and 19 are only available on the Arduino Mega
#define ENCA_L 19  // Encoder A input         Yellow
#define ENCB_L 2   // Encoder B input         Green
#define ENCA_R 18  // Encoder A input         Yellow
#define ENCB_R 3   // Encoder B input         Green

Encoder encoderL(ENCA_L, ENCB_L);
Encoder encoderR(ENCB_R, ENCA_R);  //due to wrong wiring

float cpr = GEARING * PPR;
long positionLeft;
long positionRight;

unsigned long tNow;
unsigned long tPrev;
long prevPositionLeft;
long prevPositionRight;

//servo declarations
Servo myservo;
// twelve servo objects can be created on most boards


// motor pins. Do not change
#define PWMA 5   // Motor A PWM control     Orange
#define AIN2 7   // Motor A input 2         Brown
#define AIN1 4   // Motor A input 1         Green
#define BIN1 10  // Motor B input 1         Yellow
#define BIN2 8   // Motor B input 2         Purple
#define PWMB 6   // Motor B PWM control     White


/* Assign a unique ID to this sensor at the same time */
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

/* Assign a unique ID to this sensor at the same time */
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);

//global variables
int pos = 0;  // variable to store the servo position
int speedA = 0;
int speedB = 0;
int startPos = 90;  //neutral posistion servo in degrees

float gyrox = 0;
float gyroy = 0;
float gyroz = 0;

float accx = 0;
float accy = 0;
float accz = 0;

float dT = 0;
long dPosL = 0;
long dPosR = 0;
float distanceR = 0;
float distanceL =0;

float VelocityL = 0;  //actual calculation in pulses per second
float VelocityR = 0;

float nTurnsL = 0;
float nTurnsR = 0;


int speed = 80;





void setup() {  //leave as is

  SERIALPORT.begin(BAUDRATE);
  while (!SERIALPORT) {
    // wait for SERIALPORT to start
  }
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object

  initMotors();  //function to setup motor
  initIMU();     //init IMU. Change ranges in this function
  pinMode(15, INPUT_PULLUP);

  //set steering in neutral position and wait 2 sec
  myservo.write(startPos);
  delay(5000);

  //init for rotary speeds
  tPrev = micros();
  prevPositionLeft = encoderL.read();
  prevPositionRight = encoderR.read();
}




void loop() {
  refreshSensorData();

  //in this example the motorspeed and servo are changed sinusoidal
  //speed = 255 * sin(6.28 * millis() / 1000 / 20);          //sinusoidal speed change with T = 20s
  //pos = 40 * sin(6.28 * millis() / 1000 / 10) + startPos;  //servo plusminus 40 degrees from neutralPos


  distanceR = nTurnsR*0.22;
  distanceL = nTurnsL*0.22;
  

  if (distanceL < 1.05  && distanceR < 1.05) {
    pos = 87;
    speedA = speed;
    speedB = speed;
  }
  if (distanceL > 1.05 &&  distanceR > 1.05) {
    pos = 150;
    speedA = speed*1.59 ;
    speedB = speed/1.59;
  }
  if (distanceL > 1.95 &&  distanceR > 1.4) {
    pos = 87;
    speedA = speed;
    speedB = speed;
  }
  if (distanceL > 2.825 &&  distanceR > 2.285) {
    pos = 90;
    speedA = 0;
    speedB = 0;
    brakeA();
    brakeB();
  }


  //set steering position
  myservo.write(pos);  // ranging 0-180 degrees
  forwardA(abs(speedA));
  forwardB(abs(speedB));

  delay(1);
}








/*functions*/
void forwardA(uint16_t pwm) {  //addapted to wiring
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, pwm);
}
void forwardB(uint16_t pwm) {  //addapted to wiring
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, pwm);
}
void reverseA(uint16_t pwm) {  //addapted to wiring
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, pwm);
}
void reverseB(uint16_t pwm) {  //addapted to wiring
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
void initIMU() {
  SERIALPORT.println("Gyroscope Test");
  SERIALPORT.println("");
  /* Initialise the sensor */
  if (!gyro.begin()) {
    /* There was a problem detecting the FXAS21002C ... check your connections
    */
    SERIALPORT.println("Ooops, no FXAS21002C detected ... Check your wiring!");
    while (1)
      ;
  }
  SERIALPORT.println("FXOS8700 Test");
  SERIALPORT.println("");
  /* Initialise the sensor */
  if (!accelmag.begin()) {
    /* There was a problem detecting the FXOS8700 ... check your connections */
    SERIALPORT.println("Ooops, no FXOS8700 detected ... Check your wiring!");
    while (1)
      ;
  }

  /* Set gyro range. (optional, default is 250 dps) */
  // dps = degrees per second. Sensoroutput unit is rad/s
  // gyro.setRange(GYRO_RANGE_250DPS);
  // gyro.setRange(GYRO_RANGE_500DPS);
  // gyro.setRange(GYRO_RANGE_1000DPS);
  // gyro.setRange(GYRO_RANGE_2000DPS);


  /* Display some basic information on this sensor */
  displaySensorDetailsGyro();

  /* Set accelerometer range (optional, default is 2G) */
  // 2G = 2*9,81, output unit in m/s^2
  // accelmag.setAccelRange(ACCEL_RANGE_2G);
  // accelmag.setAccelRange(ACCEL_RANGE_4G);
  // accelmag.setAccelRange(ACCEL_RANGE_8G);

  /* Display some basic information on this sensor */
  displaySensorDetailsAccel();
}

void displaySensorDetailsGyro(void) {
  sensor_t gyroscope;
  gyro.getSensor(&gyroscope);
  SERIALPORT.println("------------------------------------");
  SERIALPORT.println("GYROSCOPE");
  SERIALPORT.println("------------------------------------");
  SERIALPORT.print("Sensor:       ");
  SERIALPORT.println(gyroscope.name);
  SERIALPORT.print("Driver Ver:   ");
  SERIALPORT.println(gyroscope.version);
  SERIALPORT.print("Unique ID:    0x");
  SERIALPORT.println(gyroscope.sensor_id, HEX);
  SERIALPORT.print("Max Value:    ");
  SERIALPORT.print(gyroscope.max_value, 4);
  SERIALPORT.println(" rad/s");
  SERIALPORT.print("Min Value:    ");
  SERIALPORT.print(gyroscope.min_value, 4);
  SERIALPORT.println(" rad/s");
  SERIALPORT.print("Resolution: (no meaning)   ");
  SERIALPORT.print(gyroscope.resolution, 8);
  SERIALPORT.println(" rad/s");
  SERIALPORT.println("------------------------------------");
  SERIALPORT.println("");
  delay(500);
}
void displaySensorDetailsAccel(void) {
  sensor_t accel;
  accelmag.getSensor(&accel);
  SERIALPORT.println("------------------------------------");
  SERIALPORT.println("ACCELEROMETER");
  SERIALPORT.println("------------------------------------");
  SERIALPORT.print("Sensor:       ");
  SERIALPORT.println(accel.name);
  SERIALPORT.print("Driver Ver:   ");
  SERIALPORT.println(accel.version);
  SERIALPORT.print("Unique ID:    0x");
  SERIALPORT.println(accel.sensor_id, HEX);
  SERIALPORT.print("Min Delay:    ");
  SERIALPORT.print(accel.min_delay);
  SERIALPORT.println(" s");
  SERIALPORT.print("Max Value:    ");
  SERIALPORT.print(accel.max_value, 4);
  SERIALPORT.println(" m/s^2");
  SERIALPORT.print("Min Value:    ");
  SERIALPORT.print(accel.min_value, 4);
  SERIALPORT.println(" m/s^2");
  SERIALPORT.print("Resolution:   ");
  SERIALPORT.print(accel.resolution, 8);
  SERIALPORT.println(" m/s^2");
  SERIALPORT.println("------------------------------------");
  SERIALPORT.println("");
}

void refreshSensorData(void) {
  //read rotary sensor position
  positionLeft = encoderL.read();
  positionRight = encoderR.read();



  //calculation of rotation speed of motor
  tNow = micros();                            //timestap of calculation
  dT = (tNow - tPrev) / 1e6;                  //calculate time difference since last measurement in seconds
  dPosL = positionLeft - prevPositionLeft;    //number of pulses since last measurement
  dPosR = positionRight - prevPositionRight;  //dito

  VelocityL = dPosL / dT;  //actual calculation in pulses per second
  VelocityR = dPosR / dT;

  //store variable for next run
  tPrev = tNow;
  prevPositionLeft = positionLeft;
  prevPositionRight = positionRight;

  nTurnsL = positionLeft / cpr;
  nTurnsR = positionRight / cpr;

  SERIALPORT.print(VelocityL / cpr);
  SERIALPORT.print("; ");
  SERIALPORT.print(VelocityR / cpr);
  SERIALPORT.print("; ");
  SERIALPORT.print(" ");

  //reporting sensor data
  SERIALPORT.print("motor; ");
  SERIALPORT.print(speedA);
  SERIALPORT.print("; ");
  SERIALPORT.print(pos);
  SERIALPORT.print("; ");
  SERIALPORT.print(distanceR);  //absolute posistion
  SERIALPORT.print("; ");
  SERIALPORT.print(distanceL);
  SERIALPORT.print("; ");
  SERIALPORT.print(nTurnsL);  //number of turns
  SERIALPORT.print("; ");
  SERIALPORT.print(nTurnsR);
  SERIALPORT.print("; ");
  SERIALPORT.println(" ");

  //retrieve IMU data
  /* Get a new sensor event repeat this over and over*/
  sensors_event_t a, m, g;

  /* Get a new sensor event */
  gyro.getEvent(&g);
  accelmag.getEvent(&a, &m);

  gyrox = g.gyro.x;
  gyroy = g.gyro.y;
  gyroz = g.gyro.z;

  accx = a.acceleration.x;
  accy = a.acceleration.y;
  accz = a.acceleration.z;
}

void displayAccelStatus(void) {
  /* Display the accel results (acceleration is measured in m/s^2) */
  SERIALPORT.print("accel; ");
  SERIALPORT.print(accx, 4);
  SERIALPORT.print("; ");
  SERIALPORT.print(accy, 4);
  SERIALPORT.print("; ");
  SERIALPORT.print(accz, 4);
  SERIALPORT.println(" ");
}
void displayGyroStatus(void) {
  /* Display the results (speed is measured in rad/s) */
  SERIALPORT.print("gyro; ");
  SERIALPORT.print(gyrox, 4);
  SERIALPORT.print("; ");
  SERIALPORT.print(gyroy, 4);
  SERIALPORT.print("; ");
  SERIALPORT.print(gyroz, 4);
  SERIALPORT.print("; ");
  SERIALPORT.println(" ");
}

void calibrationMeter() {
}
