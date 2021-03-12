// MAE 322
#include <Servo.h>

//***********************************************************
//**************** CONSTANT DECLARATIONS ********************
//***********************************************************
const int transmitterConstant = 21000;
const int autonomousActivation = 1600;
const int motorMin = 1000;
const int motorMax = 2000;
const int motorZero = 1500; 

int Rwheel, Lwheel;

// *** pin numbers are arbitrary ***
int Ch1, Ch2, Ch3, Ch4, Ch5, Ch6;
const int Ch1Pin = 1;
const int Ch2Pin = 2;
const int Ch3Pin = 3;
const int Ch4Pin = 4;
const int Ch5Pin = 5;
const int Ch6Pin = 6;

// *** pin numbers are arbitrary ***
const int rPhotoSensorPin = A0;
const int rPhotoSensorPin = A1;
const int fDistSensorPin = A2;
const int rDistSensorPin = A3;
const int lDistSensorPin = A4;
const int R_ServoPin = A5;
const int L_ServoPin = A6;
const int Medkit_ServoPin = A7;

bool finishedLineFollowing = false;
bool finishedWall = false;
bool finishedChute = false;
bool finishedLightNavigation = false;
bool finishedMedkit = false;
bool faceWall = false;
bool treadsActivated = false;

// Declaring servo as global variables
Servo R_Servo;
Servo L_Servo;
Servo Medkit_Servo;

// Declaring sensors as global variables
int rPhotoSensor, lPhotoSensor, fDistSensor, rDistSensor, lDistSensor, photoSensorDiff, distSensorDiff;

//***********************************************************
//*********************** setup() ***************************
//***********************************************************
void setup() {
  pinMode(Ch1Pin, INPUT);
  pinMode(Ch2Pin, INPUT);
  pinMode(Ch3Pin, INPUT);
  pinMode(Ch4Pin, INPUT);
  pinMode(Ch5Pin, INPUT);
  pinMode(Ch6Pin, INPUT);
  
  pinMode(rPhotoSensorPin, INPUT);
  pinMode(lPhotoSensorPin, INPUT);
  pinMode(fDistSensorPin, INPUT);
  pinMode(rDistSensorPin, INPUT);
  pinMode(lDistSensorPin, INPUT);

  R_Servo.attach(R_ServoPin);
  L_Servo.attach(L_ServoPin);
  Medkit_Servo.attach(Medkit_ServoPin);
}

//***********************************************************
//*********************** forward() *************************
//***********************************************************
void forward(int delayTime, int power)
{
  int right = motorZero - 100*power;
  int left = motorZero + 100*power;

  constrain(right, motorLow, motorHigh);
  constrain(left, motorLow, motorHigh);
  
  R_Servo.writeMicroseconds(right);  // sets the servo position
  L_Servo.writeMicroseconds(left);   // sets the servo position
  
  delay(delayTime);
}

//***********************************************************
//*********************** turnLeft() ************************
//***********************************************************
void turnLeft(int delayTime, int power)
{
  int right = motorZero - 100*power;
  int left = motorZero;

  constrain(right, motorLow, motorHigh);
  constrain(left, motorLow, motorHigh);
  
  R_Servo.writeMicroseconds(right);  // sets the servo position
  L_Servo.writeMicroseconds(left);   // sets the servo position
  
  delay(delayTime);
}

//***********************************************************
//*********************** turnRight() ***********************
//***********************************************************
void turnRight(int delayTime, int power)
{
  int right = motorZero;
  int left = motorZero + 100*power;

  constrain(right, motorLow, motorHigh);
  constrain(left, motorLow, motorHigh);
  
  R_Servo.writeMicroseconds(right);  // sets the servo position
  L_Servo.writeMicroseconds(left);   // sets the servo position
  
  delay(delayTime);
}

//***********************************************************
//*********************** backward() ************************
//***********************************************************
void backward(int delayTime, int power)
{
  int right = motorZero + 100*power;
  int left = motorZero - 100*power;

  constrain(right, motorLow, motorHigh);
  constrain(left, motorLow, motorHigh);
  
  R_Servo.writeMicroseconds(right);  // sets the servo position
  L_Servo.writeMicroseconds(left);   // sets the servo position
  
  delay(delayTime);
}

//***********************************************************
//********************** stopRobot() ************************
//***********************************************************
void stopRobot(int delayTime)
{
  R_Servo.writeMicroseconds(motorZero);  // sets the servo position
  L_Servo.writeMicroseconds(motorZero);   // sets the servo position
  
  delay(delayTime);
}

//***********************************************************
//******************** checkSensors() ***********************
//***********************************************************
void checkSensors() {
  // Read photo sensors
  rPhotoSensor = analogRead(rPhotoSensorPin);
  lPhotoSensor = analogRead(lPhotoSensorPin);
  photoSensorDiff = abs(lPhotoSensor - rPhotoSensor);

  // Read distance sensors
  for (int i = 0; i < 4; i++) {
    fDistSensor += analogRead(fDistSensorPin);
    rDistSensor += analogRead(rDistSensorPin);
    lDistSensor += analogRead(lDistSensorPin);
  }
  fDistSensor /= 5;
  rDistSensor /= 5;
  lDistSensor /= 5;

  distSensorDiff = rDistSensor - lDistSensor;
}

//***********************************************************
//******************** lineFollowing() **********************
//***********************************************************
void lineFollowing() {
  
}

//***********************************************************
//******************** wallTraverse() **********************
//***********************************************************
void wallTraverse() {
  if (fDistSensor < 500 && !faceWall) {
    forward(200, 2);
  }
  else {
    facewall = true;
    treadsActivated = true;
    forward(500, 5);
    stopRobot(200);
    finishedWall = true;
  }
}
//***********************************************************
//******************** chuteTraverse() **********************
//***********************************************************
void chuteTraverse() {
 
  if (rDistSensor > 100 && lDistSensor > 100) {
    if(distSensorDiff > 50) {
      turnLeft(10, 2);
    }
    else if(distSensorDiff < -50) {
      turnRight(10, 2);
    }
    else {
      forward(10, 3);
    }
  }
  else {
    stopRobot(100);
    finishedChute = true;
  } 
}

//***********************************************************
//******************** lightNavigation() **********************
//***********************************************************
void lightNavigation() {

  if (lPhotoSensor > 350 && rPhotoSensor > 350) {
    if (lPhotoSensor > rPhotoSensor) {
      turnRight(100, 2);
    }
    else {
      turnLeft(100, 2);
    }
  }
  else if (photoSensorDiff > 70) {
    if (lPhotoSensor < rPhotoSensor) {
      turnLeft(100, 2);
      forward(100, 3);
    }
    else {
      turnRight(100, 2);
      forward(100, 3);
      }
  }
  else {
    forward(200, 3);
  }
  finishedLightNavigation = true;
}

//***********************************************************
//******************** placeMedkit() **********************
//***********************************************************
void placeMedkit() {

  Medkit_Servo.writeMicroseconds(1600);
  delay(500);
  Medkit_Servo.writeMicroseconds(motorZero);
  backward(100, 3);
  finishedMedkit = true;
}

//***********************************************************
//********************** autonomous() ***********************
//***********************************************************
void autonomous() {
  checkSensors();

  if (!finishedLineFollowing)
    lineFollowing();
  else if (finishedLineFollowing && !finishedWall)
    wallTraverse();
  else if (finishedLineFollowing && finishedWall && !finishedChute)
    chuteTraverse();
  else if (finishedLineFollowing && finishedWall && finishedChute && !finishedLightNavigation)
    lightNavigation();
  else if (finishedLineFollowing && finishedWall && finishedChute && finishedLightNavigation && !finishedMedkit)
    placeMedkit();
  else
    stopRobot();
}

//***********************************************************
//************************* loop() **************************
//***********************************************************
void loop(){
  Ch1 = pulseIn(Ch1Pin, HIGH, 21000);
  Ch2 = pulseIn(Ch2Pin, HIGH, 21000);
  Ch3 = pulseIn(Ch3Pin, HIGH, 21000);
  Ch4 = pulseIn(Ch4Pin, HIGH, 21000);
  Ch5 = pulseIn(Ch5Pin, HIGH, 21000);
  Ch6 = pulseIn(Ch6Pin, HIGH, 21000);

  checkSensors();

  if (Ch5 > autonomousActivation)
    autonomous();
  else
    driveServosRC();
}

//***********************************************************
//********************* driveServosRC() *********************
//***********************************************************
void driveServosRC() {
  if (Ch2 <= 1500) {
    Lwheel = Ch1 + Ch2 - 1500;
    Rwheel = Ch1 - Ch2 + 1500;
    SetLimits();
  }
  if (Ch2 > 1500) {
    int Ch1_mod = map(Ch1, 1000, 2000, 1000, 2000); // Invert the Ch1 axis to keep the math similar
    Lwheel = Ch1_mod + Ch2 - 1500;
    Rwheel = Ch1_mod - Ch2 + 1500;
    pulseMotors();
  }
}




//*******************   pulseMotors  ***************************
//pulses either mapped or direct signals
//**************************************************************
void pulseMotors() {

  //un-comment the next two to map a control range.
  //*** Take the standard range of 1000 to 2000 and frame it to your own minimum and maximum
  //*** for each wheel.
  Rwheel = map(1700, 1000, 2000, 1000, 2000);
  Lwheel = map(1300, 1000, 2000, 1000, 2000);
  R_Servo.writeMicroseconds(Rwheel);
  L_Servo.writeMicroseconds(Lwheel);

  // un-comment this line do display the value being sent to the motors
  //  PrintWheelCalcs(); //REMEMBER: printing values slows reaction times

}



  
 
