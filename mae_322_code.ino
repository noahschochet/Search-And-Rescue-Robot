// MAE 322 - PSEUDOCODE

//***********************************************************
//**************** CONSTANT DECLARATIONS ********************
//***********************************************************
const int transmitterConstant = 21000;
const int autonomousActivation = 1600;
const int motorMin = 1000;
const int motorMax = 2000;
const int motorZero = 1500; 

// *** pin numbers are arbitrary ***
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

bool finishedLineFollowing = false;
bool finishedWall = false;
bool finishedChute = false;
bool finishedLightNavigation = false;
bool finishedMedkit = false;


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

  //...
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
  
}


//***********************************************************
//******************** chuteTraverse() **********************
//***********************************************************
void chuteTraverse() {
  
}

//***********************************************************
//******************** lightNavigation() **********************
//***********************************************************
void lightNavigation() {
  
}

//***********************************************************
//******************** placeMedkit() **********************
//***********************************************************
void placeMedkit() {
  
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
    SetLimits();
  }
}

//********************** MixLimits() ***************************
//*******  Make sure values never exceed ranges  ***************
//******  For most all servos and like controlers  *************
//****   control must fall between 1000uS and 2000uS  **********
//**************************************************************
void SetLimits() {
  if (Lwheel < 1000) {// Can be set to a value you don't wish to exceed
    Lwheel = 1000;    // to adjust maximums for your own robot
  }
  if (Lwheel > 2000) {// Can be set to a value you don't wish to exceed
    Lwheel = 2000;    // to adjust maximums for your own robot
  }
  if (Rwheel < 1000) {// Can be set to a value you don't wish to exceed
    Rwheel = 1000;    // to adjust maximums for your own robot
  }
  if (Rwheel > 2000) {// Can be set to a value you don't wish to exceed
    Rwheel = 2000;    // to adjust maximums for your own robot
  }
  pulseMotors();
}

//*******************   pulseMotors  ***************************
//pulses either mapped or direct signals generated from Mixlimits
//**************************************************************
void pulseMotors() {
  //un-comment the next two line to drive the wheels directly with the MaxLimits Set
  //  R_Servo.writeMicroseconds(Rwheel);
  //  L_Servo.writeMicroseconds(Lwheel);

  //un-comment the next two to map a control range.
  //*** Take the standard range of 1000 to 2000 and frame it to your own minimum and maximum
  //*** for each wheel.
  Rwheel = map(Rwheel, 1000, 2000, 1000, 2000);
  Lwheel = map(Lwheel, 1000, 2000, 1000, 2000);
  R_Servo.writeMicroseconds(Rwheel);
  L_Servo.writeMicroseconds(Lwheel);

  // un-comment this line do display the value being sent to the motors
  //  PrintWheelCalcs(); //REMEMBER: printing values slows reaction times

}



  
 
