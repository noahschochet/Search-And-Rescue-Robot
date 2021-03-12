/*
  Complete RC Control and Autonomous light sensor navigation.
  MAE 322
*/

#include <Servo.h>

// Create Variables to hold the Receiver signals
int Ch1, Ch2, Ch3, Ch4, Ch5, Ch6;
int Rwheel;               // Variable to hold R wheel speed 
int Lwheel;               // Variable to hold L wheel speed
const int lPhoto = A1;    // Left Photoresistor
const int rPhoto = A0;    // Right Photoresistor
const int sharpPin = A2;  // Sharp Sensor
const int LED = 13;       // Onboard LED location
int lPhotoVal;            // Variable to store L photoresistor value
int rPhotoVal;            // Variable to store R photoresistor value
int sharpVal;             // Variable to store Sharp Sensor value
int valDif;               // Variable to store difference between photo values
int rSpeed, lSpeed;       // Variables to hold autonomous speed changes for each wheel
bool acquiring;

// Create Servo Objects as defined in the Servo.h files
Servo L_Servo;  // Servo DC Motor Driver (Designed for RC cars)
Servo R_Servo;  // Servo DC Motor Driver (Designed for RC cars)

//**************************************************************
//*****************  Setup  ************************************
//**************************************************************
void setup() {
  // Set the pins that the transmitter will be connected to all to input
  pinMode(12, INPUT); //I connected this to Chan1 of the Receiver
  pinMode(11, INPUT); //I connected this to Chan2 of the Receiver
  pinMode(10, INPUT); //I connected this to Chan3 of the Receiver
  pinMode(9, INPUT); //I connected this to Chan4 of the Receiver
  pinMode(8, INPUT); //I connected this to Chan5 of the Receiver
  pinMode(7, INPUT); //I connected this to Chan6 of the Receiver
  pinMode(LED, OUTPUT);//Onboard LED to output for diagnostics
// Attach Speed controller that acts like a servo to the board
  R_Servo.attach(1); //Pin 0
  L_Servo.attach(2); //Pin 1
  rSpeed = 1300;
  lSpeed = 1700;
  //Flash the LED on and Off 10x before entering main loop
  for (int i = 0; i < 10; i++) {
    digitalWrite(13, HIGH);
    delay(200);
    digitalWrite(13, LOW);
    delay(200);
  }
  acquiring = true;
  //Flash the LED on and Off 10x End
  Serial.begin(9600);
}
//************************  loop()  ****************************
//**********************  Main Loop  ***************************
//**************************************************************
void loop()
{
  //  Ch1 = pulseIn(12, HIGH, 21000); // Capture pulse width on Channel 1
  //  Ch2 = pulseIn(11, HIGH, 21000); // Capture pulse width on Channel 2
  //  Ch3 = pulseIn(10, HIGH, 21000);  // Capture pulse width on Channel 3
  //  Ch4 = pulseIn(9, HIGH, 21000);  // Capture pulse width on Channel 4
  //  Ch5 = pulseIn(8, HIGH, 21000); // Capture pulse width on Channel 5
  //  Ch6 = pulseIn(7, HIGH, 21000); // Capture pulse width on Channel 6
  //
  //  TestWheels();
  //  fowardSlow();
  //  DriveServosRC(); // Drive Motors under RC control
  //  LEDMix3_4(); // Display mixing or LED colors
  Ch5Check(); // brightens and darkens 2 LEDs with proportional stick control
  //  PrintRC(); //Print Values for RC Mode Diagnostics

}

//**********************  Ch5Check()  **************************
//********************** Test Channel 5   **********************
//**************************************************************
void Ch5Check() {
  Ch5 = pulseIn(8, HIGH, 21000); // Capture pulse width on Channel 5
  if (Ch5 > 1600) {
    digitalWrite(LED, HIGH);
    autonomous();
  }
  else {
    Ch1 = pulseIn(12, HIGH, 21000); // Capture pulse width on Channel 1
    Ch2 = pulseIn(11, HIGH, 21000); // Capture pulse width on Channel 2
    Ch3 = pulseIn(10, HIGH, 21000);  // Capture pulse width on Channel 3
    Ch4 = pulseIn(9, HIGH, 21000);  // Capture pulse width on Channel 4
    digitalWrite(LED, LOW);
    DriveServosRC();
  }
}
//**********************  autoMode()  **************************
//********************** Autonomous Mode   **********************
//**************************************************************
void autonomous() {
  if (checkSensors()) return;
  
  if (lPhotoVal > 350 && rPhotoVal > 350) {
//    Serial.println("looking");
//    Serial.println(lPhotoVal);
//    Serial.println(rPhotoVal);
    if (lPhotoVal > rPhotoVal) {
      TRightSlow(1500, 1);
    }
    else {
      TLeftSlow(1500, 1);
    }
  }
  else if (valDif > 70) {
    if (lPhotoVal < rPhotoVal) {
      rSpeed = rSpeed + 5;
      if (rSpeed >= 1480) {
        rSpeed = 1480;
      }
      TLeftSlow(rSpeed, 1);
//      Serial.println("rSpeed");
//      Serial.println(rSpeed);
    }
    else {
      lSpeed = lSpeed - 5;
      if (lSpeed <= 1540) {
        lSpeed = 1540;
      }
      TRightSlow(lSpeed, 1);
//      Serial.println("lSpeed");
//      Serial.println(lSpeed);
    }
  }
  else {
    rSpeed = 1300;
    lSpeed = 1700;
    Forward(10);
//    Serial.println("forward");
  }
// printSensors();

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

//*******************  DriveServosRC()  ************************
//******  Use the value collected from Ch1 and Ch2  ************
//******  on a single stick to relatively calculate  ***********
//****  speed and direction of two servo driven wheels *********
//**************************************************************
void DriveServosRC()
{
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
//*****************  Forward(int Dlay)   ***********************
//              Move the robot Slowly Forward
//**************************************************************
void Forward(int Dlay)
{
  R_Servo.writeMicroseconds(1300);  // sets the servo position
  L_Servo.writeMicroseconds(1700);   // sets the servo position
  delay(Dlay);
}
//*****************  Reverse(int Dlay)   ***********************
//                   Reverse the robot
//**************************************************************
void Reverse(int Dlay)
{
  R_Servo.writeMicroseconds(2000);  // sets the servo position
  L_Servo.writeMicroseconds(1000);   // sets the servo position
  delay(Dlay);
}
//*****************  stopBot(int Dlay)   ***********************
//                    Stop the robot
//**************************************************************
void stopBot(int Dlay)
{
  R_Servo.writeMicroseconds(1500);  // sets the servo position
  L_Servo.writeMicroseconds(1500);   // sets the servo position
  delay(Dlay);
}
//************* TLeftSlow(int rVal,int Dlay) *******************
//        left turn with tapering speed and a duration
//**************************************************************
void TLeftSlow(int rVal, int Dlay)
{
  R_Servo.writeMicroseconds(rVal);  // sets the servo position
  L_Servo.writeMicroseconds(1700);   // sets the servo position
  delay(Dlay);
}
//************* TRightSlow(int lVal,int Dlay) *******************
//        Right turn with tapering speed and a duration
//**************************************************************
void TRightSlow(int lVal, int Dlay)
{
  R_Servo.writeMicroseconds(1300);  // sets the servo position
  L_Servo.writeMicroseconds(lVal);   // sets the servo position
  delay(Dlay);
}
//******************** checkSensors() **************************
// Check value of Sensors         Stop bot if object is close
//**************************************************************
bool checkSensors()
{
  rPhotoVal = analogRead(rPhoto);
  lPhotoVal = analogRead(lPhoto);
  valDif = abs(rPhotoVal - lPhotoVal); // looking for threshold
  sharpVal = analogRead(sharpPin);
  for (int i = 0; i <= 3; i++) {
    sharpVal = sharpVal + analogRead(sharpPin);
  }
  sharpVal = sharpVal / 5;
  if (sharpVal >= 500) {//changed from 500
    stopBot(5000);
    return true;
  }
  return false;
}
//******************** printSensors() **************************
// Print the sensor values  Slows robot when in use!!!
//**************************************************************
void printSensors() {
  Serial.println("Right Value = " + (String)rPhotoVal);
  Serial.println("Left Value = " + (String)lPhotoVal);
  Serial.println("Difference Value = " + (String)valDif);
  Serial.println("Sharp Value = " + (String)sharpVal);
  Serial.println(" ");
  delay(100);
}

//**********************  PrintRC()  ***************************
//***  Simply print the collected RC values for diagnostics  ***
//**************************************************************
void PrintRC()
{ // print out the values you read in:
  Serial.println(" RC Control Mode ");
  Serial.print("Value Ch1 = ");
  Serial.println(Ch1);
  Serial.print("Value Ch2 = ");
  Serial.println(Ch2);
  Serial.print("Value Ch3 = ");
  Serial.println(Ch3);
  Serial.print("Value Ch4 = ");
  Serial.println(Ch4);
  Serial.print("Control = ");
  Serial.println(Ch5);
  Serial.print("Value Ch6 = ");
  Serial.println(Ch6);
  Serial.println(" ");
  delay(1000);
}
