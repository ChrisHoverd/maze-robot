/* 
The main purpose of this code is to test encoder readings and troubleshoot
This code was adapted from the Encoder.h "Basic" example
*/

//include libraries
#include "CytronMotorDriver.h" 
#include "Encoder.h"

// declare encoder input pins
// motor 1 and motor 2 are the left and right motors respectively, as seen from above with tripod wheel in the rear
const int left_motor_ch_A = 2;
const int left_motor_ch_B = 3;
// const int right_motor_ch_A = 2;
// const int right_motor_ch_B = 3;

// Change the encoder pin two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability

//instantiate the encoder objects
Encoder left_motor_Enc(left_motor_ch_B, left_motor_ch_A); // swapped channels A & B so that both motors have positive readings when moving forward
// Encoder right_motor_Enc(right_motor_ch_A, right_motor_ch_B);


//instantiate the motor objects
CytronMD leftMotor(PWM_DIR, 6, 7); 
CytronMD rightMotor(PWM_DIR, 5, 4); 

void setup() {
  Serial.begin(9600);
  while (!Serial) 
 { 
  
 }
  Serial.println("Basic Encoder Test:");
}

//set initial position to be compared to in the main loop
long oldPosition  = -999;

void loop() {

  //change to motor 1 or motor 2 as needed for troubleshooting
  leftMotor.setSpeed(200);
  // rightMotor.setSpeed(200);

  //checks the newest encoder count
  long newPosition = left_motor_Enc.read(); // change to encoder 1 or encoder 2 based on which motor is turning

  //if statement runs if the encoder has moved since last checked
  if (newPosition != oldPosition) {
    //updates last encoder count
    oldPosition = newPosition;
    //prints the newest encoder count
    Serial.println(newPosition); 
  }
}
