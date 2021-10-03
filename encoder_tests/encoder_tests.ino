//use this code to test encoder readings and troubleshoot

//include libraries
#include "CytronMotorDriver.h" 
#include "Encoder.h"

// declare encoder input pins
// motor 1 and motor 2 are the left and right motors respectively, as seen from above with tripod wheel in the rear
const int m1_ch_A = 2;
const int m1_ch_B = 3;
const int m2_ch_A = 18;
const int m2_ch_B = 19;

// Change the encoder pin two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability

//instantiate the encoder objects
Encoder m1_Enc(m1_ch_A, m1_ch_B);
Encoder m2_Enc(m2_ch_A, m2_ch_B);


//instantiate the motor objects
CytronMD motor1(PWM_DIR, 5, 4); // Motor 1 EN = Pin 5, DIR = 4
CytronMD motor2(PWM_DIR, 6, 7); // Motor 2 EN = Pin 6, Dir = 7

void setup() {
  Serial.begin(9600);
  Serial.println("Basic Encoder Test:");
}

long oldPosition  = -999;

void loop() {
  //change to motor 1 or motor 2
  motor1.setSpeed(60);
  long newPosition = m1_Enc.read(); // change to encoder 1 or encoder 2
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    Serial.println(newPosition); 
  }
}
