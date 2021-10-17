/*
The main use of this code is to obtain encoder values to create a Millimeter/Encoder Count conversion 
*/

//include libraries
#include "CytronMotorDriver.h" 
#include "Encoder.h"

// declare encoder input pins
// motor 1 and motor 2 are the left and right motors respectively, as seen from above with tripod wheel in the rear
const int left_motor_ch_A = 2;
const int left_motor_ch_B = 3;
const int right_motor_ch_A = 18;
const int right_motor_ch_B = 19;

//instantiate the encoder objects
Encoder left_motor_Enc(left_motor_ch_B, left_motor_ch_A); // swapped channels A & B so that both motors have positive readings when moving forward
Encoder right_motor_Enc(right_motor_ch_A, right_motor_ch_B);



//declare encoder counter variables
volatile long left_motor_counter = 0;
volatile long right_motor_counter = 0;

//instantiate the motor objects
CytronMD leftMotor(PWM_DIR, 5, 4); // Motor 1 EN = Pin 5, DIR = 4
CytronMD rightMotor(PWM_DIR, 6, 7); // Motor 2 EN = Pin 6, Dir = 7

//declare motor PWM values, ie. -255 to 255
volatile int left_motor_speed = 60;
volatile int right_motor_speed = 60;


void setup() {
  // setup code runs once at the beginning of the program

  //declare encoder pins as inputs
  pinMode (left_motor_ch_A, INPUT_PULLUP);
  pinMode (left_motor_ch_B, INPUT_PULLUP);
  pinMode (right_motor_ch_A, INPUT_PULLUP);
  pinMode (right_motor_ch_B, INPUT_PULLUP);
  
  //sets the data rate in bits per second (baud)
  Serial.begin(9600);

}

//the loop will print out the encoder count variables
//if the distance and encoder count is known, it is possible
//to determine a Millimeter/count ratio which can be used 
//to determine the distance the robot has moved
void loop() {
  //retreive the motor encoder count
  left_motor_counter = left_motor_Enc.read();

  //print the encoder count
  Serial.print("Left Motor Counter: ");
  Serial.println(left_motor_counter);

  //delay 1 second
  delay(1000);
}



