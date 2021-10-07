//hardcoded phase A

//include motor driver library
#include "CytronMotorDriver.h" 
#include "Encoder.h"

// declare encoder input pins
// motor 1 and motor 2 are the left and right motors respectively, as seen from above with tripod wheel in the rear
const int left_motor_ch_A = 2;
const int left_motor_ch_B = 3;
const int right_motor_ch_A = 18;
const int right_motor_ch_B = 19;

//declare mm/pulse constant
//converts encoder ticks to mm
const float thick_wheels_pulses_to_mm = 0.054048211; //thick wheels
const float skinny_wheels_pulses_to_mm = 0.06222775; // skinny wheels
//instantiate the encoder objects
Encoder left_Enc(left_motor_ch_B, left_motor_ch_A); // swapped channels A & B so that both motors have positive readings when moving forward
Encoder right_Enc(right_motor_ch_A, right_motor_ch_B);


//declare encoder counter variables
volatile long left_motor_counter = 0;
volatile long right_motor_counter = 0;

//instantiate the motor objects
CytronMD leftMotor(PWM_DIR, 5, 4); // Motor 1 EN = Pin 5, DIR = 4
CytronMD rightMotor(PWM_DIR, 6, 7); // Motor 2 EN = Pin 6, Dir = 7


//declare motor PWM values, ie. -255 to 255
volatile int left_motor_speed;
volatile int right_motor_speed;


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

void loop() {

//********Top Level***************

//top level - first straightaway
forward(205);

//top level - first right turn
rightTurn();

//top level to ramp - second straight away that then enters ramp
forward(1025);


// ********* Transition from first floor to ramp ************

//ramp - first right turn onto ramp
rightTurn();

//ramp - first straight away to first landing
forward(737.50);

// ramp - second right turn on ramp
rightTurn();

//ramp - second straightaway on ramp to second landing
forward(1450);

//ramp - third right turn on ramp
rightTurn();

//ramp - third straight away to bottom of ramp
forward(615);

//ramp - fourth right turn on ramp to face the lower level
rightTurn();

//ramp to lower level - final straight away of ramp to enter lower level
forward(205);




// **********Transition from ramp to lower level ****************8

//lower level - first right turn on lower level
rightTurn();

//lower level - first straight away of lower level 
forward(410);

// lower level - first left turn of lower level
leftTurn();

// lower level - second straight away on lower level
forward(615);

//lower level - last left turn of lower level
leftTurn();

//lower level - move toward minder
forward(205);

}

// **************   pick up miner  **********
void forward(double mm)
{ 

  left_motor_speed = 80;
  right_motor_speed = 80;
  float distance = 0;
  leftMotor.setSpeed(left_motor_speed);
  rightMotor.setSpeed(right_motor_speed);
  left_Enc.readAndReset();

  while(distance<mm)
    {
      left_motor_counter = left_Enc.read();
      distance = abs((left_motor_counter*skinny_wheels_pulses_to_mm));
    }

    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
    delay(2000);
}

void leftTurn()
{ 

  left_motor_speed = -60;
  right_motor_speed = 60;
  float distance = 0;
  leftMotor.setSpeed(left_motor_speed);
  rightMotor.setSpeed(right_motor_speed);
  left_Enc.readAndReset();

  while(distance<96)
    {
      left_motor_counter = left_Enc.read();
      distance = abs((left_motor_counter*skinny_wheels_pulses_to_mm));
    }

    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
    delay(2000);
}

void rightTurn()
{ 

  left_motor_speed = 60;
  right_motor_speed = -60;
  float distance = 0;
  leftMotor.setSpeed(left_motor_speed);
  rightMotor.setSpeed(right_motor_speed);
  left_Enc.readAndReset();

  while(distance<96)
    {
      left_motor_counter = left_Enc.read();
      distance = abs((left_motor_counter*skinny_wheels_pulses_to_mm));
    }

    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
    delay(2000);
}








    // even more hardcoded version
    // leftMotor.setSpeed(left_motor_speed);
    // rightMotor.setSpeed(right_motor_speed);
    
    // // go straight for XX mm
    // while(distance<200)
    // {
    //   left_motor_counter = left_Enc.read();
    //   distance = abs((left_motor_counter*pulses_to_mm));
    // }

    // //stop for 2 seconds
    // leftMotor.setSpeed(0);
    // rightMotor.setSpeed(0);
    // delay(2000);

    // //turn left
    // left_motor_speed = -60;
    // right_motor_speed = 60;
    // distance = 0;
    // left_Enc.readAndReset();
    // leftMotor.setSpeed(left_motor_speed);
    // rightMotor.setSpeed(right_motor_speed);
    // while(distance<96)
    // {
    //   left_motor_counter = left_Enc.read();
    //   distance = abs((left_motor_counter*pulses_to_mm));
    // }

    // //Stop for 2 seconds
    // leftMotor.setSpeed(0);
    // rightMotor.setSpeed(0);
    // delay(2000);


    // // go straight for XX mm
    // left_motor_speed = 60;
    // right_motor_speed = 60;
    // distance = 0;
    // left_Enc.readAndReset();
    // leftMotor.setSpeed(left_motor_speed);
    // rightMotor.setSpeed(right_motor_speed);
    // while(distance<200)
    // {
    //   left_motor_counter = left_Enc.read();
    //   distance = abs((left_motor_counter*pulses_to_mm));
    // }
    // delay(100000);