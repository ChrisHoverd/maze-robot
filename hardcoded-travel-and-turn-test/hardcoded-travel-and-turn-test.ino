//the void loop has a brief test of moving forward and doing a turn 
// this is hardcoded and could be used as a last resort

//include motor driver library
#include "CytronMotorDriver.h" 
#include "Encoder.h"
#include "TimerThree.h"

// declare encoder input pins
// motor 1 and motor 2 are the left and right motors respectively, as seen from above with tripod wheel in the rear
const int left_motor_ch_A = 2;
const int left_motor_ch_B = 3;
const int right_motor_ch_A = 18;
const int right_motor_ch_B = 19;

//declare mm/pulse constant
//converts encoder ticks to mm
const float pulses_to_mm = 0.054048211;

//instantiate the encoder objects
Encoder m1_Enc(left_motor_ch_B, left_motor_ch_A); // swapped channels A & B so that both motors have positive readings when moving forward
Encoder m2_Enc(right_motor_ch_A, right_motor_ch_B);


//declare encoder counter variables
volatile long left_motor_counter = 0;
volatile long right_motor_counter = 0;

float distance;

//instantiate the motor objects
CytronMD motor1(PWM_DIR, 5, 4); // Motor 1 EN = Pin 5, DIR = 4
CytronMD motor2(PWM_DIR, 6, 7); // Motor 2 EN = Pin 6, Dir = 7


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

void loop() {
  // put your main code here, to run repeatedly:

    motor1.setSpeed(left_motor_speed);
    motor2.setSpeed(right_motor_speed);

    while(distance<200)
    {
      left_motor_counter = m1_Enc.read();
      distance = abs((left_motor_counter*pulses_to_mm));
    }

    motor1.setSpeed(0);
    motor2.setSpeed(0);
    delay(2000);
    left_motor_speed = 60;
    right_motor_speed = -60;
    motor1.setSpeed(left_motor_speed);
    motor2.setSpeed(right_motor_speed);
    distance = 0;
    m1_Enc.readAndReset();
    while(distance<96)
    {
      left_motor_counter = m1_Enc.read();
      distance = abs((left_motor_counter*pulses_to_mm));
    }
    motor1.setSpeed(0);
    motor2.setSpeed(0);
    delay(100000);
}



