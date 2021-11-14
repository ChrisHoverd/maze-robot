/*
  The main purpose of this code is to achieve pwm vs speed values
    change the pwm values sent to the motors in the setup loop
*/

#include "CytronMotorDriver.h" 
#include "Encoder.h"


const int left_motor_ch_A = 0;
const int left_motor_ch_B = 1;
const int right_motor_ch_A = 2;
const int right_motor_ch_B = 3;


const float pulses_to_mm = 0.03282036321;
volatile long last_right_motor_counter = 0;
volatile long right_motor_counter = 0;
volatile long right_motor_counter_change;


Encoder left_Enc(left_motor_ch_B, left_motor_ch_A); 
Encoder right_Enc(right_motor_ch_A, right_motor_ch_B);

//instantiate the motor objects
//new pinouts below
CytronMD leftMotor(PWM_DIR, 6, 7); //EN = Pin 6, Dir = 7
CytronMD rightMotor(PWM_DIR, 5, 4); // EN = Pin 5, DIR = 4


volatile int left_motor_speed = 0;
volatile int right_motor_speed = 0;

double distance;
double speed;

//time values for printing debugging values
float time_now = 0;
float time_prev = 0;
float time_change;

void setup() {
  pinMode (left_motor_ch_A, INPUT_PULLUP);
  pinMode (left_motor_ch_B, INPUT_PULLUP);
  pinMode (right_motor_ch_A, INPUT_PULLUP);
  pinMode (right_motor_ch_B, INPUT_PULLUP);
  Serial.begin(9600);
  last_right_motor_counter = left_Enc.read(); 
  delay(1000);
  //change these PWM values to compare PWM to speed
  leftMotor.setSpeed(255);
  rightMotor.setSpeed(255);
}

void loop() {

  time_now = millis();
  time_change = time_now - time_prev;
  if (time_change>=1000)
  {
    right_motor_counter = right_Enc.read();
    right_motor_counter_change = right_motor_counter - last_right_motor_counter;
    last_right_motor_counter = right_motor_counter;
    distance = right_motor_counter_change*pulses_to_mm;
    speed = (distance/time_change)*1000;
    Serial.print("Speed: ");
    Serial.print(speed);
    Serial.println(" mm/s");
    time_prev = time_now;
  }
  
}
