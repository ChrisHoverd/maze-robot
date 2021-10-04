/*
    Maze Robot 

    The robot solves a maze using wheel odometry and wall following

    Components:
    DC-DC Power Module 25W (DFR0205)                                               x 1
    Pololu 5V, 2.5A Step-Down Voltage Regulator                                    x 1
    0.8Amp 5V-26V DC Motor Driver Shield for Arduino (2 Channels) (SHIELD-2AMOTOR) x 1
    Micro DC Motor with Encoder (FIT0458)                                          x 2
    GOLDBAT 1500mAh 11.1V 100C 3S Lipo Battery                                     x 1

    Pin Connections: 


    Created 9/26/2021
    Christopher Hoverd

    https://github.com/ChrisHoverd/maze-robot
*/

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

//declare left ir sensor PID constants and error values
double left_ir_kp;
double left_ir_kd;
double left_ir_ki;
double left_ir_desired_distance = 47; //keep robot 47mm from wall
double left_ir_distance_error;
double left_ir_last_distance_error;
double left_ir_derivative_error;
double left_ir_integral_error;
double turn_rate;
double forward_power;

//declare PID constants **uncomment if needed
// double front_ir_kp;
// double front_ir_kd;
// double front_ir_ki;
// double front_ir_desired_distance;
// double front_ir_distance_error;
// double front_ir_derivative_error;
// double front_ir_integral_error;

//declare ir values
int left_ir_pin = A0;
int front_ir_pin = A1;
float sensor_sample = 10;
float left_ir_distance;
float front_ir_distance;

//declare mm/pulse constant
//converts encoder ticks to mm
const float pulses_to_mm = 0.054048211;

//instantiate the encoder objects
Encoder left_motor_Enc(left_motor_ch_B, left_motor_ch_A); // swapped channels A & B so that both motors have positive readings when moving forward
Encoder right_motor_Enc(right_motor_ch_A, right_motor_ch_B);


//declare encoder counter variables
volatile long left_motor_counter = 0;
volatile long right_motor_counter = 0;
volatile double left_motor_delta = 0;
volatile double right_motor_delta = 0;

volatile double orientation = 0;
volatile double delta_orientation = 0;
volatile double delta_distance = 0;
volatile double xPosition = 0;
volatile double yPosition = 0;


float distance;

//instantiate the motor objects
CytronMD leftMotor(PWM_DIR, 5, 4); // Motor 1 EN = Pin 5, DIR = 4
CytronMD rightMotor(PWM_DIR, 6, 7); // Motor 2 EN = Pin 6, Dir = 7


//declare motor PWM values, ie. -255 to 255
int left_motor_speed;
int right_motor_speed;

//declare wheel odometry variables
int wheelbase = 96;

void setup() {

  //declare encoder pins as inputs
  pinMode (left_motor_ch_A, INPUT_PULLUP);
  pinMode (left_motor_ch_B, INPUT_PULLUP);
  pinMode (right_motor_ch_A, INPUT_PULLUP);
  pinMode (right_motor_ch_B, INPUT_PULLUP);
  
  //sets the reference voltage for the analog inputs, ie. 3.3V from the arduino for the IR sensors
  analogReference(EXTERNAL);

  Timer3.initialize(50000); // 50 millisecond interrupt timer
  Timer3.attachInterrupt(timerInterrupt); //calls timerInterrupt every 50 milliseconds
  
  //sets the data rate in bits per second (baud)
  Serial.begin(9600);

}

void loop() 
{

    

}

void timerInterrupt()
{
  left_ir_distance = readIRSensor(left_ir_pin);
  front_ir_distance = readIRSensor(front_ir_pin);
  
  wallFollowPID();

  right_motor_speed = forward_power + turn_rate;
  left_motor_speed = forward_power - turn_rate;

  if(right_motor_speed>65)
  {
    right_motor_speed = 65;
  }

    if(right_motor_speed<-65)
  {
    right_motor_speed = -65;
  }

  if(left_motor_speed>65)
  {
    left_motor_speed = 65;
  }

    if(left_motor_speed<-65)
  {
    left_motor_speed = -65;
  }

  rightMotor.setSpeed(right_motor_speed);
  leftMotor.setSpeed(left_motor_speed);

}

float readIRSensor(int ir_pin)
{
  float ir_sum;
  float ir_average_val;
  float ir_distance;
  for(int i = 0; i<sensor_sample; ++i)
  {
    ir_sum+= analogRead(ir_pin);
  }

  ir_average_val = ir_sum/sensor_sample;
  ir_distance = (-0.0903*ir_average_val + 65.306)*10; //analog to mm function
  return ir_distance;
}

void wallFollowPID()
{
  left_ir_distance_error = left_ir_desired_distance - left_ir_distance;
  left_ir_derivative_error = left_ir_distance_error - left_ir_last_distance_error;
  left_ir_last_distance_error = left_ir_derivative_error;
  left_ir_integral_error += left_ir_distance_error;
  turn_rate = left_ir_kp*left_ir_distance_error + left_ir_kd*left_ir_derivative_error + left_ir_ki*left_ir_integral_error;
}