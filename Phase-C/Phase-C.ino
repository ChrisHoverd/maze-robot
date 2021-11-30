

/*
This code will be used in phase C demonstration
*/


#include "CytronMotorDriver.h" 
#include "VarSpeedServo.h"
#include "Encoder.h"

const int left_motor_ch_A = 0;
const int left_motor_ch_B = 1;
const int right_motor_ch_A = 2;
const int right_motor_ch_B = 3;

const float pulses_to_mm = 0.03282036321;
//declare encoder counter variables
volatile long left_motor_counter = 0;
volatile long right_motor_counter = 0;

Encoder left_Enc(left_motor_ch_B, left_motor_ch_A); // swapped channels A & B so that both motors have positive readings when moving forward
Encoder right_Enc(right_motor_ch_A, right_motor_ch_B);

//declare left ir sensor PID constants and error values
// kp = 2.2, kd=1000, ki = 0.00001 stays in middle of lane better but goes around corners unstable
//kp = 2.2, kd=100, ki = 0.00001 turns well but isnt in middle of lane all the time
//kp = 4, kd = 1, ki = 0 middle ground of turning and staying in middle lane

 double kp = 2.2; 
 double kd = 1000;
 double ki = 0.00001;

// double kp = 2.2; 
// double kd = 100;
// double ki = 0.00001;

// double kp = 2.2; //pretty solid
// double kd = 300;
// double ki = 0.00001;

// double kp = 2.2; //pretty solid
// double kd = 220;
// double ki = 0.00001;

// double kp = 2.2; //pretty solid
// double kd = 150;
// double ki = 0.00001;

//double kp = 3; //pretty solid best so far!!
//double kd = 150;
//double ki = 0.00001;

// double kp = 4; 
// double kd = 1;
// double ki = 0;

double desired_distance = 45; //use for right angle IR position
// double desired_distance = 80; //use for 45 degree IR position
volatile double error;
volatile double last_error;
volatile double derivative_error;
volatile double integral_error;
volatile double turn_rate;
volatile double forward_power;

//time values for PID
double time_now;
double time_prev = 0;
double time_change;

//declare ir values
int left_ir_pin = A0;
int front_ir_pin = A1;
int right_ir_pin = A2;
int back_left_ir_pin = A3;

//declare IR sampling size and distance
int sensor_sample = 50;
float left_ir_distance;
float front_ir_distance;
float right_ir_distance;
float back_left_ir_distance;

//instantiate the motor objects
//new pinouts below
CytronMD leftMotor(PWM_DIR, 6, 7); //EN = Pin 6, Dir = 7
CytronMD rightMotor(PWM_DIR, 5, 4); // EN = Pin 5, DIR = 4

//declare motor PWM values, ie. -255 to 255
volatile int left_motor_speed;
volatile int right_motor_speed;

//time values for printing debugging values
unsigned long debugging_time_now;
unsigned long debugging_time_prev = 0;
unsigned long debugging_time_change;

int obstacleGripperSpeed = 50; 
int craneSpeed = 10;

VarSpeedServo obstacleGripper;
VarSpeedServo crane;

int obstacleGripperPos = 0;
int cranePos = 0;




void setup() {
  obstacleGripper.attach(8);
  crane.attach(10);
  pinMode (left_motor_ch_A, INPUT_PULLUP);
  pinMode (left_motor_ch_B, INPUT_PULLUP);
  pinMode (right_motor_ch_A, INPUT_PULLUP);
  pinMode (right_motor_ch_B, INPUT_PULLUP);
  Serial.begin(9600);

// for crane, 0 is the 0 degrees position, 55 is 90 degree position, 145 is 180 degrees position (WITHOUT ADDED WEIGHT OF GRIPPER)
// for gripper, 0 is fully closed grip and 150 is fully open grip.
  obstacleGripper.write(0, obstacleGripperSpeed, true); //close gripper
  crane.write(55, craneSpeed, true); //90 degree crane

}

void loop() {

//uncomment this section to print debugging values
//********************************

/*
debugging_time_now = millis();

if (debugging_time_now - debugging_time_prev >= 1000)
{
//  Serial.print("Distance ");
//  Serial.print(left_ir_distance);
//  Serial.println(" mm");
 Serial.println(turn_rate);
 debugging_time_prev = debugging_time_now;
}
*/

//********************************
  delay(40);
  left_ir_distance = readIRSensor(left_ir_pin);
  front_ir_distance = readIRSensor(front_ir_pin);
  right_ir_distance = readIRSensor(right_ir_pin);
  wallFollowPID();

  forward_power = 75;
  // forward_power = 60;
  right_motor_speed = forward_power - turn_rate;
  left_motor_speed = forward_power + turn_rate;
  
  
  //sets boundaries on how high and low the PWM values sent to the motors can be
  if(right_motor_speed>255)  right_motor_speed = 255;
  if(right_motor_speed<-255) right_motor_speed = -255;
  
  
  if(left_motor_speed>255)   left_motor_speed = 255;
  if(left_motor_speed<-255)  left_motor_speed = -255;
  
  
  
  // //sends the motor speeds to the right and left motors
  rightMotor.setSpeed(right_motor_speed);
  leftMotor.setSpeed(left_motor_speed);

}

void wallFollowPID()
{
  time_now = micros();
  time_change = (float)(time_now - time_prev)/1000;
  error = desired_distance - left_ir_distance;
  derivative_error = (error - last_error)/time_change;
  last_error = error;
  integral_error += (error*time_change);
  turn_rate = (kp*error) + (kd*derivative_error) + (ki*integral_error);
  //Serial.println(derivative_error);
  time_prev = time_now;
}

float readIRSensor(int ir_pin)
{

  float ir_val = 0;
  float ir_sum = 0;
  float ir_average_val = 0;
  float ir_distance = 0;

  for(int i = 0; i<sensor_sample; ++i)
  {
    ir_val = analogRead(ir_pin);
    ir_sum+= ir_val;
  }

  ir_average_val = ir_sum/sensor_sample;
  ir_distance = (478.49*(pow(ir_average_val,-0.866)))*10; //analog to mm function
  if (ir_distance>150) ir_distance = 150;
  return ir_distance;
}

void leftTurn()
{ 

  left_motor_speed = -80;
  right_motor_speed = 80;
  float distance = 0;
  leftMotor.setSpeed(left_motor_speed);
  rightMotor.setSpeed(right_motor_speed);
  left_Enc.readAndReset();

  while(distance<55)
    {
      left_motor_counter = left_Enc.read();
      distance = abs((left_motor_counter*pulses_to_mm));
    }

  leftMotor.setSpeed(0);
  rightMotor.setSpeed(0);
  delay(1000);
}

//this function makes a right turn
void rightTurn()
{ 

  left_motor_speed = 80;
  right_motor_speed = -80;
  float distance = 0;
  leftMotor.setSpeed(left_motor_speed);
  rightMotor.setSpeed(right_motor_speed);
  left_Enc.readAndReset();

  while(distance<68)
    {
      left_motor_counter = left_Enc.read();


      distance = abs((left_motor_counter*pulses_to_mm));
    }

  leftMotor.setSpeed(0);
  rightMotor.setSpeed(0);
  delay(1000);
}

//this function makes a 180 degree turn
void aboutTurn()
{ 

  left_motor_speed = 80;
  right_motor_speed = -80;
  float distance = 0;
  leftMotor.setSpeed(left_motor_speed);
  rightMotor.setSpeed(right_motor_speed);
  left_Enc.readAndReset();

  while(distance<136)
    {
      left_motor_counter = left_Enc.read();


      distance = abs((left_motor_counter*pulses_to_mm));
    }

  leftMotor.setSpeed(0);
  rightMotor.setSpeed(0);
  delay(1000);
}
