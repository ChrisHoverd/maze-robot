/*
The main use of this code is to integrate IR sensor logic and PID wall following. This program was used in the Phase B demonstration. PID testing is done in PID-tests
*/

//include motor driver library
#include "CytronMotorDriver.h" 
#include "Encoder.h"
#include "Servo.h"

Servo gripper;

const int left_motor_ch_A = 0;
const int left_motor_ch_B = 1;
const int right_motor_ch_A = 2;
const int right_motor_ch_B = 3;

//declare left ir sensor PID constants and error values
double kp = 2.2; 
double kd = 1000;
double ki = 0.00001;
double desired_distance = 45; //use for right angle IR position
// double desired_distance = 80; //use for 45 degree IR position
volatile double error;
volatile double last_error;
volatile double derivative_error;
volatile double integral_error;
volatile double turn_rate;
volatile double forward_power;

//declare right ir sensor PID constants and error values
// double right_ir_kp = 8;
// double right_ir_kd = 50;
// double right_ir_ki = 0;
// double right_ir_desired_distance = 50; 
// volatile double right_ir_distance_error;
// volatile double right_ir_last_distance_error;
// volatile double right_ir_derivative_error;
// volatile double right_ir_integral_error;




const float pulses_to_mm = 0.03282036321;
//declare encoder counter variables
volatile long left_motor_counter = 0;
volatile long right_motor_counter = 0;

//declare ir values
int left_ir_pin = A0;
int front_ir_pin = A1;
int right_ir_pin = A2;

//declare IR sampling size and distance
float sensor_sample = 50;
float left_ir_distance;
float front_ir_distance=60;
float right_ir_distance;

int left_turn_counter = 0;
int right_turn_counter = 0;
int about_turn_counter = 0;

Encoder left_Enc(left_motor_ch_B, left_motor_ch_A); // swapped channels A & B so that both motors have positive readings when moving forward
Encoder right_Enc(right_motor_ch_A, right_motor_ch_B);

//instantiate the motor objects
//new pinouts below
CytronMD leftMotor(PWM_DIR, 6, 7); //EN = Pin 6, Dir = 7
CytronMD rightMotor(PWM_DIR, 5, 4); // EN = Pin 5, DIR = 4


//declare motor PWM values, ie. -255 to 255
volatile int left_motor_speed = 0;
volatile int right_motor_speed = 0;

//time values for printing debugging values
float time_now = 0;
float time_prev = 0;
float time_change;

float distance = 0;
void setup() {
  gripper.attach(9);
  pinMode (left_motor_ch_A, INPUT_PULLUP);
  pinMode (left_motor_ch_B, INPUT_PULLUP);
  pinMode (right_motor_ch_A, INPUT_PULLUP);
  pinMode (right_motor_ch_B, INPUT_PULLUP);
  openGripper();
  Serial.begin(9600);
  delay(2000);

}

void loop() 
{

  left_ir_distance = readIRSensor(left_ir_pin);
  front_ir_distance = readIRSensor(front_ir_pin);
  right_ir_distance = readIRSensor(right_ir_pin);
  delay(40);
  // if (left_ir_distance>60)
  // {
  //   forward(65);
  //   leftTurn();
  //   forward(80);
  // }
  leftwallFollowPID();

}
//this function receives an ir pin value and returns a distance value
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
  
  //returns the ir distance
  if (ir_distance > 150) ir_distance = 150;
  return ir_distance;
}


//this function calculates the PID error values and from that
// it determines the turn_rate 
void leftwallFollowPID()
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
  
  //left_ir_distance = readIRSensor(left_ir_pin);

  forward_power = 80;

  right_motor_speed = forward_power - turn_rate;
  left_motor_speed = forward_power + turn_rate;



  if(right_motor_speed>255)  right_motor_speed = 255;
  if(right_motor_speed<-255) right_motor_speed = -255;

  if(left_motor_speed>255)   left_motor_speed = 255;
  if(left_motor_speed<-255)  left_motor_speed = -255;


  rightMotor.setSpeed(right_motor_speed);
  leftMotor.setSpeed(left_motor_speed);

}

//this function calculates the PID error values and from that
// it determines the turn_rate 
// void rightwallFollowPID()
// {
//   time_now = micros();
//   time_change = time_now - time_prev;
//   right_ir_distance_error = right_ir_desired_distance - right_ir_distance;
//   right_ir_derivative_error = (right_ir_last_distance_error - right_ir_distance_error)/time_change;
//   right_ir_last_distance_error = right_ir_derivative_error;
//   right_ir_integral_error += (right_ir_distance_error*time_change);
//   time_prev = time_now;
//   turn_rate = right_ir_kp*right_ir_distance_error + right_ir_kd*right_ir_derivative_error + right_ir_ki*right_ir_integral_error;

//   forward_power = 80;

//   right_motor_speed = forward_power + turn_rate;
//   left_motor_speed = forward_power - turn_rate;



//   if(right_motor_speed>255)  right_motor_speed = 255;
//   if(right_motor_speed<-255) right_motor_speed = -255;

//   if(left_motor_speed>255)   left_motor_speed = 255;
//   if(left_motor_speed<-255)  left_motor_speed = -255;


//   rightMotor.setSpeed(right_motor_speed);
//   leftMotor.setSpeed(left_motor_speed);

// }

//this function moves the robot forward a desired distance
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
      distance = abs((left_motor_counter*pulses_to_mm));
    }

  leftMotor.setSpeed(0);
  rightMotor.setSpeed(0);
  delay(1000);
}

//this function makes a left turn
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

//this function performs a special left turn where it rotates less than 90 degrees
void specialLeftTurn()
{ 

  left_motor_speed = -80;
  right_motor_speed = 80;
  float distance = 0;
  leftMotor.setSpeed(left_motor_speed);
  rightMotor.setSpeed(right_motor_speed);
  left_Enc.readAndReset();

  while(distance<45)
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

//this function makes a special right turn where it rotates more than 90 degrees
void specialRightTurn()
{ 

  left_motor_speed = 80;
  right_motor_speed = -80;
  float distance = 0;
  leftMotor.setSpeed(left_motor_speed);
  rightMotor.setSpeed(right_motor_speed);
  left_Enc.readAndReset();

  while(distance<75)
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

//this function opens the gripper
void openGripper()
{
  //100 is open
  gripper.write(100);
  delay(100);
}

//this function closes the gripper
void closeGripper()
{
  //205 is close
  gripper.write(230);
  delay(100);
}
