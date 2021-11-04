/*
The main use of this code is to integrate wheel odometry and the PID. PID testing is done in PID-tests
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
double left_ir_kp = 5;
double left_ir_kd = 0.1;
double left_ir_ki = 0;
double left_ir_desired_distance = 50; 
volatile double left_ir_distance_error;
volatile double left_ir_last_distance_error;
volatile double left_ir_derivative_error;
volatile double left_ir_integral_error;
volatile double turn_rate;
volatile double forward_power;

//declare right ir sensor PID constants and error values
double right_ir_kp = 5;
double right_ir_kd = 0.1;
double right_ir_ki = 0;
double right_ir_desired_distance = 50; 
volatile double right_ir_distance_error;
volatile double right_ir_last_distance_error;
volatile double right_ir_derivative_error;
volatile double right_ir_integral_error;




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
float timenow = 0;
float time_prev = 0;

float distance = 0;
void setup() {
  gripper.attach(9);
  pinMode (left_motor_ch_A, INPUT_PULLUP);
  pinMode (left_motor_ch_B, INPUT_PULLUP);
  pinMode (right_motor_ch_A, INPUT_PULLUP);
  pinMode (right_motor_ch_B, INPUT_PULLUP);
  openGripper();
  Serial.begin(9600);

}

void loop() 
{
  left_ir_distance = readIRSensor(left_ir_pin);
  front_ir_distance = readIRSensor(front_ir_pin);
  right_ir_distance = readIRSensor(right_ir_pin);
  leftwallFollowPID();

  if (left_ir_distance<60 && front_ir_distance<50 && right_ir_distance>120)
  {
    right_turn_counter++;
    rightTurn();


  }
  
  if (left_ir_distance<70 && front_ir_distance<50 && right_ir_distance<70)
  {
    about_turn_counter++;
    aboutTurn();
  }
  // if (left_ir_distance>120 && right_ir_distance<70)
  // {
  //   left_turn_counter++;
  //   leftTurn();
  // }
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
  return ir_distance;
}


//this function calculates the PID error values and from that
// it determines the turn_rate 
void leftwallFollowPID()
{

  left_ir_distance_error = left_ir_desired_distance - left_ir_distance;
  left_ir_derivative_error = left_ir_distance_error - left_ir_last_distance_error;
  left_ir_last_distance_error = left_ir_derivative_error;
  left_ir_integral_error += left_ir_distance_error;
  turn_rate = left_ir_kp*left_ir_distance_error + left_ir_kd*left_ir_derivative_error + left_ir_ki*left_ir_integral_error;
  
  left_ir_distance = readIRSensor(left_ir_pin);

  forward_power = 80;

  right_motor_speed = forward_power - turn_rate;
  left_motor_speed = forward_power + turn_rate;



  if(right_motor_speed>200)  right_motor_speed = 200;
  if(right_motor_speed<-200) right_motor_speed = -200;

  if(left_motor_speed>200)   left_motor_speed = 200;
  if(left_motor_speed<-200)  left_motor_speed = -200;


  rightMotor.setSpeed(right_motor_speed);
  leftMotor.setSpeed(left_motor_speed);

}

void rightwallFollowPID()
{

  right_ir_distance_error = right_ir_desired_distance - right_ir_distance;
  right_ir_derivative_error = right_ir_distance_error - right_ir_last_distance_error;
  right_ir_last_distance_error = right_ir_derivative_error;
  right_ir_integral_error += right_ir_distance_error;
  turn_rate = right_ir_kp*right_ir_distance_error + right_ir_kd*right_ir_derivative_error + right_ir_ki*right_ir_integral_error;

  forward_power = 80;

  right_motor_speed = forward_power + turn_rate;
  left_motor_speed = forward_power - turn_rate;



  if(right_motor_speed>200)  right_motor_speed = 200;
  if(right_motor_speed<-200) right_motor_speed = -200;

  if(left_motor_speed>200)   left_motor_speed = 200;
  if(left_motor_speed<-200)  left_motor_speed = -200;


  rightMotor.setSpeed(right_motor_speed);
  leftMotor.setSpeed(left_motor_speed);

}

void forward(double mm)
{ 

  left_motor_speed = 70;
  right_motor_speed = 70;
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
  delay(2000);
}


void leftTurn()
{ 

  left_motor_speed = -80;
  right_motor_speed = 80;
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

void openGripper()
{
  //100 is open
  gripper.write(100);
  delay(100);
}

void closeGripper()
{
  //205 is close
  gripper.write(205);
  delay(100);
}
