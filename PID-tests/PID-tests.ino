/*
The main use of this code is to develop, test, and troubleshoot the wall following PID algorithim
*/

//include motor driver library
#include "CytronMotorDriver.h" 
#include "Servo.h"


//declare left ir sensor PID constants and error values
// kp = 2.2, kd=1000, ki = 0.00001 stays in middle of lane better but goes around corners unstable
//kp = 2.2, kd=100, ki = 0.00001 turns well but isnt in middle of lane all the time
//kp = 4, kd = 1, ki = 0 middle ground of turning and staying in middle lane


 double kp = 5; 
 double kd = 500;
 double ki = 0;

//  double kp = 5; 
//  double kd = 300;
//  double ki = 0;

//  double kp = 2.2; 
//  double kd = 1000;
//  double ki = 0.00001;

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

Servo obstacleGripper;
Servo crane;

int craneBackPos = 0;
int craneUpPos = 55;
// int craneFrontPos = 145; // fully front
int craneFrontPos = 140;
int cranePartlyFrontPos = 100;

int gripperOpenPos = 150; // fully open
// int gripperOpenPos = 60;
int gripperClosePos = 0; // fully closed
// int gripperClosePos = 50;

int currentCranePos;
int currentGripperPos; 

int obstacleGripperDelay = 10; 
int slowObstacleGripperDelay = 40; 
int craneDelay = 40;

//declare motor PWM values, ie. -255 to 255
volatile int left_motor_speed;
volatile int right_motor_speed;

//time values for printing debugging values
unsigned long debugging_time_now;
unsigned long debugging_time_prev = 0;
unsigned long debugging_time_change;

void setup() {
  obstacleGripper.attach(8);
  crane.attach(9);
  Serial.begin(9600);

    initialCraneMoveUp();
  delay(1000);
  slowCloseObstacleGripper();
}

void loop() 
{

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


//uncomment this section to plot values
//********************************
// Serial.print(desired_distance);
// Serial.print(" ");
// Serial.println(left_ir_distance);
//********************************


wallFollowPID();


// forward_power = 75;
forward_power = 85;
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
  if (ir_distance>150) ir_distance = 150;
  return ir_distance;
}


//this function calculates the PID error values and from that
// it determines the turn_rate 
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

// int craneBack = 0   ,   int craneUp = 55 , int craneFront = 145
void craneMoveUp()
{
  currentCranePos = crane.read();
  if (currentCranePos >= craneUpPos)
  {
    for(int i = currentCranePos; i >= craneUpPos; i--)
    {
      crane.write(i);
      delay(craneDelay);
    }
  }
  else
  {
    for(int i = currentCranePos; i <= craneUpPos; i++)
    {
      crane.write(i);
      delay(craneDelay);
    }
  }
}

void initialCraneMoveUp()
{
  for(int i = craneFrontPos; i >= craneUpPos; i--)
  {
    crane.write(i);
    delay(craneDelay);
  }
}

void craneMoveFront()
{
  currentCranePos = crane.read();
  if (currentCranePos >= craneFrontPos)
  {
    for(int i = currentCranePos; i >= craneFrontPos; i--)
    {
      crane.write(i);
      delay(craneDelay);
    }
  }
  else
  {
    for(int i = currentCranePos; i <= craneFrontPos; i++)
    {
      crane.write(i);
      delay(craneDelay);
    }
  }
}

void craneMovePartlyFront()
{
  currentCranePos = crane.read();
  if (currentCranePos >= cranePartlyFrontPos)
  {
    for(int i = currentCranePos; i >= cranePartlyFrontPos; i--)
    {
      crane.write(i);
      delay(craneDelay);
    }
  }
  else
  {
    for(int i = currentCranePos; i <= cranePartlyFrontPos; i++)
    {
      crane.write(i);
      delay(craneDelay);
    }
  }
}


//int gripperOpenPos = 150     int gripperClosePos = 0

void openObstacleGripper()
{
  currentGripperPos = obstacleGripper.read();
  if (currentGripperPos >= gripperOpenPos)
  {
    for(int i = currentGripperPos; i >= gripperOpenPos; i--)
    {
      obstacleGripper.write(i);
      delay(obstacleGripperDelay);
    }
  }
  else
  {
    for(int i = currentGripperPos; i <= gripperOpenPos; i++)
    {
      obstacleGripper.write(i);
      delay(obstacleGripperDelay);
    }
  }
}

void closeObstacleGripper()
{ 
  currentGripperPos = obstacleGripper.read();
  if (currentGripperPos >= gripperClosePos)
  {
    for(int i = currentGripperPos; i >= gripperClosePos; i--)
    {
      obstacleGripper.write(i);
      delay(obstacleGripperDelay);
    }
  }
  else
  {
    for(int i = currentGripperPos; i <= gripperClosePos; i++)
    {
      obstacleGripper.write(i);
      delay(obstacleGripperDelay);
    }
  }
}

void slowCloseObstacleGripper()
{
  currentGripperPos = obstacleGripper.read();
  if (currentGripperPos >= gripperClosePos)
  {
    for(int i = currentGripperPos; i >= gripperClosePos; i--)
    {
      obstacleGripper.write(i);
      delay(slowObstacleGripperDelay);
    }
  }
  else
  {
    for(int i = currentGripperPos; i <= gripperClosePos; i++)
    {
      obstacleGripper.write(i);
      delay(slowObstacleGripperDelay);
    }
  }
}
