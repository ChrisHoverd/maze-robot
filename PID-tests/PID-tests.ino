/*
The main use of this code is to develop, test, and troubleshoot the wall following PID algorithim
*/

//include motor driver library
#include "CytronMotorDriver.h" 



//declare left ir sensor PID constants and error values
double kp = 2.2;
double kd = 300;
double ki = 0.00001;
double desired_distance = 45;
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
//declare IR sampling size and distance
int sensor_sample = 50;
float left_ir_distance;
float front_ir_distance;
float right_ir_distance;


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

void setup() {

  Serial.begin(9600);

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


delay(120);
left_ir_distance = readIRSensor(left_ir_pin);

//uncomment this section to plot values
//********************************
// Serial.print(desired_distance);
// Serial.print(" ");
// Serial.println(left_ir_distance);
//********************************


wallFollowPID();


forward_power = 50;
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
