/*
    This is the main code that will eventually incorporate all the different algorothims such as PID
    wall following and Wheel odometry to solve a maze and retreive a miner
    Maze Robot 

    Created 9/26/2021
    Christopher Hoverd

    https://github.com/ChrisHoverd/maze-robot/tree/develop
*/

//include motor driver library
#include "CytronMotorDriver.h" 
#include "Encoder.h"
#include "TimerOne.h"

// declare encoder input pins
// motor 1 and motor 2 are the left and right motors respectively, as seen from above with tripod wheel in the rear
const int left_motor_ch_A = 2;
const int left_motor_ch_B = 3;
const int right_motor_ch_A = 18;
const int right_motor_ch_B = 19;

//declare left ir sensor PID constants and error values
double left_ir_kp = 0.1;
double left_ir_kd = 0;
double left_ir_ki = 0;
double left_ir_desired_distance = 60; //keep robot 47mm from wall
volatile double left_ir_distance_error;
volatile double left_ir_last_distance_error;
volatile double left_ir_derivative_error;
volatile double left_ir_integral_error;
volatile double turn_rate;
volatile double forward_power;

//declare PID constants **uncomment if needed
// double front_ir_kp;
// double front_ir_kd;
// double front_ir_ki;
// double front_ir_desired_distance;
// double front_ir_distance_error;
// double front_ir_derivative_error;
// double front_ir_integral_error;

//declare IR sampling size and distance
int left_ir_pin = A0;
int front_ir_pin = A1;
float sensor_sample = 20;
float left_ir_distance;
volatile float front_ir_distance;

//declare mm/pulse constant
//converts encoder ticks to mm
const float thick_wheels_pulses_to_mm = 0.054048211; //thick wheels
const float skinny_wheels_pulses_to_mm = 0.06222775; // skinny wheels

//instantiate the encoder objects
Encoder left_motor_Enc(left_motor_ch_B, left_motor_ch_A); // swapped channels A & B so that both motors have positive readings when moving forward
Encoder right_motor_Enc(right_motor_ch_A, right_motor_ch_B);


//declare encoder counter variables for wheel odometry
volatile long left_motor_counter = 0;
volatile long right_motor_counter = 0;
volatile double left_motor_delta = 0;
volatile double right_motor_delta = 0;

volatile double orientation = 0;
volatile double delta_orientation = 0;
volatile double delta_distance = 0;
volatile double xPosition = 0;
volatile double yPosition = 0;
int wheelbase = 96;

//instantiate the motor objects
CytronMD leftMotor(PWM_DIR, 5, 4); // Motor 1 EN = Pin 5, DIR = 4
CytronMD rightMotor(PWM_DIR, 6, 7); // Motor 2 EN = Pin 6, Dir = 7


//declare motor PWM values, ie. -255 to 255
volatile int left_motor_speed = 0;
volatile int right_motor_speed = 0;

//time values for printing debugging values
float timenow = 0;
float time_prev = 0;
void setup() {

  //declare encoder pins as inputs
  pinMode (left_motor_ch_A, INPUT_PULLUP);
  pinMode (left_motor_ch_B, INPUT_PULLUP);
  pinMode (right_motor_ch_A, INPUT_PULLUP);
  pinMode (right_motor_ch_B, INPUT_PULLUP);
  
  //sets the reference voltage for the analog inputs, ie. 3.3V from the arduino for the IR sensors
  analogReference(EXTERNAL);

  //initializes a timer that is going to be used to have the program function
  //somewhat like a multithreaded program
  Timer1.initialize(100000); // 100 millisecond interrupt timer
  Timer1.attachInterrupt(timerInterrupt); //calls timerInterrupt every 100 milliseconds
  
  //sets the data rate in bits per second (baud)
  Serial.begin(9600);

}

void loop() 
{
//uncomment this section to print debugging values
//********************************

/*
timenow = millis();

if (timenow - time_prev > 1000)
{
 Serial.print("Left Motor Speed: ");
 Serial.println(left_motor_speed);
 Serial.print("Right Motor Speed: ");
 Serial.println(right_motor_speed);

 Serial.print("IR Distance");
 Serial.println(left_ir_distance);
 Serial.print("Analog Value");
 Serial.println(right_motor_speed);

  time_prev = timenow;
}
*/
//*****************************

}


//this interrupt takes place every 100ms to conduct various functions such as
//determine the distance from left wall and determine PID values based on that/
//the PID will determine a turn rate to keep the robot tracking straight
//eventually this interrupt will also include wheel odometry and coordinate point to
//point methods
void timerInterrupt()
{
  
  //retireve a left IR distance value
  left_ir_distance = readIRSensor(left_ir_pin);
  //front_ir_distance = readIRSensor(front_ir_pin);

  //calculate PID error values, updates the turn_rate variable
  wallFollowPID();

  //sets forward power to 70 in PWM
  forward_power = 70;

  //sets the right and left motor speeds according to the turn_rate calculated in the PID function
  right_motor_speed = forward_power - turn_rate;
  left_motor_speed = forward_power + turn_rate;



  //sets boundaries on how high and low the PWM values sent to the motors can be
  if(right_motor_speed>140)  right_motor_speed = 140;
  //if(right_motor_speed<-100) right_motor_speed = -100;
  if(right_motor_speed<60) right_motor_speed = 60;

  if(left_motor_speed>140)   left_motor_speed = 140;
  // if(left_motor_speed<-100)  left_motor_speed = -100;
  if(left_motor_speed<60)  left_motor_speed = 60;


  //sends the motor speeds to the right and left motors

  rightMotor.setSpeed(right_motor_speed);
  leftMotor.setSpeed(left_motor_speed);
 
}

//this function receives an ir pin value and returns a distance value
float readIRSensor(int ir_pin)
{

  //declare variables required for sampling and distance calculation
  float ir_val = 0;
  float ir_sum = 0;
  float ir_average_val = 0;
  float ir_distance = 0;

  for(int i = 0; i<sensor_sample; ++i)
  {

    //sums an average of analog sensor readings
    ir_val = analogRead(ir_pin);
    ir_sum+= ir_val;
  }
  //takes the summed value and divides it by the number of samples
  ir_average_val = ir_sum/sensor_sample;

  //sets a condition that when an analog value <500 (which is the range of values
  // we get when the IR sensor is less than 4cm from a wall which causes incorrect analog values)
  // to set the distance to 20 mm 
  if(ir_average_val<500)
  {
    ir_distance = 20;
  }

  //if the analog value is anything other than <500, do the normal analog to mm calculation
  else
  {
    ir_distance = (-0.0903*ir_average_val + 65.306)*10; //analog to mm function
  }

  //returns the ir distance
  return ir_distance;
}

//this function calculates the PID error values and from that
// it determines the turn_rate 
void wallFollowPID()
{

  //calculates the error value
  left_ir_distance_error = left_ir_desired_distance - left_ir_distance;

  //calculates the derivate error value
  left_ir_derivative_error = left_ir_distance_error - left_ir_last_distance_error;

  //sets the last distance error value to this derivative error value for the next iteration of the loop
  left_ir_last_distance_error = left_ir_derivative_error;

  //calculates the integral error value 
  left_ir_integral_error += left_ir_distance_error;

  //mulitplies each error value by its kp, kd, and ki gain to determine the turn_rate
  turn_rate = left_ir_kp*left_ir_distance_error + left_ir_kd*left_ir_derivative_error + left_ir_ki*left_ir_integral_error;
}