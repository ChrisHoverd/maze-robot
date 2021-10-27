/*
The main use of this code is to develop, test, and troubleshoot the wall following PID algorithim
*/

//include motor driver library
#include "CytronMotorDriver.h" 



//declare left ir sensor PID constants and error values
double left_ir_kp = 5;
double left_ir_kd = 0;
double left_ir_ki = 0.001;
double left_ir_desired_distance = 30; //keep robot 60mm from wall
volatile double left_ir_distance_error;
volatile double left_ir_last_distance_error;
volatile double left_ir_derivative_error;
volatile double left_ir_integral_error;
volatile double turn_rate;
volatile double forward_power;

//declare ir values
int left_ir_pin = A0;
int front_ir_pin = A1;

//declare IR sampling size and distance
float sensor_sample = 50;
float left_ir_distance;


//instantiate the motor objects
// CytronMD leftMotor(PWM_DIR, 5, 4); // EN = Pin 5, DIR = 4
// CytronMD rightMotor(PWM_DIR, 6, 7); //EN = Pin 6, Dir = 7
//new pinouts below
CytronMD leftMotor(PWM_DIR, 6, 7); //EN = Pin 6, Dir = 7
CytronMD rightMotor(PWM_DIR, 5, 4); // EN = Pin 5, DIR = 4


//declare motor PWM values, ie. -255 to 255
volatile int left_motor_speed = 0;
volatile int right_motor_speed = 0;

//time values for printing debugging values
float timenow = 0;
float time_prev = 0;

void setup() {

  
//sets the reference voltage for the analog inputs, ie. 3.3V from the arduino for the IR sensors
// analogReference(EXTERNAL);
  
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

//********************************

//retireve a left IR distance value
left_ir_distance = readIRSensor(left_ir_pin);

//calculate PID error values, updates the turn_rate variable
wallFollowPID();

//sets forward power to 70 in PWM
forward_power = 80;

//sets the right and left motor speeds according to the turn_rate calculated in the PID function
right_motor_speed = forward_power - turn_rate;
left_motor_speed = forward_power + turn_rate;


//sets boundaries on how high and low the PWM values sent to the motors can be
if(right_motor_speed>200)  right_motor_speed = 200;
if(right_motor_speed<-200) right_motor_speed = -200;

if(left_motor_speed>200)   left_motor_speed = 200;
if(left_motor_speed<-200)  left_motor_speed = -200;


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
  // if(ir_average_val<500)
  // {
  //   ir_distance = 20;
  // }

  //if the analog value is anything other than <500, do the normal analog to mm calculation
  // else
  // {
  //   ir_distance = (-0.0903*ir_average_val + 65.306)*10; //analog to mm function
  // }

  ir_distance = (478.49*(pow(ir_average_val,-0.866)))*10; //analog to mm function
  
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
