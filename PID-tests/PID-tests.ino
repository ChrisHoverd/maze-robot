/*
The main use of this code is to develop, test, and troubleshoot the wall following PID algorithim
*/

//include motor driver library
#include "CytronMotorDriver.h" 



//declare left ir sensor PID constants and error values
double kp = 5;
double kd = 0.001;
double ki = 0.001;
double desired_distance = 45;
volatile double error;
volatile double last_error;
volatile double derivative_error;
volatile double integral_error;
volatile double turn_rate;
volatile double forward_power;

//declare ir values
int left_ir_pin = A0;
int front_ir_pin = A1;
int right_ir_pin = A2;
//declare IR sampling size and distance
float sensor_sample = 50;
float left_ir_distance;
float front_ir_distance;
float right_ir_distance;


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

void setup() {

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


left_ir_distance = readIRSensor(left_ir_pin);
wallFollowPID();


forward_power = 80;
right_motor_speed = forward_power - turn_rate;
left_motor_speed = forward_power + turn_rate;


//sets boundaries on how high and low the PWM values sent to the motors can be
if(right_motor_speed>255)  right_motor_speed = 255;
if(right_motor_speed<-255) right_motor_speed = -255;

if(left_motor_speed>255)   left_motor_speed = 255;
if(left_motor_speed<-255)  left_motor_speed = -255;


//sends the motor speeds to the right and left motors
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
  
  return ir_distance;
}


//this function calculates the PID error values and from that
// it determines the turn_rate 
void wallFollowPID()
{
  time_now = micros();
  time_change = time_now - time_prev;
  error = desired_distance - left_ir_distance;
  derivative_error = (error - last_error)/time_change;
  last_error = derivative_error;
  integral_error += (error*time_change);
  turn_rate = kp*error + kd*derivative_error + ki*integral_error;
  time_prev = time_now;
}
