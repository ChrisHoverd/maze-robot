/*
The main use of this code is to develop, test, and troubleshoot the wall following PID SPEED algorithim
*/

//include motor driver library
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
volatile long last_left_motor_counter = 0;
volatile long left_motor_counter = 0;
volatile long left_motor_counter_change;

//declare left ir sensor distance PID constants and error values
double kp = 5;
double kd = 0;
double ki = 0;
double desired_distance = 45;
volatile double error;
volatile double last_error;
volatile double derivative_error;
volatile double integral_error;
volatile double turn_rate;
volatile double forward_speed;

//time values for Distance PID
unsigned long time_now;
unsigned long time_prev = 0;
unsigned long time_change;

//declare left wheel speed PID variables
int left_wheel_kp = 1;
int left_wheel_kd = 0;
int left_wheel_ki = 0;
double left_wheel_speed_error;
double left_wheel_speed_derivative_error;
double last_left_wheel_speed_error;
double left_wheel_speed_integral_error;
double left_wheel_pwm;

//declare right wheel speed PID variables
int right_wheel_kp = 1;
int right_wheel_kd = 0;
int right_wheel_ki = 0;
double right_wheel_speed_error;
double right_wheel_speed_derivative_error;
double last_right_wheel_speed_error;
double right_wheel_speed_integral_error;
double right_wheel_pwm;

//declare ir values
int left_ir_pin = A0;
int right_ir_pin = A2;

//declare IR sampling size and distance
float sensor_sample = 50;
float left_ir_distance;
float right_ir_distance;


Encoder left_Enc(left_motor_ch_B, left_motor_ch_A); 
Encoder right_Enc(right_motor_ch_A, right_motor_ch_B);
CytronMD leftMotor(PWM_DIR, 6, 7); //EN = Pin 6, Dir = 7
CytronMD rightMotor(PWM_DIR, 5, 4); // EN = Pin 5, DIR = 4


//declare motor PWM values, ie. -255 to 255
volatile int left_motor_speed;
volatile int right_motor_speed;

double speed;

//time values for left wheel speed PID
unsigned long left_wheel_time_now;
unsigned long left_wheel_time_prev = 0;
unsigned long left_wheel_time_change;

//time values for left wheel speed PID
unsigned long right_wheel_time_now;
unsigned long right_wheel_time_prev = 0;
unsigned long right_wheel_time_change;


//time values for printing debugging values
unsigned long debugging_time_now;
unsigned long debugging_time_prev = 0;
unsigned long debugging_time_change;

void setup() {
  pinMode (left_motor_ch_A, INPUT_PULLUP);
  pinMode (left_motor_ch_B, INPUT_PULLUP);
  pinMode (right_motor_ch_A, INPUT_PULLUP);
  pinMode (right_motor_ch_B, INPUT_PULLUP);
  Serial.begin(9600);

}

void loop() 
{

left_ir_distance = readIRSensor(left_ir_pin);
wallFollowPID();

if(turn_rate>255)  turn_rate = 255;
if(turn_rate<-255) turn_rate = -255;

pwmToSpeed();


forward_speed = 45;
right_motor_speed = forward_speed - speed;
left_motor_speed = forward_speed + speed;
rightWheelSpeedPID(right_motor_speed);
leftWheelSpeedPID(left_motor_speed);


//sets boundaries on how high and low the PWM values sent to the motors can be
if(right_wheel_pwm>255)  right_wheel_pwm = 255;
if(right_wheel_pwm<0) right_wheel_pwm = 0;

if(left_wheel_pwm>255)   left_wheel_pwm = 255;
if(left_wheel_pwm<0)  left_wheel_pwm = 0;


// //sends the motor speeds to the right and left motors
rightMotor.setSpeed(right_wheel_pwm);
leftMotor.setSpeed(left_wheel_pwm);
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
  time_now = millis();
  time_change = time_now - time_prev;
  error = desired_distance - left_ir_distance;
  derivative_error = (error - last_error)/time_change;
  last_error = error;
  integral_error += (error*time_change);
  turn_rate = (kp*error) + (kd*derivative_error) + (ki*integral_error);
  //Serial.println(derivative_error);
  time_prev = time_now;
}

void pwmToSpeed()
{
  if(turn_rate>0)
  {
    speed = 5.09 + (2*turn_rate) - (pow(4.56,-3) * pow(turn_rate,2));
  }
  if(turn_rate<0)
  {
    speed = -5.09 + (2*turn_rate) + (pow(4.56,-3) * pow(turn_rate,2));
  }

}

void leftWheelSpeedPID(double desired_speed)
{
  double desired_left_wheel_speed = desired_speed;
  left_wheel_time_now = millis();
  left_wheel_time_change = left_wheel_time_now - left_wheel_time_prev;
  left_motor_counter = left_Enc.read();
  left_motor_counter_change = left_motor_counter - last_left_motor_counter;
  last_left_motor_counter = left_motor_counter;
  double left_wheel_distance = left_motor_counter_change*pulses_to_mm;
  double left_wheel_speed = (left_wheel_distance/left_wheel_time_change)*1000;
  left_wheel_speed_error = desired_left_wheel_speed - left_wheel_speed;
  left_wheel_speed_derivative_error = (left_wheel_speed_error - last_left_wheel_speed_error)/left_wheel_time_change;
  last_left_wheel_speed_error = left_wheel_speed_error;
  left_wheel_speed_integral_error += (left_wheel_speed_error*left_wheel_time_change);
  left_wheel_pwm = (left_wheel_speed_error*left_wheel_kp) + (left_wheel_speed_derivative_error*left_wheel_kd) + (left_wheel_speed_integral_error*ki);
}

void rightWheelSpeedPID(double desired_speed)
{ 
  double desired_right_wheel_speed = desired_speed;
  right_wheel_time_now = millis();
  right_wheel_time_change = right_wheel_time_now - right_wheel_time_prev;
  right_motor_counter = right_Enc.read();
  right_motor_counter_change = right_motor_counter - last_right_motor_counter;
  last_right_motor_counter = right_motor_counter;
  double right_wheel_distance = right_motor_counter_change*pulses_to_mm;
  double right_wheel_speed = (right_wheel_distance/right_wheel_time_change)*1000;
  right_wheel_speed_error = desired_right_wheel_speed - right_wheel_speed;
  right_wheel_speed_derivative_error = (right_wheel_speed_error - last_right_wheel_speed_error)/right_wheel_time_change;
  last_right_wheel_speed_error = right_wheel_speed_error;
  right_wheel_speed_integral_error += (right_wheel_speed_error*right_wheel_time_change);
  right_wheel_pwm = (right_wheel_speed_error*right_wheel_kp) + (right_wheel_speed_derivative_error*right_wheel_kd) + (right_wheel_speed_integral_error*right_wheel_ki);
}