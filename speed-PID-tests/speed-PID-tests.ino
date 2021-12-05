/*
The main use of this code is to develop, test, and troubleshoot the wall following PID SPEED algorithim.
It uses a 
*/

//include motor driver library
#include "CytronMotorDriver.h" 
#include "Encoder.h"

//encoder channel pins
const int left_motor_ch_A = 0;
const int left_motor_ch_B = 1;
const int right_motor_ch_A = 2;
const int right_motor_ch_B = 3;

const float pulses_to_mm = 0.03282036321;
volatile long last_rw_counter = 0;
volatile long rw_counter = 0;
volatile long rw_counter_change;
volatile long last_lw_counter = 0;
volatile long lw_counter = 0;
volatile long lw_counter_change;

//declare left ir sensor PID constants and error values

// kp = 2, kd = 75, ki = 0, most usable so far

// double kp = 2.2; 
// double kd = 1000;
// double ki = 0.00001;

// double kp = 1.5; // most usable
// double kd = 20;
// double ki = 0;

// double kp = 2.2; 
// double kd = 450;
// double ki = 0.00001;

// double kp = 2.2; 
// double kd = 100;
// double ki = 0.00001;

// double kp = 4; 
// double kd = 1;
// double ki = 0;

// double kp = 2;
// double kd = 0.5;
// double ki = 0;


double kp = 2.2; 
double kd = 1000;
double ki = 0.00001;

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
double time_change;

//declare left wheel speed PID variables
int lw_kp = 1;
int lw_kd = 0.00001;
int lw_ki = 0;
double lw_error;
double lw_derivative_error;
double last_lw_error;
double lw_integral_error;
double lw_pwm;

//declare right wheel speed PID variables
int rw_kp = 1;
int rw_kd = 0.00001;
int rw_ki = 0;
double rw_error;
double rw_derivative_error;
double last_rw_error;
double rw_integral_error;
double rw_pwm;

//declare ir values
int left_ir_pin = A0;
int right_ir_pin = A2;

//declare IR sampling size and distance
int sensor_sample = 50;
float left_ir_distance;
float right_ir_distance;


Encoder left_Enc(left_motor_ch_B, left_motor_ch_A); 
Encoder right_Enc(right_motor_ch_A, right_motor_ch_B);
CytronMD leftMotor(PWM_DIR, 6, 7); //EN = Pin 6, Dir = 7
CytronMD rightMotor(PWM_DIR, 5, 4); // EN = Pin 5, DIR = 4


//declare motor PWM values, ie. -255 to 255
double lw_speed;
double rw_speed;

double turn_speed;

//time values for left wheel speed PID
unsigned long lw_time_now;
unsigned long lw_time_prev = 0;
double lw_time_change;

//time values for left wheel speed PID
unsigned long rw_time_now;
unsigned long rw_time_prev = 0;
double rw_time_change;


//time values for printing debugging values
unsigned long debugging_time_now;
unsigned long debugging_time_prev = 0;
double debugging_time_change;

void setup() {
  pinMode (left_motor_ch_A, INPUT_PULLUP);
  pinMode (left_motor_ch_B, INPUT_PULLUP);
  pinMode (right_motor_ch_A, INPUT_PULLUP);
  pinMode (right_motor_ch_B, INPUT_PULLUP);
  Serial.begin(115200);

}

void loop() 
{
delay(40);
left_ir_distance = readIRSensor(left_ir_pin);
wallFollowPID();
//Serial.print("time change: ");
//Serial.print(time_change);
//Serial.print(" error ");
//Serial.print(error);
//Serial.print(" derivative error: ");
//Serial.println(derivative_error);

//sets boundaries on the PWM to speed conversion function within the domain of where it applies
if(turn_rate>255)  turn_rate = 255;
if(turn_rate<-255) turn_rate = -255;

pwmToSpeed();


forward_speed = 30;
rw_speed = forward_speed - turn_speed;
lw_speed = forward_speed + turn_speed;
rightWheelSpeedPID(rw_speed);
leftWheelSpeedPID(lw_speed);


//sets boundaries on how high and low the PWM values sent to the motors can be
if(rw_pwm>255)  rw_pwm = 255;
if(rw_pwm<-255) rw_pwm = -255;

if(lw_pwm>255)   lw_pwm = 255;
if(lw_pwm<-255)  lw_pwm = -255;


//sends the motor speeds to the right and left motors
 rightMotor.setSpeed(rw_pwm);
 leftMotor.setSpeed(lw_pwm);
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
  if (ir_distance<0) ir_distance = 0;
  return ir_distance;
}


//this function calculates the PID error values and from that
// it determines the turn_rate 
void wallFollowPID()
{
  time_now = millis();
  time_change = (float)(time_now - time_prev);
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
    //speed = 5.09 + (2*turn_rate) - (pow(4.56,-3) * pow(turn_rate,2));old speed function
    turn_speed = -133 + (6.42 * turn_rate) - (0.0492 * (pow(turn_rate,2))) + ((1.76 * pow(10, -4)) * (pow(turn_rate,3))) - (2.35*(pow(10,-7)) * (pow(turn_rate, 4)));
  }
  if(turn_rate<0)
  {
    //speed = -5.09 + (2*turn_rate) + (pow(4.56,-3) * pow(turn_rate,2)); old speed function
    turn_speed = 133 + (6.42 * turn_rate) + (0.0492 * pow(turn_rate,2)) + ((1.76 * pow(10, -4)) * (pow(turn_rate,3))) + (2.35*(pow(10,-7)) * (pow(turn_rate, 4)));
  }

}

void leftWheelSpeedPID(double desired_speed)
{
  double desired_lw_speed = desired_speed;
  lw_time_now = millis();
  lw_time_change = (float)(lw_time_now - lw_time_prev);
  lw_counter = left_Enc.read();
  lw_counter_change = lw_counter - last_lw_counter;
  last_lw_counter = lw_counter;
  double lw_distance = lw_counter_change*pulses_to_mm;
  double lw_speed = (lw_distance/lw_time_change)*1000;
  lw_error = desired_lw_speed - lw_speed;
  lw_derivative_error = (lw_error - last_lw_error)/lw_time_change;
  last_lw_error = lw_error;
  lw_integral_error += (lw_error*lw_time_change);
  lw_pwm = (lw_error*lw_kp) + (lw_derivative_error*lw_kd) + (lw_integral_error*ki);
}

void rightWheelSpeedPID(double desired_speed)
{ 
  double desired_rw_speed = desired_speed;
  rw_time_now = millis();
  rw_time_change = (float)(rw_time_now - rw_time_prev);
  rw_counter = right_Enc.read();
  rw_counter_change = rw_counter - last_rw_counter;
  last_rw_counter = rw_counter;
  double rw_distance = rw_counter_change*pulses_to_mm;
  double rw_speed = (rw_distance/rw_time_change)*1000;
  rw_error = desired_rw_speed - rw_speed;
  rw_derivative_error = (rw_error - last_rw_error)/rw_time_change;
  last_rw_error = rw_error;
  rw_integral_error += (rw_error*rw_time_change);
  rw_pwm = (rw_error*rw_kp) + (rw_derivative_error*rw_kd) + (rw_integral_error*rw_ki);
}
