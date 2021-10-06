/*
PID test
*/

//include motor driver library
#include "CytronMotorDriver.h" 



//declare left ir sensor PID constants and error values
double left_ir_kp = 1;
double left_ir_kd = 0;
double left_ir_ki = 0;
double left_ir_desired_distance = 50; //keep robot 47mm from wall
volatile double left_ir_distance_error;
volatile double left_ir_last_distance_error;
volatile double left_ir_derivative_error;
volatile double left_ir_integral_error;
volatile double turn_rate;
volatile double forward_power;

//declare ir values
int left_ir_pin = A0;
int front_ir_pin = A1;
float sensor_sample = 3;
float left_ir_distance;


//instantiate the motor objects
CytronMD leftMotor(PWM_DIR, 5, 4); // Motor 1 EN = Pin 5, DIR = 4
CytronMD rightMotor(PWM_DIR, 6, 7); // Motor 2 EN = Pin 6, Dir = 7


//declare motor PWM values, ie. -255 to 255
volatile int left_motor_speed = 0;
volatile int right_motor_speed = 0;

float timenow = 0;
float time_prev = 0;
void setup() {

  
  //sets the reference voltage for the analog inputs, ie. 3.3V from the arduino for the IR sensors
analogReference(EXTERNAL);
  
  //sets the data rate in bits per second (baud)
Serial.begin(9600);

}

void loop() 
{
timenow = millis();

if (timenow - time_prev > 1000)
{
 Serial.print("Left Motor Speed: ");
 Serial.println(left_motor_speed);
 Serial.print("Right Motor Speed: ");
 Serial.println(right_motor_speed);

 Serial.print("IR Distance");
 Serial.println(left_ir_distance);
//  Serial.print("Analog Value");
//  Serial.println(right_motor_speed);

time_prev = timenow;
}
 
left_ir_distance = readIRSensor(left_ir_pin);
wallFollowPID();
forward_power = 80;
right_motor_speed = forward_power - turn_rate;
left_motor_speed = forward_power + turn_rate;

//works
if(right_motor_speed>100)  right_motor_speed = 100;
if(right_motor_speed<-100) right_motor_speed = -100;

if(left_motor_speed>100)   left_motor_speed = 100;
if(left_motor_speed<-100)  left_motor_speed = -100;



rightMotor.setSpeed(right_motor_speed);
leftMotor.setSpeed(left_motor_speed);

 // test 
  // if(right_motor_speed>100) right_motor_speed = 100;
  // if(right_motor_speed<-100) right_motor_speed = -100;
  // if(right_motor_speed>0 && right_motor_speed<=60)
  // {
  //   right_motor_speed = 60;
  // }

  // if(right_motor_speed<0 && right_motor_speed>-60)
  // {
  //   right_motor_speed = -60;
  // }

  // if(left_motor_speed>100) left_motor_speed = 100;
  // if(left_motor_speed<-100) left_motor_speed = -100;
  // if(left_motor_speed>0 && left_motor_speed<=60)
  // {
  //   if (left_motor_speed>0) left_motor_speed = 60;
  // }

  //   if(left_motor_speed<0 && left_motor_speed>-60)
  // {
  //   left_motor_speed = -60;
  // }
 
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
  if(ir_average_val<500)
  {
    ir_distance = 20;
  }

  else
  {
    ir_distance = (-0.0903*ir_average_val + 65.306)*10; //analog to mm function
  }
  
  return ir_distance;
}

void wallFollowPID()
{
  left_ir_distance_error = left_ir_desired_distance - left_ir_distance;
  left_ir_derivative_error = left_ir_distance_error - left_ir_last_distance_error;
  left_ir_last_distance_error = left_ir_derivative_error;
  left_ir_integral_error += left_ir_distance_error;
  turn_rate = left_ir_kp*left_ir_distance_error + left_ir_kd*left_ir_derivative_error + left_ir_ki*left_ir_integral_error;
}
