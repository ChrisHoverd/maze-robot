

/*
This code will be used in phase C demonstration
*/


#include "CytronMotorDriver.h" 
#include "Encoder.h"
#include "Servo.h"

const int left_motor_ch_A = 0;
const int left_motor_ch_B = 1;
const int right_motor_ch_A = 2;
const int right_motor_ch_B = 3;

const float pulses_to_mm = 0.03282036321;
volatile long left_motor_counter = 0;
volatile long right_motor_counter = 0;

Servo obstacleGripper;
Servo crane;

Encoder left_Enc(left_motor_ch_B, left_motor_ch_A); // swapped channels A & B so that both motors have positive readings when moving forward
Encoder right_Enc(right_motor_ch_A, right_motor_ch_B);

CytronMD leftMotor(PWM_DIR, 6, 7); //EN = Pin 6, Dir = 7
CytronMD rightMotor(PWM_DIR, 5, 4); // EN = Pin 5, DIR = 4

//declare left ir sensor PID constants and error values
// kp = 2.2, kd=1000, ki = 0.00001 stays in middle of lane better but goes around corners unstable
//kp = 2.2, kd=100, ki = 0.00001 turns well but isnt in middle of lane all the time
//kp = 4, kd = 1, ki = 0 middle ground of turning and staying in middle lane



 double kp = 5; //best with new weight
 double kd = 500;
 double ki = 0;
 
// double kp = 2.2; //best so far with crane
// double kd = 1000;
// double ki = 0.00001;

// double kp = 2.2; 
// double kd = 100;
// double ki = 0.00001;

// double kp = 2.2; //pretty solid
// double kd = 500;
// double ki = 0.00001;

//  double kp = 2.2; //pretty solid
//  double kd = 300;
//  double ki = 0.00001;

// double kp = 2.2; //pretty solid
// double kd = 220;
// double ki = 0.00001;

// double kp = 2.2; //pretty solid
// double kd = 150;
// double ki = 0.00001;
//
//double kp = 3; //pretty solid best so far!!
//double kd = 150;
//double ki = 0.00001;

// double kp = 4; 
// double kd = 1;
// double ki = 0;


volatile double turn_rate;
volatile double forward_power;

// left wall following PID variables
double L_desired_distance = 45; //use for right angle IR position
volatile double L_error;
volatile double L_last_error;
volatile double L_derivative_error;
volatile double L_integral_error;

//time values for left PID
double L_time_now;
double L_time_prev = 0;
double L_time_change;

double R_desired_distance = 45; //use for right angle IR position
volatile double R_error;
volatile double R_last_error;
volatile double R_derivative_error;
volatile double R_integral_error;


//time values for left PID
double R_time_now;
double R_time_prev = 0;
double R_time_change;

//declare ir values
int left_ir_pin = A0;
int front_ir_pin = A1;
int right_ir_pin = A2;
int lower_front_ir_pin = A3;



//declare IR sampling size and distance
int sensor_sample = 50;
float left_ir_distance;
float front_ir_distance;
float right_ir_distance;
float lower_front_ir_distance;

//declare motor PWM values, ie. -255 to 255
volatile int left_motor_speed;
volatile int right_motor_speed;

int left_turn_counter = 0;
int right_turn_counter = 0;
int obstacle_counter = 0;

//time values for printing debugging values
unsigned long debugging_time_now;
unsigned long debugging_time_prev = 0;
unsigned long debugging_time_change;

int obstacleGripperDelay = 10; 
int slowObstacleGripperDelay = 40; 
int craneDelay = 40;

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

void setup() {
  obstacleGripper.attach(8);
  crane.attach(9);
  pinMode (left_motor_ch_A, INPUT_PULLUP);
  pinMode (left_motor_ch_B, INPUT_PULLUP);
  pinMode (right_motor_ch_A, INPUT_PULLUP);
  pinMode (right_motor_ch_B, INPUT_PULLUP);
  Serial.begin(9600);

  delay(2000);
// for crane, 0 is the 0 degrees position, 55 is 90 degree position, 145 is 180 degrees position (WITHOUT ADDED WEIGHT OF GRIPPER)
// for gripper, 0 is fully closed grip and 150 is fully open grip.
  
  initialCraneMoveUp();
  delay(1000);
  slowCloseObstacleGripper();
  delay(1000);

}

void loop() {

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
  // delay(40);
   left_ir_distance = readIRSensor(left_ir_pin);
   front_ir_distance = readIRSensor(front_ir_pin);
   right_ir_distance = readIRSensor(right_ir_pin);
   lower_front_ir_distance = readIRSensor(lower_front_ir_pin);

  // //normal right turn conditions
  // if (left_ir_distance<60 && front_ir_distance<50 && right_ir_distance>120 && right_turn_counter >= 1)
  // {
  //   right_turn_counter++;
  //   rightTurn();
  // }

  // //special left turn for left turn counter = 3    PID change
  // if (right_ir_distance<60 && front_ir_distance<50 && left_ir_distance>120 && left_turn_counter == 3)
  // {
  //   left_turn_counter++;
  //   specialLeftTurn();
  //   forward(120);
  // }

  //   //left turn conditions
  // if (right_ir_distance<60 && front_ir_distance<50 && left_ir_distance>120 && left_turn_counter <=2)
  // {
  //   left_turn_counter++;
  //   leftTurn();
  // }


  // //right wall follow conditions
  // if(left_turn_counter<4)
  // {
  //   rightWallFollowPID();
  // }

  // //left wall follow conditions
  // if(left_turn_counter>=4)
  // {
  //   leftWallFollowPID();
  // }

   rightWallFollowPID();
  //normal obstacle pickup
  if (front_ir_distance>50 && right_ir_distance <80 && lower_front_ir_distance <75/*&& lower_front_ir_distance <100 old distance*/)
  {
   if (obstacle_counter < 2)
   {
    obstaclePickup();
   }

  
  
  }

}

void obstaclePickup(){
   rightMotor.setSpeed(0);
   leftMotor.setSpeed(0);
   craneMoveFront();
   craneMoveUp();
   shallowLeftTurn();
   craneMovePartlyFront();
   openObstacleGripper();
   delay(500);
   closeObstacleGripper();
   craneMoveUp();
   delay(500);
   shallowRightTurn();
   delay(500);
   obstacle_counter++;
}
void leftWallFollowPID()
{
  L_time_now = micros();
  L_time_change = (float)(L_time_now - L_time_prev)/1000;
  L_error = L_desired_distance - left_ir_distance;
  L_derivative_error = (L_error - L_last_error)/L_time_change;
  L_last_error = L_error;
  L_integral_error += (L_error*L_time_change);
  turn_rate = (kp*L_error) + (kd*L_derivative_error) + (ki*L_integral_error);
  //Serial.println(derivative_error);
  L_time_prev = L_time_now;

  forward_power = 85;
   right_motor_speed = forward_power - turn_rate;
   left_motor_speed = forward_power + turn_rate;
  
  // //sets boundaries on how high and low the PWM values sent to the motors can be
   if(right_motor_speed>255)  right_motor_speed = 255;
   if(right_motor_speed<-255) right_motor_speed = -255;
  
  
   if(left_motor_speed>255)   left_motor_speed = 255;
   if(left_motor_speed<-255)  left_motor_speed = -255;

   // //sends the motor speeds to the right and left motors
   rightMotor.setSpeed(right_motor_speed);
   leftMotor.setSpeed(left_motor_speed);
}


void rightWallFollowPID()
{
  R_time_now = micros();
  R_time_change = (float)(R_time_now - R_time_prev)/1000;
  R_error = R_desired_distance - right_ir_distance;
  R_derivative_error = (R_error - R_last_error)/R_time_change;
  R_last_error = R_error;
  R_integral_error += (R_error*R_time_change);
  turn_rate = (kp*R_error) + (kd*R_derivative_error) + (ki*R_integral_error);
  //Serial.println(derivative_error);
  R_time_prev = R_time_now;

  forward_power = 85;
  right_motor_speed = forward_power + turn_rate;
  left_motor_speed = forward_power - turn_rate;
  
  // //sets boundaries on how high and low the PWM values sent to the motors can be
   if(right_motor_speed>255)  right_motor_speed = 255;
   if(right_motor_speed<-255) right_motor_speed = -255;
  
  
   if(left_motor_speed>255)   left_motor_speed = 255;
   if(left_motor_speed<-255)  left_motor_speed = -255;

   // //sends the motor speeds to the right and left motors
   rightMotor.setSpeed(right_motor_speed);
   leftMotor.setSpeed(left_motor_speed);
}

void forward(double mm)
{ 

  left_motor_speed = 50;
  right_motor_speed = 50;
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
}



void leftTurn()
{ 

  left_motor_speed = -50;
  right_motor_speed = 50;
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
}

void shallowLeftTurn()
{ 

  left_motor_speed = -50;
  right_motor_speed = 50;
  float distance = 0;
  leftMotor.setSpeed(left_motor_speed);
  rightMotor.setSpeed(right_motor_speed);
  left_Enc.readAndReset();

  while(distance<30)
    {
      left_motor_counter = left_Enc.read();
      distance = abs((left_motor_counter*pulses_to_mm));
    }

  leftMotor.setSpeed(0);
  rightMotor.setSpeed(0);
}


//this function makes a right turn
void rightTurn()
{ 

  left_motor_speed = 50;
  right_motor_speed = -50;
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
}

//this function makes a shallow right turn
void shallowRightTurn()
{ 

  left_motor_speed = 50;
  right_motor_speed = -50;
  float distance = 0;
  leftMotor.setSpeed(left_motor_speed);
  rightMotor.setSpeed(right_motor_speed);
  left_Enc.readAndReset();

  while(distance<30)
    {
      left_motor_counter = left_Enc.read();


      distance = abs((left_motor_counter*pulses_to_mm));
    }

  leftMotor.setSpeed(0);
  rightMotor.setSpeed(0);
}
//this function makes a 180 degree turn
void aboutTurn()
{ 

  left_motor_speed = 50;
  right_motor_speed = -50;
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
}

//this function makes a special right turn where it rotates more than 90 degrees
void specialRightTurn()
{ 

  left_motor_speed = 50;
  right_motor_speed = -50;
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

//this function performs a special left turn where it rotates less than 90 degrees
void specialLeftTurn()
{ 

  left_motor_speed = -50;
  right_motor_speed = 50;
  float distance = 0;
  leftMotor.setSpeed(left_motor_speed);
  rightMotor.setSpeed(right_motor_speed);
  left_Enc.readAndReset();

  while(distance<60)
    {
      left_motor_counter = left_Enc.read();
      distance = abs((left_motor_counter*pulses_to_mm));
    }

  leftMotor.setSpeed(0);
  rightMotor.setSpeed(0);
  delay(4000);
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
  ir_distance = (478.49*(pow(ir_average_val,-0.866)))*10; //analog to mm function
  if (ir_distance>150) ir_distance = 150;
    return ir_distance;
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

void wallFollowSetDis(int distance_to_travel)
{
  float distance = 0;
  left_Enc.readAndReset();

  while(distance < distance_to_travel)
    {
      left_motor_counter = left_Enc.read();
      left_ir_distance = readIRSensor(left_ir_pin);
      right_ir_distance = readIRSensor(right_ir_pin);
      rightWallFollowPID();
      distance = abs((left_motor_counter*pulses_to_mm));
      delay(40);
    }

  leftMotor.setSpeed(0);
  rightMotor.setSpeed(0);
}
