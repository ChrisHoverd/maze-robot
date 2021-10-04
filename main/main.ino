/*
    Maze Robot 

    The robot solves a maze.

    Components:
    DC-DC Power Module 25W (DFR0205)                                               x 1
    Pololu 5V, 2.5A Step-Down Voltage Regulator                                    x 1
    0.8Amp 5V-26V DC Motor Driver Shield for Arduino (2 Channels) (SHIELD-2AMOTOR) x 1
    Micro DC Motor with Encoder (FIT0458)                                          x 2
    GOLDBAT 1500mAh 11.1V 100C 3S Lipo Battery                                     x 1

    Pin Connections: 


    Created 9/26/2021
    Christopher Hoverd

    https://github.com/ChrisHoverd/maze-robot
*/

//include motor driver library
#include "CytronMotorDriver.h" 
#include "Encoder.h"
#include "TimerThree.h"

// declare encoder input pins
// motor 1 and motor 2 are the left and right motors respectively, as seen from above with tripod wheel in the rear
const int m1_ch_A = 2;
const int m1_ch_B = 3;
const int m2_ch_A = 18;
const int m2_ch_B = 19;

//declare ir values
int left_ir_pin = A0;
int front_ir_pin = A1;
float sensor_sample = 10;
float left_ir_distance = 0;
float front_ir_distance =0;

//declare mm/pulse constant
//converts encoder ticks to mm
const float pulses_to_mm = 0.054048211;

//instantiate the encoder objects
Encoder m1_Enc(m1_ch_B, m1_ch_A); // swapped channels A & B so that both motors have positive readings when moving forward
Encoder m2_Enc(m2_ch_A, m2_ch_B);


//declare encoder counter variables
volatile long m1_counter = 0;
volatile long m2_counter = 0;
volatile double m1_delta = 0;
volatile double m2_delta = 0;

volatile double orientation = 0;
volatile double delta_orientation = 0;
volatile double delta_distance = 0;
volatile double xPosition = 0;
volatile double yPosition = 0;

volatile 

float distance;

//instantiate the motor objects
CytronMD motor1(PWM_DIR, 5, 4); // Motor 1 EN = Pin 5, DIR = 4
CytronMD motor2(PWM_DIR, 6, 7); // Motor 2 EN = Pin 6, Dir = 7



//declare motor PWM values, ie. -255 to 255
int m1_PWM = 60;
int m2_PWM = 60;

//declare wheel odometry variables

int wheelbase = 96;
void setup() {
  // setup code runs once at the beginning of the program

  //declare encoder pins as inputs
  pinMode (m1_ch_A, INPUT_PULLUP);
  pinMode (m1_ch_B, INPUT_PULLUP);
  pinMode (m2_ch_A, INPUT_PULLUP);
  pinMode (m2_ch_B, INPUT_PULLUP);
  
  //sets the reference voltage for the analog inputs, ie. 3.3V from the arduino for the IR sensors
  analogReference(EXTERNAL);

  Timer3.initialize(50000); // 50 millisecond interrupt timer
  Timer3.attachInterrupt(timerInterrupt); //calls timerInterrupt every 50 milliseconds
  
  //sets the data rate in bits per second (baud)
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
    
    motor1.setSpeed(m1_PWM);
    motor2.setSpeed(m2_PWM);

    while(distance<200)
    {
      m1_counter = m1_Enc.read();
      distance = abs((m1_counter*pulses_to_mm));
    }

    motor1.setSpeed(0);
    motor2.setSpeed(0);
    delay(2000);
    m1_PWM = 60;
    m2_PWM = -60;
    motor1.setSpeed(m1_PWM);
    motor2.setSpeed(m2_PWM);
    distance = 0;
    m1_Enc.readAndReset();
    while(distance<96)
    {
      m1_counter = m1_Enc.read();
      distance = abs((m1_counter*pulses_to_mm));
    }
    motor1.setSpeed(0);
    motor2.setSpeed(0);
  delay(100000);
}

// void leftIRCalc()
// {
//     for(int i = 0; i<sensor_sample; ++i)
//   {
//     left_ir_sum+= analogRead(left_ir_pin);
//   }

//   left_ir_average_val = left_ir_sum/sensor_sample;
//   Serial.print("Sensor Value: ");
//   Serial.println(left_ir_average_val);
//   left_ir_distance = (-0.0903*left_ir_average_val + 65.306)*10; //analog to mm function
//   Serial.print("Distance in cm: ");
//   Serial.println(left_ir_distance); 
//   left_ir_sum = 0;
// }

void timerInterrupt()
{
  left_ir_distance = readIRSensor(left_ir_pin);
  front_ir_distance = readIRSensor(front_ir_pin);
}

float readIRSensor(int ir_pin)
{
  float ir_sum;
  float ir_average_val;
  float ir_distance;
  for(int i = 0; i<sensor_sample; ++i)
  {
    ir_sum+= analogRead(ir_pin);
  }

  ir_average_val = ir_sum/sensor_sample;
  ir_distance = (-0.0903*ir_average_val + 65.306)*10; //analog to mm function
  return ir_distance;

}