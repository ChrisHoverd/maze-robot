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

// declare encoder input pins
// motor 1 and motor 2 are the right and left motors respectively, as seen from above with nose wheel in the front
const int m1_ch_A = 2;
const int m1_ch_B = 3;
const int m2_ch_A = 18;
const int m2_ch_B = 19;

//declare motor directions, ie. CW or CCW
String m1_direction = "";
String m2_direction = "";

//declare motor PWM values, ie. -255 to 255
int m1_PWM;
int m2_PWM;

//declare encoder state and counter variables
volatile int m1_counter = 0;
volatile int m1_current_state_ch_A;
volatile int m1_current_state_ch_B;
volatile int m2_counter = 0;
volatile int m2_current_state_ch_A;
volatile int m2_current_state_ch_B;

//instantiate the motor objects
CytronMD motor1(PWM_DIR, 5, 4); // Motor 1 EN = Pin 5, DIR = 4
CytronMD motor2(PWM_DIR, 6, 7); // Motor 2 EN = Pin 6, Dir = 7

void setup() {
  // setup code runs once at the beginning of the program

  //declare encoder pins as inputs
  pinMode (m1_ch_A, INPUT);
  pinMode (m1_ch_B, INPUT);
  pinMode (m2_ch_A, INPUT);
  pinMode (m2_ch_B, INPUT);
  
  //sets the data rate in bits per second (baud)
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:

  motor1.setSpeed(200);   // Motor 1 runs forward at 50% speed.
  motor2.setSpeed(200);  // Motor 2 runs backward at 50% speed.
  delay(3000);
  
  motor1.setSpeed(-200);   // Motor 1 runs forward at full speed.
  motor2.setSpeed(-200);  // Motor 2 runs backward at full speed.
  delay(3000);
}

