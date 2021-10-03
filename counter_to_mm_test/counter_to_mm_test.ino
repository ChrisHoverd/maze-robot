//use this code to conduct required tests to receive good readings for mm/pulses

//include libraries
#include "CytronMotorDriver.h" 
#include "Encoder.h"
#include "TimerThree.h"

// declare encoder input pins
// motor 1 and motor 2 are the left and right motors respectively, as seen from above with tripod wheel in the rear
const int m1_ch_A = 2;
const int m1_ch_B = 3;
const int m2_ch_A = 18;
const int m2_ch_B = 19;

//instantiate the encoder objects
Encoder m1_Enc(m1_ch_A, m1_ch_B);
Encoder m2_Enc(m2_ch_A, m2_ch_B);


//declare encoder counter variables
volatile long m1_counter = 0;
volatile long m2_counter = 0;

//instantiate the motor objects
CytronMD motor1(PWM_DIR, 5, 4); // Motor 1 EN = Pin 5, DIR = 4
CytronMD motor2(PWM_DIR, 6, 7); // Motor 2 EN = Pin 6, Dir = 7

//declare motor PWM values, ie. -255 to 255
int m1_PWM = 60;
int m2_PWM = 60;


void setup() {
  // setup code runs once at the beginning of the program

  //declare encoder pins as inputs
  pinMode (m1_ch_A, INPUT_PULLUP);
  pinMode (m1_ch_B, INPUT_PULLUP);
  pinMode (m2_ch_A, INPUT_PULLUP);
  pinMode (m2_ch_B, INPUT_PULLUP);
  
  //sets the data rate in bits per second (baud)
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:

    delay(5000);
    motor1.setSpeed(m1_PWM);
    motor2.setSpeed(m2_PWM);

    delay(3000);
    motor1.setSpeed(0);
    motor2.setSpeed(0);
    
    while(true)

    {
      m1_counter = m1_Enc.read();
      m2_counter = m2_Enc.read();
      Serial.print("M1 Counter: ");
      Serial.print(m1_counter);
      Serial.print("  |  ");
      Serial.print("M2 Counter: ");
      Serial.println(m2_counter);
      delay(3000);
    }


}



