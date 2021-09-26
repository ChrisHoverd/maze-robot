 
 //include motor driver library
 #include "CytronMotorDriver.h" 

// declare encoder input pins
// motor 1 and 2 as are the right and left motors respectively, as seen from above with nose wheel in the front
int motor1_ch_A = 2;
int motor1_ch_B = 3;
int motor2_ch_A = 18;
int motor2_ch_B = 19;

CytronMD motor1(PWM_DIR, 5, 4); // Motor 1 EN = Pin 5, DIR = 4
CytronMD motor2(PWM_DIR, 6, 7); // Motor 2 EN = Pin 6, Dir = 7

void setup() {
  // setup code runs once at the begining of the program
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
    motor1.setSpeed(128);   // Motor 1 runs forward at 50% speed.
    motor2.setSpeed(-128);  // Motor 2 runs backward at 50% speed.
  delay(1000);
  
  motor1.setSpeed(255);   // Motor 1 runs forward at full speed.
  motor2.setSpeed(-255);  // Motor 2 runs backward at full speed.
  delay(1000);
}

