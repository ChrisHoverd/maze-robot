
# Maze Solving Robot

This project was developed for Ontario Tech University Mechatronics Design ME4100U course. 

## Components

DC-DC Power Module 25W (DFR0205)                                               x 1 \
0.8Amp 5V-26V DC Motor Driver Shield for Arduino (2 Channels) (SHIELD-2AMOTOR) x 1 \
DF Robot Motor Kit w/ Encoders                                                 x 1 \
GOLDBAT 1500mAh 11.1V 100C 3S Lipo Battery                                     x 1 \
Ball Transfer Bearing                                                          x 1 \
Sharp GP2Y0E02A Infrared Sensors                                               x 2 \
TowerPro Micro Servo Motor                                                     x 1 \
Various Wiring and Connectors                                                  x 1

## Program Descriptions
For the Phase A demonstration, *hardcoded-phaseA.ino* was used, however, many other programs were
developed to calibrate sensors, troubleshoot algorithims, and to develop the overall solution. 

### hardcoded-phaseA.ino
This was the program used in the Phase A demonstration. The reason it's called *hardcoded*
is because it is not doing any wall following. It simply uses the encoder counts to determine
how much distance it has travelled. Using the map provided beforehand, the straightaways and left/right
turns were programmed. Although not ideal, this was done as this program provided better results than the 
wall following *PID-tests.ino* program at the time of the Phase A demonstration.

#### main.ino
This is the main program where all the other programs will be integrated into. The wall following PID 
and wheel odometry will be integrated into this program. As it stands, this program has the same
PID algorithim as *PID-tests.ino*, which does not provide good results. This program also incorporates
the use of a timer interrupt. The interrupt is triggered every 100 ms where the PID and wheel odometry
algorithims are executed. This allows the program to work in a less sequential and more autonomous way.

### PID-tests.ino
This program is where development of the wall following PID was done. It is the same PID that was put into *main.ino*.
Development of the wall following PID is done in this program before it is integrated into others.

### counter_to_mm_test.ino
The main use of this code is to obtain encoder values to create a Millimeter/Encoder count conversion.
With this conversion, it is possible to determine distance travelled from the encoder count.

### encoder-tests.ino
The main purpose of this code is to test encoder readings and troubleshoot. This code was adapted from the Encoder.h "Basic" 
example.

### ir-tests.ino
The main function of this program is to test the functionality of the IR Sensor and to gain data to plot a 
graph relating analog readings and distance.