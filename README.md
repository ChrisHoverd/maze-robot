
# Maze-Solving SARbot (Search and Rescue Robot)

This project was developed as a part of the Mechatronics Design ME4100U course at Ontario Tech University.

## Program Descriptions

### Phase A (Phase-A.ino)
This was the program used in the Phase A demonstration. It simply usesd the encoder counts to determine
how much distance it had travelled. Using the map provided beforehand, the straightaways and left/right
turns were hardcoded into the robots path by determining distance travelled. Although not ideal, this was 
done as this program provided better results than the wall-following and turn counting logic developed at 
the time of the demo presentation.

#### Phase A Components

Arduino Mega                                                                   x 1 \
DC-DC Power Module 25W (DFR0205)                                               x 1 \
0.8Amp 5V-26V DC Motor Driver Shield for Arduino (2 Channels) (SHIELD-2AMOTOR) x 1 \
DF Robot Motor Kit w/ Encoders                                                 x 1 \
GOLDBAT 1500mAh 11.1V 100C 3S Lipo Battery                                     x 1 \
Ball Transfer Bearing                                                          x 1 \
Sharp GP2Y0E02A Infrared Sensors                                               x 2 \
TowerPro Micro Servo Motor                                                     x 1 \
Various Wiring and Connectors                                                  

### Phase B (Phase-B.ino)
This was the program used in the Phase B demonstration. It integrated a PID wall-following control system
with turn-counting logic to find its way to the trapped miner.

#### Phase B Components

Arduino Leonardo                                                               x 1 \
DC-DC Power Module 25W (DFR0205)                                               x 1 \
0.8Amp 5V-26V DC Motor Driver Shield for Arduino (2 Channels) (SHIELD-2AMOTOR) x 1 \
6V 210:1 75 RPM Micro Motors                                                   x 2 \
Tattu 3S1P 75C 11.1V 650mAh Lipo Battery                                       x 1 \
Ball Transfer Bearing                                                          x 1 \
Sharp GP2Y0A51SK0F Analog Distance Sensor 2-15cm                               x 3 \
TowerPro Micro Servo Motor                                                     x 1 \
Pololu Wheel 60×8mm                                                            x 2 \
Various Wiring and Connectors     

### Phase C (Phase-C.ino)
This was the program used in the Phase C demonstration. The main difference between Phase C and Phase B
was that Phase C included obstacles. To remove the obstacles from the path, a crane with a gripper at the 
end of it was used. All other wall following and turn-counting logic was the same as Phase B. 

#### Phase C Components

Arduino Leonardo                                                               x 1 \
DC-DC Power Module 25W (DFR0205)                                               x 1 \
0.8Amp 5V-26V DC Motor Driver Shield for Arduino (2 Channels) (SHIELD-2AMOTOR) x 1 \
6V 210:1 75 RPM Micro Motors                                                   x 2 \
Tattu 3S1P 75C 11.1V 650mAh Lipo Battery                                       x 1 \
Ball Transfer Bearing                                                          x 1 \
Sharp GP2Y0A51SK0F Analog Distance Sensor 2-15cm                               x 4 \
TowerPro Micro Servo Motor                                                     x 1 \
Radio Shack Servo Motor                                                        x 1 \
Pololu Wheel 60×8mm                                                            x 2 \
Various Wiring and Connectors     
