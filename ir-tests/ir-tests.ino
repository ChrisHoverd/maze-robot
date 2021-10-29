/*
  The main function of this code is to test the functionality of the IR Sensor and to gain data to plot a 
  graph relating analog readings and distance
*/


#include <SharpDistSensor.h>

//change ir pin to A0, A1, or A2 depending on IR sensor
int ir_pin = A2;
int ir_val = 0;

//declare sample size value and sum values used for averaging readings
float sensor_sample = 50;
float sensor_sum = 0;
float average_sensor_val = 0;
float distance = 0;

// const float polyCoefficients[] = {19.427, -0.2249, 0.0015, -5E-06, 1E-08, -7E-12};
// const byte nbCoefficients = 6;  // Number of coefficients
// const byte medianFilterWindowSize = 3;
// const unsigned int minVal = 55; // ~800 mm
// const unsigned int maxVal = 400; // ~50mm
// SharpDistSensor leftIRSensor(ir_pin, medianFilterWindowSize);

void setup() {
  Serial.begin(9600);
  //  leftIRSensor.setPolyFitCoeffs(nbCoefficients, polyCoefficients, minVal, maxVal);
  //an external analog reference of 3.3V is required because the 
  //IR sensor operates on 3.3V analog logic
  //analogReference(EXTERNAL); // use AREF for reference voltage

}

void loop() {


  //comment out one of the below sections depending on use case

  //*************************************************************************

//  //Section 1
//  //use this section to obtain data required for graphing distance and analog reading values

  // for(int i = 0; i<sensor_sample; ++i)
  // {
  // //sums an average of analog sensor readings
  //   sensor_sum+= analogRead(ir_pin);
  // }
  // //takes the summed value and divides it by the number of samples
  // //prints the averaged sensor reading to the serial monitor
  // Serial.println(sensor_sum/sensor_sample); 

  // //set the sum to 0
  // sensor_sum = 0;

  // //delay 2 seconds
  // delay(1000);


//***************************************************************************************************

  // Section 2
  // use below to test IR distance readings using determined analog reading to distance function

  for(int i = 0; i<sensor_sample; ++i)
  {
    //sums an average of analog sensor readings
    sensor_sum+= analogRead(ir_pin);
  }
  //takes the summed value and divides it by the number of samples
  average_sensor_val = sensor_sum/sensor_sample;

  //print the average sensor value on the serial monitor for troubleshooting
  Serial.print("Sensor Value: ");
  Serial.println(average_sensor_val);

  //analog value to distance in cm function
  //distance = -0.0903*average_sensor_val + 65.306; //GP2Y0E02A (Phase A) IR sensor function
  distance = (478.49*(pow(average_sensor_val,-0.866))); //GP2Y0A51SK0F IR sensor function 
  //print distance value on serial monitor for troubleshooting
  Serial.print("Distance in cm: ");
  Serial.println(distance); 

  //set the sum to 0
  sensor_sum = 0;

  //delay 2 seconds
  delay(2000);



  //***************************************************************************************************

  // Section 3
  // use below to test SharpIRdist library

  //   // Get distance from sensor
  // unsigned int distance = leftIRSensor.getDist();

  // // Print distance to Serial
  // Serial.println(distance);

  // // Wait some time
  // delay(2000);
}

