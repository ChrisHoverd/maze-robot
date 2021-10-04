
//declare ir analog input pin and analog reading 
int ir_pin = A0;
int ir_val = 0;

//declare sample size value and sum 
float sensor_sample = 10;
float sensor_sum = 0;
float average_sensor_val = 0;
float distance = 0;


void setup() {
  Serial.begin(9600);
  analogReference(EXTERNAL); // use AREF for reference voltage
}

void loop() {
  // use below to obtain data required for graphing
  // for(int i = 0; i<sensor_sample; ++i)
  // {
  //   sensor_sum+= analogRead(ir_pin);
  // }
  // Serial.println(sensor_sum/sensor_sample); 
  // sensor_sum = 0;
  // delay(2000);

  //use below to test IR sensor after developing a function that fits the analog values
  for(int i = 0; i<sensor_sample; ++i)
  {
    sensor_sum+= analogRead(ir_pin);
  }

  average_sensor_val = sensor_sum/sensor_sample;
  Serial.print("Sensor Value: ");
  Serial.println(average_sensor_val);
  distance = -0.0903*average_sensor_val + 65.306; //analog to cm functio
  Serial.print("Distance in cm: ");
  Serial.println(distance); 
  sensor_sum = 0;
  delay(2000);
}

