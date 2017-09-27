#include <Wire.h>
#include <VL53L0X.h>
const int MIN = 10;
const int MAX = 1300;

//#define XSHUT_pin6 A7  //not required for address change
//#define XSHUT_pin5 A6
#define XSHUT_pin4 A3
#define XSHUT_pin3 A2
#define XSHUT_pin2 A1
#define XSHUT_pin1 A0

//ADDRESS_DEFAULT 0b0101001 or 41
//#define Sensor1_newAddress 41 not required address change
#define Sensor2_newAddress 42
#define Sensor3_newAddress 43
#define Sensor4_newAddress 44
#define Sensor5_newAddress 45
#define Sensor6_newAddress 46

VL53L0X Sensor1;
VL53L0X Sensor2;
VL53L0X Sensor3;
VL53L0X Sensor4;
//VL53L0X Sensor5;
//VL53L0X Sensor6;

void setup()
{ /*WARNING*/
  //Shutdown pins of VL53L0X ACTIVE-LOW-ONLY NO TOLERANT TO 5V will fry them
  pinMode(XSHUT_pin1, OUTPUT);
  pinMode(XSHUT_pin2, OUTPUT);
  pinMode(XSHUT_pin3, OUTPUT);
  pinMode(XSHUT_pin4, OUTPUT);
//  pinMode(XSHUT_pin5, OUTPUT);
//  pinMode(XSHUT_pin6, OUTPUT);
  
  Serial.begin(115200);
  
  Wire.begin();
  //Change address of sensor and power up next one
 // Sensor6.setAddress(Sensor6_newAddress);//For power-up procedure t-boot max 1.2ms "Datasheet: 2.9 Power sequence"
 
 // pinMode(XSHUT_pin5, INPUT);
 // delay(10); 
 // Sensor5.setAddress(Sensor5_newAddress);
 
  pinMode(XSHUT_pin4, INPUT);
  delay(10);
  Sensor4.setAddress(Sensor4_newAddress);

  pinMode(XSHUT_pin3, INPUT);
  delay(10);
  Sensor3.setAddress(Sensor3_newAddress);
  
  pinMode(XSHUT_pin2, INPUT);
  delay(10);
  Sensor2.setAddress(Sensor2_newAddress);
  
  pinMode(XSHUT_pin1, INPUT);
  delay(10);
  //ADDRESS_DEFAULT 0b0101001 or 41
  
  Sensor1.init();
  Sensor2.init();
  Sensor3.init();
  Sensor4.init();
 // Sensor5.init();
  //Sensor6.init();
  
  Sensor1.setTimeout(500);
  Sensor2.setTimeout(500);
  Sensor3.setTimeout(500);
  Sensor4.setTimeout(500);
 // Sensor5.setTimeout(500);
 // Sensor6.setTimeout(500);

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  Sensor1.startContinuous();
  Sensor2.startContinuous();
  Sensor3.startContinuous();
  Sensor4.startContinuous();
 // Sensor5.startContinuous();
  //Sensor6.startContinuous();

}

void loop()
{

  float Sensor1Smooth = Sensor1.readRangeContinuousMillimeters();
  float Sensor1Filter = constrain(Sensor1Smooth, MIN , MAX);
   
  float Sensor2Smooth = Sensor2.readRangeContinuousMillimeters();
  float Sensor2Filter constrain(Sensor2Smooth, MIN , MAX);
  
  float Sensor3Smooth = Sensor3.readRangeContinuousMillimeters();
  float Sensor3Filter = constrain(Sensor3Smooth, MIN, MAX);
   
  float Sensor4Smooth = Sensor4.readRangeContinuousMillimeters();
  float Sensor4Filter constrain(Sensor4Smooth, MIN , MAX);
  
 // float Sensor5Smooth = Sensor5.readRangeContinuousMillimeters();
  //float Sensor5Filter = constrain(Sensor5Smooth, MIN, MAX);


  
  //Serial.print("!ANG:");
  Serial.print(Sensor1Filter);
  Serial.print(',');
  Serial.print(Sensor2Filter);
  Serial.print(',');
  Serial.print(Sensor3Filter);
  Serial.print(','); 
  Serial.print(Sensor4Filter);
  //Serial.print(',');
  //Serial.print(Sensor5Filter);

  Serial.println();
  delay (0.1);
  
    
}
