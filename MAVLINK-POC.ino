
// Arduino MAVLink  http://forum.arduino.cc/index.php?topic=382592.0
// https://github.com/ArduPilot/ardupilot_wiki/blob/master/dev/source/docs/code-overview-object-avoidance.rst

/*
 *  The system id of the message should match the system id of the vehicle 
 *  (default is "1" but can be changed using the SYSID_THISMAV parameter). 
 *  The component id can be anything but MAV_COMP_ID_PATHPLANNER (195) 
 *  or MAV_COMP_ID_PERIPHERAL (158) are probably good choices.
 *  
 * # Define function to send distance_message mavlink message for mavlink based rangefinder, must be >10hz
# http://mavlink.org/messages/common#DISTANCE_SENSOR
def send_distance_message(dist):
    msg = vehicle.message_factory.distance_sensor_encode(
        0,          # time since system boot, not used
        1,          # min distance cm
        10000,      # max distance cm
        dist,       # current distance, must be int
        0,          # type = 0 MAV_DISTANCE_SENSOR_LASER Laser rangefinder, e.g. LightWare SF02/F or PulsedLight units
        0,          # onboard id, not used
        mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270, # must be set to MAV_SENSOR_ROTATION_PITCH_270 for mavlink rangefinder, represents downward facing
        0           # covariance, not used
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()
    if args.verbose:
        log.debug("Sending mavlink distance_message:" +str(dist))
        */
  
 
#include "mavlink.h"        // Mavlink interface
#include "mavlink_msg_distance_sensor.h"
#include <Wire.h>
#include <VL53L0X.h>
const int MIN = 10;
const int MAX = 1600;
const int idle = 200;
/*
Valid values are (even numbers only):
Pre: 12 to 18 (initialized to 14 by default)
Final: 8 to 14 (initialized to 10 by default)
*/
const int PreRng = 18;  
const int PostRng = 14;

const int Scale = 10;
#define bRate 115200


//#define XSHUT_pin7 xx  //not required for address change
#define XSHUT_pin6 8
#define XSHUT_pin5 7
#define XSHUT_pin4 6
#define XSHUT_pin3 5
#define XSHUT_pin2 4  //skip pin #3 :  upward looking  
#define XSHUT_pin1 2

//ADDRESS_DEFAULT 0b0101001 or 41
//#define Sensor1_newAddress 41 not required address change
#define Sensor2_newAddress 42
#define Sensor3_newAddress 43
#define Sensor4_newAddress 44
#define Sensor5_newAddress 45
#define Sensor6_newAddress 46
//#define Sensor7_newAddress 47

VL53L0X Sensor1;
VL53L0X Sensor2;
VL53L0X Sensor3;
VL53L0X Sensor4;
VL53L0X Sensor5;
VL53L0X Sensor6;

void setup()
{ /*WARNING*/
  //Shutdown pins of VL53L0X ACTIVE-LOW-ONLY NO TOLERANT TO 5V will fry them
  pinMode(XSHUT_pin1, OUTPUT);
  digitalWrite(XSHUT_pin1, LOW); 
  pinMode(XSHUT_pin2, OUTPUT);
  digitalWrite(XSHUT_pin2, LOW);
  pinMode(XSHUT_pin3, OUTPUT);
  digitalWrite(XSHUT_pin3, LOW);
  pinMode(XSHUT_pin4, OUTPUT);
  digitalWrite(XSHUT_pin4, LOW);
  pinMode(XSHUT_pin5, OUTPUT);
  digitalWrite(XSHUT_pin5, LOW);
  pinMode(XSHUT_pin6, OUTPUT);
  digitalWrite(XSHUT_pin6, LOW);

  
  Serial.begin(bRate);
  Wire.begin();

  
  //Change address of sensor and power up next one
 // Sensor6.setAddress(SensorXX_newAddress);//For power-up procedure t-boot max 1.2ms "Datasheet: 2.9 Power sequence"

  pinMode(XSHUT_pin6, INPUT);;
  delay(10);
  Sensor6.setAddress(Sensor6_newAddress);
 
  pinMode(XSHUT_pin5, INPUT);;
  delay(10);
  Sensor5.setAddress(Sensor5_newAddress);
  
  pinMode(XSHUT_pin4, INPUT);
  delay(10);
  Sensor4.setAddress(Sensor4_newAddress);
  
  pinMode(XSHUT_pin3, INPUT);;
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
  Sensor5.init();
  Sensor6.init();
  
  Sensor1.setTimeout(idle);
  Sensor2.setTimeout(idle);
  Sensor3.setTimeout(idle);
  Sensor4.setTimeout(idle);
  Sensor5.setTimeout(idle);
  Sensor6.setTimeout(idle);


  // lower the return signal rate limit (default is 0.25 MCPS)
  Sensor1.setSignalRateLimit(0.25);
  Sensor2.setSignalRateLimit(0.25);
  Sensor3.setSignalRateLimit(0.25);
  Sensor4.setSignalRateLimit(0.25);
  Sensor5.setSignalRateLimit(0.25);
  Sensor6.setSignalRateLimit(0.25);
  
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  Sensor1.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, PreRng);
  Sensor1.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, PostRng);
  Sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, PreRng);
  Sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, PostRng);
  Sensor3.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, PreRng);
  Sensor3.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, PostRng);
  Sensor4.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, PreRng);
  Sensor4.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, PostRng);
  Sensor5.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, PreRng);
  Sensor5.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, PostRng);
  Sensor6.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, PreRng);
  Sensor6.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, PostRng);

}


void loop() {

  command_heartbeat();
  command_distance_1();
  command_distance_2();
  //command_distance_3();
  command_distance_4();
  //command_distance_5();
  command_distance_6();
 
}

void command_heartbeat() {

  //< ID 1 for this system
  int sysid = 100;                   
  //< The component sending the message.
  int compid = MAV_COMP_ID_PATHPLANNER;    
  
  // Define the system type, in this case ground control station
  uint8_t system_type =MAV_TYPE_GCS;
  uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
  
  uint8_t system_mode = 0; 
  uint32_t custom_mode = 0;                
  uint8_t system_state = 0;
  
  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  // Pack the message
  mavlink_msg_heartbeat_pack(sysid,compid, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state);
  
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  // Send the message 
  //delay(1);
  Serial.write(buf, len);
}


void command_distance_1() {

// READ THE DISTANCE SENSOR
  float Sensor1Smooth  = Sensor1.readRangeSingleMillimeters();
  Sensor1Smooth = constrain(Sensor1Smooth, MIN , MAX);
  float dist1 = Sensor1Smooth / Scale;

  //MAVLINK DISTANCE MESSAGE
  int sysid = 1;                   
  //< The component sending the message.
  int compid = 158;    

  uint32_t time_boot_ms = 0; /*< Time since system boot*/
  uint16_t min_distance = 1; /*< Minimum distance the sensor can measure in centimeters*/
  uint16_t max_distance = 170; /*< Maximum distance the sensor can measure in centimeters*/
  uint16_t current_distance = dist1; /*< Current distance reading*/
  uint8_t type = 0; /*< Type from MAV_DISTANCE_SENSOR enum.*/
  uint8_t id = 1; /*< Onboard ID of the sensor*/
  uint8_t orientation = 4; /*(0=forward, each increment is 45degrees more in clockwise direction), 24 (upwards) or 25 (downwards)*/
// Consumed within ArduPilot by the proximity class

  uint8_t covariance = 0; /*< Measurement covariance in centimeters, 0 for unknown / invalid readings*/


  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
 mavlink_msg_distance_sensor_pack(sysid,compid,&msg,time_boot_ms,min_distance,max_distance,current_distance,type,id,orientation,covariance);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes) 
  //delay(1);
  Serial.write(buf, len);
}


void command_distance_2() {

// READ THE DISTANCE SENSOR
  float Sensor2Smooth  = Sensor2.readRangeSingleMillimeters();
  Sensor2Smooth = constrain(Sensor2Smooth, MIN , MAX);
  float dist2 = Sensor2Smooth / Scale;

  //MAVLINK DISTANCE MESSAGE
  int sysid = 1;                   
  //< The component sending the message.
  int compid = 158;    

  uint32_t time_boot_ms = 0; /*< Time since system boot*/
  uint16_t min_distance = 1; /*< Minimum distance the sensor can measure in centimeters*/
  uint16_t max_distance = 170; /*< Maximum distance the sensor can measure in centimeters*/
  uint16_t current_distance = dist2; /*< Current distance reading*/
  uint8_t type = 0; /*< Type from MAV_DISTANCE_SENSOR enum.*/
  uint8_t id = 2; /*< Onboard ID of the sensor*/
  uint8_t orientation = 6; /*(0=forward, each increment is 45degrees more in clockwise direction), 24 (upwards) or 25 (downwards)*/
// Consumed within ArduPilot by the proximity class

  uint8_t covariance = 0; /*< Measurement covariance in centimeters, 0 for unknown / invalid readings*/


  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
 mavlink_msg_distance_sensor_pack(sysid,compid,&msg,time_boot_ms,min_distance,max_distance,current_distance,type,id,orientation,covariance);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes) 
  //delay(1);
  Serial.write(buf, len);
}


void command_distance_3() {

// READ THE DISTANCE SENSOR
  float Sensor3Smooth  = Sensor3.readRangeSingleMillimeters();
  Sensor3Smooth = constrain(Sensor3Smooth, MIN , MAX);
  float dist3 = Sensor3Smooth / Scale;

  //MAVLINK DISTANCE MESSAGE
  int sysid = 1;                   
  //< The component sending the message.
  int compid = 158;    

  uint32_t time_boot_ms = 0; /*< Time since system boot*/
  uint16_t min_distance = 1; /*< Minimum distance the sensor can measure in centimeters*/
  uint16_t max_distance = 170; /*< Maximum distance the sensor can measure in centimeters*/
  uint16_t current_distance = dist3; /*< Current distance reading*/
  uint8_t type = 0; /*< Type from MAV_DISTANCE_SENSOR enum.*/
  uint8_t id = 3; /*< Onboard ID of the sensor*/
  uint8_t orientation = 7; /*(0=forward, each increment is 45degrees more in clockwise direction), 24 (upwards) or 25 (downwards)*/
// Consumed within ArduPilot by the proximity class

  uint8_t covariance = 0; /*< Measurement covariance in centimeters, 0 for unknown / invalid readings*/


  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
 mavlink_msg_distance_sensor_pack(sysid,compid,&msg,time_boot_ms,min_distance,max_distance,current_distance,type,id,orientation,covariance);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes) 
  //delay(1);
  Serial.write(buf, len);
}



void command_distance_4() {

// READ THE DISTANCE SENSOR
  float Sensor4Smooth  = Sensor4.readRangeSingleMillimeters();
  Sensor4Smooth = constrain(Sensor4Smooth, MIN , MAX);
  float dist4 = Sensor4Smooth / Scale;

  //MAVLINK DISTANCE MESSAGE
  int sysid = 1;                   
  //< The component sending the message.
  int compid = 158;    

  uint32_t time_boot_ms = 0; /*< Time since system boot*/
  uint16_t min_distance = 1; /*< Minimum distance the sensor can measure in centimeters*/
  uint16_t max_distance = 170; /*< Maximum distance the sensor can measure in centimeters*/
  uint16_t current_distance = dist4; /*< Current distance reading*/
  uint8_t type = 0; /*< Type from MAV_DISTANCE_SENSOR enum.*/
  uint8_t id = 4; /*< Onboard ID of the sensor*/
  uint8_t orientation = 0; /*(0=forward, each increment is 45degrees more in clockwise direction), 24 (upwards) or 25 (downwards)*/
// Consumed within ArduPilot by the proximity class

  uint8_t covariance = 0; /*< Measurement covariance in centimeters, 0 for unknown / invalid readings*/


  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
 mavlink_msg_distance_sensor_pack(sysid,compid,&msg,time_boot_ms,min_distance,max_distance,current_distance,type,id,orientation,covariance);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes) 
  //delay(1);
  Serial.write(buf, len);
}

void command_distance_5() {

// READ THE DISTANCE SENSOR
  float Sensor5Smooth  = Sensor5.readRangeSingleMillimeters();
  Sensor5Smooth = constrain(Sensor5Smooth, MIN , MAX);
  float dist5 = Sensor5Smooth / Scale;

  //MAVLINK DISTANCE MESSAGE
  int sysid = 1;                   
  //< The component sending the message.
  int compid = 158;    

  uint32_t time_boot_ms = 0; /*< Time since system boot*/
  uint16_t min_distance = 1; /*< Minimum distance the sensor can measure in centimeters*/
  uint16_t max_distance = 170; /*< Maximum distance the sensor can measure in centimeters*/
  uint16_t current_distance = dist5; /*< Current distance reading*/
  uint8_t type = 0; /*< Type from MAV_DISTANCE_SENSOR enum.*/
  uint8_t id = 5; /*< Onboard ID of the sensor*/
  uint8_t orientation = 1; /*(0=forward, each increment is 45degrees more in clockwise direction), 24 (upwards) or 25 (downwards)*/
// Consumed within ArduPilot by the proximity class

  uint8_t covariance = 0; /*< Measurement covariance in centimeters, 0 for unknown / invalid readings*/


  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
 mavlink_msg_distance_sensor_pack(sysid,compid,&msg,time_boot_ms,min_distance,max_distance,current_distance,type,id,orientation,covariance);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes) 
  //delay(1);
  Serial.write(buf, len);
}

void command_distance_6() {

// READ THE DISTANCE SENSOR
  float Sensor6Smooth  = Sensor6.readRangeSingleMillimeters();
  Sensor6Smooth = constrain(Sensor6Smooth, MIN , MAX);
  float dist6 = Sensor6Smooth / Scale;

  //MAVLINK DISTANCE MESSAGE
  int sysid = 1;                   
  //< The component sending the message.
  int compid = 158;    

  uint32_t time_boot_ms = 0; /*< Time since system boot*/
  uint16_t min_distance = 1; /*< Minimum distance the sensor can measure in centimeters*/
  uint16_t max_distance = 170; /*< Maximum distance the sensor can measure in centimeters*/
  uint16_t current_distance = dist6; /*< Current distance reading*/
  uint8_t type = 0; /*< Type from MAV_DISTANCE_SENSOR enum.*/
  uint8_t id = 6; /*< Onboard ID of the sensor*/
  uint8_t orientation = 2; /*(0=forward, each increment is 45degrees more in clockwise direction), 24 (upwards) or 25 (downwards)*/
// Consumed within ArduPilot by the proximity class

  uint8_t covariance = 0; /*< Measurement covariance in centimeters, 0 for unknown / invalid readings*/


  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
 mavlink_msg_distance_sensor_pack(sysid,compid,&msg,time_boot_ms,min_distance,max_distance,current_distance,type,id,orientation,covariance);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes) 
  //delay(1);
  Serial.write(buf, len);
}



