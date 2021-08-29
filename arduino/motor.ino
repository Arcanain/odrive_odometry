/* Board  : Arduino Mega 2560
 * Author : Ramune6110
 * Data   : 2021 08/29
 * ********************************************************************************
 * Topic
 * Publish  | encoder
 * Sbscribe | cmd_vel
 * ********************************************************************************
 */
/* Odrive */
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>

/* ROS */
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <ArduinoHardware.h>
#include <geometry_msgs/Twist.h>

// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

// Arduino Mega or Due - Serial1
// pin 19: RX - connect to ODrive TX
// pin 18: TX - connect to ODrive RX
// See https://www.arduino.cc/reference/en/language/functions/communication/serial/ for other options
//HardwareSerial& odrive_serial = Serial1;

// Arduino Mega or Due - Serial2
// Serial1はrosserialの方で使用されるため、ArduinoとOdriveはSerial2でUART通信を行う
// pin 17: RX - connect to ODrive TX GPIO 1
// pin 16: TX - connect to ODrive RX GPIO 2
HardwareSerial& odrive_serial = Serial2;

// ODrive object
ODriveArduino odrive(odrive_serial);

void messageCb(const geometry_msgs::Twist& msg);

/* ROS */
ros::NodeHandle  nh;
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &messageCb);

std_msgs::Float32MultiArray encoder_data;
ros::Publisher encoder_pub("/encoder", &encoder_data);

/***********************************************************************
 * Global variables
 **********************************************************************/
const int kv = 16;          // moter constant
const int encoder_cpr = 90; // Number of encoder counts per revolution
const int num_motor = 2;    // moter number

float w_r = 0.0f;           // right angle accl
float w_l = 0.0f;           // left angle accl

float wheel_rad = 0.085f;   // wheel radius
float wheel_sep = 0.32f;    // Wheel interval

float speed_ang = 0.0f;     // angle velocity
float speed_lin = 0.0f;     // linear velocity

float vel1 = 0.0f;          // axis0 velocity
float vel2 = 0.0f;          // axis1 velocity

/***********************************************************************
 * Prototype declaration
 **********************************************************************/
void array_init(std_msgs::Float32MultiArray& data, int array); // Dynamic memory allocation for arrays
float get_encoder_data(int axis);                              // Get encoder value from Odrive

void ros_init()
{
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  array_init(encoder_data, num_motor);
  nh.advertise(encoder_pub);
}

void odrive_calibration()
{
  int motornum0 = 0;
  int motornum1 = 1;
  int requested_state0;
  int requested_state1;

  requested_state0 = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
  if(!odrive.run_state(motornum0, requested_state0, false /*don't wait*/)) return;
  
  requested_state1 = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
  if(!odrive.run_state(motornum1, requested_state1, false /*don't wait*/)) return;
}

void setup() 
{
  // ODrive uses 115200 baud(Odriveは115200でないと動かない様子)
  odrive_serial.begin(115200);
  delay(100);

  odrive_calibration();
  delay(1000);

  ros_init();
  delay(1000);
}

void loop() 
{
  for(int motor = 0; motor < num_motor; motor++){
    encoder_data.data[motor] = get_encoder_data(motor); 
  }
  
  encoder_pub.publish(&encoder_data);

  nh.spinOnce();
}

void messageCb(const geometry_msgs::Twist& msg)
{
  speed_lin = msg.linear.x;
  speed_ang = msg.angular.z;
  w_r = (speed_lin / wheel_rad) + ((speed_ang * wheel_sep) / (2.0 * wheel_rad));
  w_l = (speed_lin / wheel_rad) - ((speed_ang * wheel_sep) / (2.0 * wheel_rad));

  vel1 = w_r * 0.3;
  vel2 = w_l * 0.3;

  //velに送る値の単位はrpm(rad/s → turn/sに変換する必要あり)
  odrive.SetVelocity(0, vel1);
  odrive.SetVelocity(1, vel2);
}

void array_init(std_msgs::Float32MultiArray& data, int array) 
{
    data.data_length = array;
    data.data = (float *)malloc(sizeof(float)*array);
    for(int i = 0; i < array; i++){
        data.data[i] = 0.0f;
    }
}
 
float get_encoder_data(int axis)
{
    float encoder_pos = 0;
    odrive_serial << "r axis" << axis << ".encoder.pos_estimate\n";
    //encoder_pos = odrive.readFloat();
    encoder_pos = odrive.readFloat() * 100.0f;
    return encoder_pos;
}