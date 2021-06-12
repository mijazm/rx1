/*** 
 This code makes an Arduino Mega 2560 board act as a ROS Node. 
For a simple diff drive robot it can publish the speed and positon
of left and right motor wheels, and subscribe to a control topic 
to accept control commands (PID) from a computer / Raspberry Pi running ROS.

Author: Mijaz Mukundan

References:
1. https://github.com/merose/ROSRobotControl
2. https://maker.pro/arduino/tutorial/how-to-use-arduino-with-robot-operating-system-ros
3. https://dronebotworkshop.com/rotary-encoders-arduino/
4. https://github.com/sungjik/my_personal_robotic_companion

**/
#include <Arduino.h>
#include <L298N.h>
#include "MPU9250.h"
// Arduino – ROS headers
#include <ros.h>
#include <std_msgs/Empty.h>
#include <ros/time.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <rx1_imu/imu_raw.h>

#define LOOPTIME 100 //Update time in ms

//Define the IMU
MPU9250 IMU(Wire,0x68);
int status;


// Pin definition for right motor
const unsigned int IN1 = 4;
const unsigned int IN2 = 5;
const unsigned int EN12 = 8;

// Pin definition for left motor
const unsigned int IN3 = 6;
const unsigned int IN4 = 7;
const unsigned int EN34 = 9;

// Create left motor instance
L298N left_motor(EN34, IN3, IN4);
L298N::Direction left_motor_direction;
uint8_t left_pwm;

// Create right motor instance
L298N right_motor(EN12, IN1, IN2);
L298N::Direction right_motor_direction;
uint8_t right_pwm;

//Defining Encoder Pins
// Left Motor Encoder
const int ML_A = 2;
const int ML_B = 3;

//Right Motor Encoder
const int MR_A = 18;
const int MR_B = 19;

//Creating a Nodehandle object
ros::NodeHandle nh;

//Position Publisher Messages
std_msgs::Int16 lwheelMsg;
ros::Publisher lwheelPub("lwheel", &lwheelMsg);

std_msgs::Int16 rwheelMsg;
ros::Publisher rwheelPub("rwheel", &rwheelMsg);

rx1_imu::imu_raw imuMsg;
ros::Publisher imuPub("imu_raw", &imuMsg);


//callback function when a control message is received
void rover_control(const geometry_msgs::Twist& ctrl_msg)
{ 
  
  double left_motor_pid = ctrl_msg.linear.x;
  double right_motor_pid = ctrl_msg.angular.z;

  int32_t left_motor_pwm = constrain(left_motor_pid, -255, 255);
  int32_t right_motor_pwm = constrain(right_motor_pid, -255, 255);
 
  // Serial1.println(left_motor_pwm);
  // Serial1.println(right_motor_pwm);

  if (left_motor_pwm < -1){
    left_motor.backward();
  }
  else if(left_motor_pwm>1){
    left_motor.forward();
  }
  else{
    left_motor.stop();
  }

  if (right_motor_pwm < -1){
    right_motor.backward();
  }
  else if(right_motor_pwm>1){
    right_motor.forward();
  }
  else{
    right_motor.stop();
  }

  left_motor.setSpeed(abs(left_motor_pwm));
  right_motor.setSpeed(abs(right_motor_pwm));
  
}

// Subscribing to "rover_control" ROS master serves the commands though this topic
// We will use this message to deliver left and right motor PWMs 
ros::Subscriber<geometry_msgs::Twist> controlSub("/pid_control", rover_control );

// It is best to use volatile variables for the updates in interrupt service routines
volatile long lwheel = 0;
volatile long rwheel = 0;

long lastLwheel = 0;
long lastRwheel = 0;

//for loop timing
unsigned long last_time = 0;

// These are ISRs for reading the encoder ticks
void leftAChange() {
  if (digitalRead(ML_A) == digitalRead(ML_B)) {
    --lwheel;
  } else {
    ++lwheel;
  }
}

void rightAChange() {
  if (digitalRead(MR_A) != digitalRead(MR_B)) {
    --rwheel;
  } else {
    ++rwheel;
  }
}

void setup() {

  Serial2.begin(115200);

  Serial2.println("Serial communication started successfully");
  status = IMU.begin();
  delay(1000);

  Serial2.print("Status of IMU is:");
  Serial2.println((String)status);
  

  left_pwm = 0;
  right_pwm = 0;
  //Stop Motors
  left_motor.setSpeed(left_pwm);
  right_motor.setSpeed(right_pwm);
  
  pinMode(ML_A,INPUT);
  pinMode(ML_B,INPUT);
  pinMode(MR_A,INPUT);
  pinMode(MR_B,INPUT);
  
  //Interrupts are the best way to read encoder values
  attachInterrupt(digitalPinToInterrupt(ML_A), leftAChange, RISING);
  attachInterrupt(digitalPinToInterrupt(MR_A), rightAChange, RISING);
  nh.initNode();
  
  nh.subscribe(controlSub);
  
  nh.advertise(lwheelPub);
  nh.advertise(rwheelPub);
  nh.advertise(imuPub);
  
  //This is a trick from
  //https://answers.ros.org/question/232355/tried-to-publish-before-configured-topic-id-125/
  //without the following it causes the "tried to publish before configured error" in ROS
  nh.negotiateTopics();
  
  last_time = millis();
  
}

void loop() {
  
  unsigned long current_time = millis();

  if((current_time-last_time) >= LOOPTIME){
  
  noInterrupts();
  long curLwheel = lwheel;
  long curRwheel = rwheel;
  interrupts();
  
  IMU.readSensor();
  
  // Serial2.print("Accel x:");
  // Serial2.println((String)IMU.getAccelX_mss());

  imuMsg.accel.x=IMU.getAccelX_mss();
  imuMsg.accel.y=IMU.getAccelY_mss();
  imuMsg.accel.z=IMU.getAccelZ_mss();

  imuMsg.gyro.x=IMU.getGyroX_rads();
  imuMsg.gyro.y=IMU.getGyroY_rads();
  imuMsg.gyro.z=IMU.getGyroZ_rads();
  
  imuMsg.mag.x=IMU.getMagX_uT();
  imuMsg.mag.y=IMU.getMagY_uT();
  imuMsg.mag.z=IMU.getMagZ_uT();

  //Publish the IMU message
  imuPub.publish(&imuMsg);

  // Publish the wheel ticks
  lwheelMsg.data = (int) curLwheel;
  rwheelMsg.data = (int) curRwheel;
  lwheelPub.publish(&lwheelMsg);
  rwheelPub.publish(&rwheelMsg);

  
  lastLwheel = curLwheel;
  lastRwheel = curRwheel;
  last_time = current_time;

  // Serial2.print((String)imuMsg.accel.x);
  // Serial2.print(",");
  // Serial2.print((String)imuMsg.accel.y);
  // Serial2.print(",");
  // Serial2.println((String)imuMsg.accel.z);
  }
  
  
  nh.spinOnce();
  

}

