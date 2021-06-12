# RX1 : A ROS based robot

This package implements the following for RX1 rover:
1. PID controller
2. Odometry with Encoder and IMU.

The low level i/o is handled by an Arduino Mega 2560 over rosserial.
The rover is controlled using a joystick.

The ROS code does three things:  
1. Use gamepad to generate a twist message on /cmd_vel topic.  
2. Implements a simple PID controller which reads the wheel encoder ticks and publishes the computed control effort as a twist message to the /pid_control topic.  
3. Calculates odomentry using wheel encoder ticks and publishes it under /odom topic.  

The Arduino code can be found in the /firmware folder. Once you open that folder separately in VSCode, you will be recommended to install PlatformIO, if its not already installed. PIO will take care of the rest of the dependencies for the arduino code. 

The arduino code implements the following:
1. Reads the encoder values and publishes them to /lwheel and /rwheel topic. 
2. It also receives the control message on /pid_control from Raspberry Pi for PID control.
3. Reads a 9DOF IMU (MPU9250) over I2C and publishes it on the topic /imu_raw using a custom message.