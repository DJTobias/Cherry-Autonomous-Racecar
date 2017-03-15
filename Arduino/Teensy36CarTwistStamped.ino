/*
  Arduino ROS node for JetsonCar project
  The Arduino controls a TRAXXAS Rally Car
  MIT License
  JetsonHacks (2016)
*/

/*
Modified by Daniel Tobias
Runs on a Teensy 3.6 
Runs a Traxxas Rally 2WD
ESC Tied to pin(10) 
Steering Servo tied to pin(9)
Runs everything using floating point cause Teensy 3.6 has Hardware Floating Point Unit (FPU)
Changed PWM write to writeMicroseconds for precision
Added a smoothing function to throttle to handle stray pulses and behave more like a car
Added Turn Signal LEDs
Kill switch signal from RF reciever tied to pin(38)
Added break/reverse signal on twist.linear.z ESC takes first signal below 1500 microseconds as breaks and second sequential signal as reverse
Changed message from Twist to TwistStamped for recording synchronization 
Pin Out for Teensy 3.6 is in link below
https://forum.pjrc.com/attachment.php?attachmentid=8071&d=1473211335

*/

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include <WProgram.h>
#endif

#include <Servo.h>
#define USB_USBCON
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

ros::NodeHandle  nodeHandle;
// These are general bounds for the steering servo and the
// TRAXXAS Electronic Speed Controller (ESC)
const float minSteering = 1200 ;
const float maxSteering = 1800 ;
const float minThrottle = 0 ;
const float maxThrottle = 1680 ;
const float steeringIncrement = 9.0;
  float escCommand;
  float escThrottle;
  float smoothSteering = 1500;
  float diffGreat;
  float diffLess;

Servo steeringServo;
Servo electronicSpeedController ;  // The ESC on the TRAXXAS works like a Servo

std_msgs::Int32 str_msg;
ros::Publisher chatter("chatter", &str_msg);

// Arduino 'map' funtion for floating point
float fmap (float toMap, float in_min, float in_max, float out_min, float out_max) {
  return (toMap - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void driveCallback ( const geometry_msgs::TwistStamped&  twistMsg )
{

  float steeringAngle = fmap(twistMsg.twist.angular.z, 0.0, 1.0, minSteering, maxSteering) ;
  // The following could be useful for debugging
   //str_msg.data= steeringAngle ;
  // Check to make sure steeringAngle is within car range
  if (steeringAngle < minSteering) {
    steeringAngle = minSteering;
  }
  if (steeringAngle > maxSteering) {
    
    steeringAngle = maxSteering ;
  }

//   float diff = steeringAngle - smoothSteering;     //Future smoothing for steering
//   if (diff >= steeringIncrement){                  //currently solved with 60Hz refresh on callbacks from node
//    smoothSteering += steeringIncrement;            
//   }
//   else if (diff <= -steeringIncrement) {
//    smoothSteering -= steeringIncrement;
//   }
//   else {
//    smoothSteering = steeringAngle;
//   }
   
  steeringServo.writeMicroseconds(steeringAngle) ;
  if(steeringAngle < 1450)
    digitalWrite(29, HIGH - digitalRead(29));  //blinks LED if turning left
  else if(steeringAngle> 1550)
    digitalWrite(30, HIGH - digitalRead(30));  //blinds LED if turning right
  else {
    digitalWrite(30,LOW);
    digitalWrite(29,LOW);
  }

  // ESC forward is between 0.5 and 1.0

  if (twistMsg.twist.linear.x >= 0.5) {
    escCommand = (float)fmap(twistMsg.twist.linear.x, 0.5, 1.0, 1500.0, maxThrottle) ;
  } else {
    //escCommand = (int)fmap(twistMsg.twist.linear.x, 0.0, 1.0, 0.0, 1780.0) ;
    escCommand = (float)fmap(twistMsg.twist.linear.x, 0.0, 0.5, 0.0, 1500.0) ;
  }
  // Check to make sure throttle command is within bounds
  if (escCommand < minThrottle) {
    escCommand = minThrottle;
  }
  if (escCommand > maxThrottle) {
    escCommand = maxThrottle ;
  }
  // The following could be useful for debugging
   //str_msg.data= escCommand ;
   //chatter.publish(&str_msg);
    if(digitalRead(38)){          //reads signal from RF Reciever. Will set throttle to neutral if signal is high
       delay(5);
       if(digitalRead(38)){
       electronicSpeedController.writeMicroseconds(1500) ;
       }
    }
    else if(twistMsg.twist.linear.z == 1.0) {
      electronicSpeedController.writeMicroseconds(1400); //brakes
    }
    else{
      escThrottle = escThrottle + 0.05*(escCommand-escThrottle);   //Exponential Smoothing ( ͡° ͜ʖ﻿ ͡°) for throttle
      electronicSpeedController.writeMicroseconds(escThrottle) ;
      digitalWrite(13, HIGH - digitalRead(13)); //toggle led
    }


  
}

ros::Subscriber<geometry_msgs::TwistStamped> driveSubscriber("/cmd_vel", &driveCallback) ;

void setup() {
  pinMode(38, INPUT);
  pinMode(30, OUTPUT);
  pinMode(29, OUTPUT);
  pinMode(13, OUTPUT);
  Serial.begin(57600) ;
  nodeHandle.initNode();
  // This can be useful for debugging purposes
  nodeHandle.advertise(chatter);
  // Subscribe to the steering and throttle messages
  nodeHandle.subscribe(driveSubscriber) ;
  // Attach the servos to actual pins
  steeringServo.attach(9); // Steering servo is attached to pin 9
  electronicSpeedController.attach(10); // ESC is on pin 10
  // Initialize Steering and ESC setting
  // Steering centered is 90, throttle at neutral is 90
  steeringServo.writeMicroseconds(1500) ;
                                     
  //digitalWrite(13, HIGH - digitalRead(13));    
  //electronicSpeedController.writeMicroseconds(1700);   //ESC Activation sequence
  //delay(1000);
  //electronicSpeedController.writeMicroseconds(1300);
  //delay(1000);
  //electronicSpeedController.writeMicroseconds(1500);
  //delay(1000);
  
}

void loop() 
{
  nodeHandle.spinOnce();      //refreshes nodes
  delay(1);
}
