/*

IMU_Steering_Bot
A wheeled robot based on Arduino and the Pololu MinIMU-9

Copyright (c) 2012 by Doug Sievers (dwsievers@gmail.com)

IMU_Steering_Bot is free software: you can redistribute it and/or modify it
under the terms of the GNU Lesser General Public License as published by the
Free Software Foundation, either version 3 of the License, or (at your option)
any later version.

IMU_Steering_Bot is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for
more details.

You should have received a copy of the GNU Lesser General Public License along
with IMU_Steering_Robot. If not, see <http://www.gnu.org/licenses/>.

*/

// tested with Arduino Uno with ATmega328

#include <Streaming.h>
#include <Messenger.h>
#include <Servo.h>

Servo leftMotor;
Servo rightMotor;

int leftMotorPin = 9;
int rightMotorPin = 10;

float angle_target;
float angle_current;
float angle_delta;

boolean clockwise;
boolean turning;
boolean enabled;

void messageReady()
{
  if ( message.checkString("N") ) {
    angle_target = 0.0f;
    Serial << "North: " << angle_target << endl;
  } 
  else if ( message.checkString("E") ) {
    angle_target = 90.0f;
    Serial << "East:" << angle_target << endl;
  } 
  else if ( message.checkString("S") ) {
    angle_target = 180.0f;
    Serial << "South: " << angle_target << endl;
  } 
  else if ( message.checkString("W") ) {
    angle_target = -90.0f;
    Serial << "West: " << angle_target << endl;
  } 
  else if ( message.checkString("A") ) {
    angle_target = (float)message.readInt();
    Serial << "Angle: " << angle_target << endl;
  }
  else if ( message.checkString("1") ) {
    enabled = true;
    Serial << "Motion Enabled" << endl;
  }
  else if ( message.checkString("0") ) {
    enabled = false;
    Serial << "Motion Disabled" << endl;
  }
}


void setupSteering()
{ 
  // Initialize variables for steering
  angle_current = 0.0f;
  angle_target = angle_current;
  turning = false;
  clockwise = true;
  enabled = false;
  
  // Attach servos
  leftMotor.attach(leftMotorPin);
  leftMotor.write(90);
  rightMotor.attach(rightMotorPin);
  rightMotor.write(90);
  
  digitalWrite(STATUS_LED,HIGH);
  
  // Attach message handler and output instructions
  message.attach(messageReady);
  Serial << "IMU_Steering_Bot" << endl;
  Serial << "Enter digit 1 to enable motion, or 0 to disable." << endl;
  Serial << "Enter direction N, E, W, or S, or A <angle> to turn a specific direction." << endl;
  
}

void steeringCalc() {

  // Convert yaw to degrees and determine remaining angle to turn
  angle_current = ToDeg(yaw);
  angle_delta = angle_target - angle_current;
  
  // Make sure that turn angle is 180 degrees or less, since AHRS code uses -180 < angle < 180
  if ( angle_delta > 180.0f ) {
    angle_delta -= 360.0f;
  } else if ( angle_delta < -180.0f ) {
    angle_delta += 360.0f;
  }
  
  // Determine if turn is CW or CCW and if we should turn.
  // To handle noise, stops when angle difference is < 0.5 degree.
  clockwise = (angle_delta >= 0.0f);  
  if ( abs(angle_delta) < 0.5f ) {
    turning = false;
  } else {
    turning = true;
  }
  
}

void steeringControl() {

  // Motor direction and speed are selected based on turning and clockwise flags.
  if ( turning && enabled) {
    if (clockwise) {
      leftMotor.write(93);
      rightMotor.write(93); //make same since opposite sign
    } else {
      leftMotor.write(86);
      rightMotor.write(86); //make same since opposite sign
    }
  } else {
    leftMotor.write(90);
    rightMotor.write(90);
  }
   
}
