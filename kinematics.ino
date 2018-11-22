/*
  Copyright (c) 2016, Juan Jimeno
  Source: http://research.ijcaonline.org/volume113/number3/pxc3901586.pdf
  All rights reserved.
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
   Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
   Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.
   Neither the name of  nor the names of its contributors may be used to
  endorse or promote products derived from this software without specific
  prior written permission.
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORTPPIPI (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/

#include "Kinematics.h"

#define MOTOR_MAX_RPM 90        // motor's maximum rpm
#define WHEEL_DIAMETER 0.2      // robot's wheel diameter expressed in meters
#define FR_WHEEL_DISTANCE 0.6   // distance between front wheel and rear wheel
#define LR_WHEEL_DISTANCE 0.5   // distance between left wheel and right wheel
#define PWM_BITS 8              // microcontroller's PWM pin resolution. Arduino Uno/Mega Teensy is using 8 bits(0-255)

Kinematics kinematics(MOTOR_MAX_RPM, WHEEL_DIAMETER, FR_WHEEL_DISTANCE, LR_WHEEL_DISTANCE, PWM_BITS);

void setup() 
{
    Serial.begin(9600);
}

void loop() 
{
    Kinematics::output rpm;

    //simulated required velocities
    float linear_vel_x = 1;  // 1 m/s
    float linear_vel_y = 0;  // 0 m/s
    float angular_vel_z = 1; // 1 m/s

    //given the required velocities for the robot, you can calculate the rpm required for each motor
    rpm = kinematics.getRPM(linear_vel_x, linear_vel_y, angular_vel_z);

    Serial.print(" FRONT LEFT MOTOR: ");
    // Assuming you have an encoder for each wheel, you can pass this RPM value to a PID controller 
    // as a setpoint and your encoder data as a feedback.
    Serial.print(rpm.motor1);

    Serial.print(" FRONT RIGHT MOTOR: ");
    Serial.print(rpm.motor2);

    Serial.print(" REAR LEFT MOTOR: ");
    Serial.print(rpm.motor3);

    Serial.print(" REAR RIGHT MOTOR: ");
    Serial.println(rpm.motor4);

    delay(5000);


    // This is a simulated feedback from each motor. We'll just pass the calculated rpm above for demo's sake.
    // In a live robot, these should be replaced with real RPM values derived from encoder.
    int motor1_feedback = rpm.motor1; //in rpm
    int motor2_feedback = rpm.motor2; //in rpm
    int motor3_feedback = rpm.motor3; //in rpm
    int motor4_feedback = rpm.motor4; //in rpm

    Kinematics::velocities vel;

    // Now given the RPM from each wheel, you can calculate the linear and angular velocity of the robot.
    // This is useful if you want to create an odometry data (dead reckoning)
    vel = kinematics.getVelocities(motor1_feedback, motor2_feedback, motor3_feedback, motor4_feedback);
    Serial.print(" VEL X: ");
    Serial.print(vel.linear_x, 4);

    Serial.print(" VEL_Y: ");
    Serial.print(vel.linear_y, 4);

    Serial.print(" ANGULAR_Z: ");
    Serial.println(vel.angular_z, 4);
    Serial.println("");
}
