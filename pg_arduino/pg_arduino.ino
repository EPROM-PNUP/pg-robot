// Copyright (c) 2021, EPROM PNUP
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
// 
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Author: Wahyu Mahardika


/////////////////////
// INCLUDE HEADERS //
/////////////////////

#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>

#include "src/motor/motor.hpp"
#include "src/rotary_encoder/rotary_encoder.hpp"
// #include "src/cmps12/cmps12.hpp"
#include "src/dribbler/dribbler.hpp"
#include "src/proximity/proximity.hpp"


////////////
// MACROS //
////////////

#define MOTOR_1_PIN_A 4
#define MOTOR_1_PIN_B 5
#define MOTOR_2_PIN_A 8
#define MOTOR_2_PIN_B 9
#define MOTOR_3_PIN_A 6
#define MOTOR_3_PIN_B 7

#define ENCODER_1_PIN_C1 2
#define ENCODER_1_PIN_C2 3
#define ENCODER_2_PIN_C1 18
#define ENCODER_2_PIN_C2 19
#define ENCODER_3_PIN_C1 20
#define ENCODER_3_PIN_C2 21

#define DRIBBLER_LEFT_PIN_A 10
#define DRIBBLER_LEFT_PIN_B 11
#define DRIBBLER_RIGHT_PIN_A 12
#define DRIBBLER_RIGHT_PIN_B 13

#define PROXIMITY_PIN 23


////////////////////////////
// OBJECTS INITIALIZATION //
////////////////////////////

// Motors 1, 2, 3
pg_ns::Motor motor_1(MOTOR_1_PIN_A, MOTOR_1_PIN_B);
pg_ns::Motor motor_2(MOTOR_2_PIN_A, MOTOR_2_PIN_B);
pg_ns::Motor motor_3(MOTOR_3_PIN_A, MOTOR_3_PIN_B);
// Encoders 1, 2, 3
pg_ns::RotaryEncoder re_1(ENCODER_1_PIN_C1, ENCODER_1_PIN_C2);
pg_ns::RotaryEncoder re_2(ENCODER_2_PIN_C1, ENCODER_2_PIN_C2);
pg_ns::RotaryEncoder re_3(ENCODER_3_PIN_C1, ENCODER_3_PIN_C2);
// IMU Sensor
// pg_ns::CMPS12 imu;
// pg_ns::ImuDataRaw imu_data_raw;
// Dribbler Left & Right
pg_ns::Dribbler dribbler_left(DRIBBLER_LEFT_PIN_A, DRIBBLER_LEFT_PIN_B);
pg_ns::Dribbler dribbler_right(DRIBBLER_RIGHT_PIN_A, DRIBBLER_RIGHT_PIN_B);
// Proximity Sensor
pg_ns::Proximity proxi(PROXIMITY_PIN);


////////////////////////
// CALLBACK FUNCTIONS //
////////////////////////

void motor1Callback(const std_msgs::Int16 &msg) {
	motor_1.move(msg.data);
}

void motor2Callback(const std_msgs::Int16 &msg) {
	motor_2.move(msg.data);
}

void motor3Callback(const std_msgs::Int16 &msg) {
	motor_3.move(msg.data);
}

void dribbleLeftCallback(const std_msgs::Bool &msg) {
	if (msg.data) {
		dribbler_left.dribble();
	}
	else {
		dribbler_right.stop();
	}
}

void dribbleRightCallback(const std_msgs::Bool &msg) {
	if (msg.data) {
		dribbler_right.dribble();
	}
	else {
		dribbler_right.stop();
	}
}


////////////////////////////
// ROS NODE HANDLE & MSGS //
////////////////////////////

ros::NodeHandle nh;

// Motor PWM Subscriber
ros::Subscriber<std_msgs::Int16> motor_1_pwm_sub(
	"wheel_1/motor_pwm", &motor1Callback);

ros::Subscriber<std_msgs::Int16> motor_2_pwm_sub(
	"wheel_2/motor_pwm", &motor2Callback);

ros::Subscriber<std_msgs::Int16> motor_3_pwm_sub(
	"wheel_3/motor_pwm", &motor3Callback);

// Dribbler Command Subscriber
ros::Subscriber<std_msgs::Bool> dribble_cmd_left_sub(
	"dribbler/left/cmd_dribble", &dribbleLeftCallback);

ros::Subscriber<std_msgs::Bool> dribble_cmd_right_sub(
	"dribbler/right/cmd_dribble", &dribbleRightCallback);

// Encoders Publisher & msg
std_msgs::Int16 encoder_1_pulse;
ros::Publisher encoder_1_pulse_pub(
	"wheel_1/encoder_pulse",
	&encoder_1_pulse
	);

std_msgs::Int16 encoder_2_pulse;
ros::Publisher encoder_2_pulse_pub(
	"wheel_2/encoder_pulse",
	&encoder_2_pulse
	);

std_msgs::Int16 encoder_3_pulse;
ros::Publisher encoder_3_pulse_pub(
	"wheel_3/encoder_pulse",
	&encoder_3_pulse
	);

// // CMPS12 Orientation Publisher & msg
// std_msgs::Int16MultiArray cmps12_orientation_msg;
// ros::Publisher cmps12_orientation_pub(
//  	"imu/orientation",
//  	&cmps12_orientation_msg
//  	);

// // Magnetometer Publisher & msg
// std_msgs::Int16MultiArray magnetometer_msg;
// ros::Publisher magnetometer_pub("imu/magnetometer", &magnetometer_msg);

// // Accelerometer Publisher & msg
// std_msgs::Int16MultiArray accelerometer_msg;
// ros::Publisher accelerometer_pub("imu/accelerometer", &accelerometer_msg);

// // Gyroscope Publisher & msg
// std_msgs::Int16MultiArray gyroscope_msg;
// ros::Publisher gyroscope_pub("imu/gyroscope", &gyroscope_msg);

// Proximity Sensor Publisher & msg
std_msgs::Bool ball_in_range_msg;
ros::Publisher proximity_pub("dribbler/ball_in_range", &ball_in_range_msg);


//////////////////////
// GLOBAL VARIABLES //
//////////////////////

long current_millis = 0;
long previous_millis = 0;

uint16_t uint_temp_3[3] = {0, 0, 0};
int16_t int_temp_3[3] = {0, 0, 0};


////////////////////////////
// ARDUINO SETUP FUNCTION //
////////////////////////////

void setup() {
	// Initialize encoder objects
	re_1.init();
	re_2.init();
	re_3.init();

	// Attach interrupt on every encoder C1 pins
	attachInterrupt(
		digitalPinToInterrupt(re_1.getC1Pin()), 
		pg_ns::pulseInterruptA, 
		RISING
		);

	attachInterrupt(
		digitalPinToInterrupt(re_2.getC1Pin()), 
		pg_ns::pulseInterruptB, 
		RISING
		);

	attachInterrupt(
		digitalPinToInterrupt(re_3.getC1Pin()), 
		pg_ns::pulseInterruptC, 
		RISING
		);

	// Initialize dc motor objects
	motor_1.init();
	motor_2.init();
	motor_3.init();

	// // Initialize imu sensor object
	// imu.init();

	// Initialize dribbler motor object
	dribbler_left.init();
	dribbler_right.init();

	// Initialize proximity sensor
	proxi.init();
	
	// Initialize ROS node and baudrate
	nh.getHardware()->setBaud(115200);
	nh.initNode();

	// Initialize subscribers 
	nh.subscribe(motor_1_pwm_sub);
	nh.subscribe(motor_2_pwm_sub);
	nh.subscribe(motor_3_pwm_sub);
	// nh.subscribe(dribble_cmd_left_sub);
	// nh.subscribe(dribble_cmd_right_sub);

	// Initialize publishers
	nh.advertise(encoder_1_pulse_pub);
	nh.advertise(encoder_2_pulse_pub);
	nh.advertise(encoder_3_pulse_pub);
	// nh.advertise(cmps12_orientation_pub);
	// nh.advertise(magnetometer_pub);
	// nh.advertise(accelerometer_pub);
	// nh.advertise(gyroscope_pub);
	nh.advertise(proximity_pub);


	// // Initialize cmps12 orientation msg data length
	// cmps12_orientation_msg.data_length = 3;
	// cmps12_orientation_msg.data = uint_temp_3;

	// // Initialize magnetometer msg data length
	// magnetometer_msg.data_length = 3;
	// magnetometer_msg.data = int_temp_3;

	// // Initialize accelerometer msg data length
	// accelerometer_msg.data_length = 3;
	// accelerometer_msg.data = int_temp_3;

	// // Initialize gyroscope msg data length
	// gyroscope_msg.data_length = 3;
	// gyroscope_msg.data = int_temp_3;
}


///////////////////////////
// ARDUINO LOOP FUNCTION //
///////////////////////////

void loop() {
	nh.spinOnce();

	current_millis = millis();

	if ((current_millis - previous_millis) > 20) {
		previous_millis = current_millis;

		// Publish encoders pulse counts every 20 ms
		encoder_1_pulse_pub.publish(&encoder_1_pulse);
		encoder_2_pulse_pub.publish(&encoder_2_pulse);
		encoder_3_pulse_pub.publish(&encoder_3_pulse);

		// // Publish IMU raw data
		// imu_data_raw = imu.getRawData();

		// cmps12_orientation_msg.data[0] = imu_data_raw.bearing_;
		// cmps12_orientation_msg.data[1] = imu_data_raw.pitch_;
		// cmps12_orientation_msg.data[2] = imu_data_raw.roll_;

		// magnetometer_msg.data[0] = imu_data_raw.mag_x_;
		// magnetometer_msg.data[1] = imu_data_raw.mag_y_;
		// magnetometer_msg.data[2] = imu_data_raw.mag_z_;

		// accelerometer_msg.data[0] = imu_data_raw.accel_x_;
		// accelerometer_msg.data[1] = imu_data_raw.accel_y_;
		// accelerometer_msg.data[2] = imu_data_raw.accel_z_;

		// gyroscope_msg.data[0] = imu_data_raw.gyro_x_;
		// gyroscope_msg.data[1] = imu_data_raw.gyro_y_;
		// gyroscope_msg.data[2] = imu_data_raw.gyro_z_;

		// cmps12_orientation_pub.publish(&cmps12_orientation_msg);
		// magnetometer_pub.publish(&magnetometer_msg);
		// accelerometer_pub.publish(&accelerometer_msg);
		// gyroscope_pub.publish(&gyroscope_msg);

		// Publish proxi data
		ball_in_range_msg.data = proxi.ballIsInRange();
		proximity_pub.publish(&ball_in_range_msg);
	}
}


/////////////////////////////////
// ENCODERS FUNCTION INTERRUPT //
/////////////////////////////////

// Wheel 1 encoder interrupt
void pg_ns::pulseInterruptA() {
	if(digitalRead(re_1.en_c2_) == LOW) {
		re_1.direction_cw_ = true;
	}
	else {
		re_1.direction_cw_ = false;
	}

	if(re_1.direction_cw_) {
		if(re_1.pulse_count_ == RotaryEncoder::ENCODER_MAX) {
			re_1.pulse_count_ = RotaryEncoder::ENCODER_MIN;
			encoder_1_pulse.data = re_1.pulse_count_;
		}
		else {
			re_1.pulse_count_++;
			encoder_1_pulse.data = re_1.pulse_count_;
		}
	}
	else {
		if(re_1.pulse_count_ == RotaryEncoder::ENCODER_MIN) {
			re_1.pulse_count_ = RotaryEncoder::ENCODER_MAX;
			encoder_1_pulse.data = re_1.pulse_count_;
		}
		else {
			re_1.pulse_count_--;
			encoder_1_pulse.data = re_1.pulse_count_;
		}
	}
}

// Wheel 2 encoder interrupt
void pg_ns::pulseInterruptB() {
	if(digitalRead(re_2.en_c2_) == LOW) {
		re_2.direction_cw_ = true;
	}
	else {
		re_2.direction_cw_ = false;
	}

	if(re_2.direction_cw_) {
		if(re_2.pulse_count_ == RotaryEncoder::ENCODER_MAX) {
			re_2.pulse_count_ = RotaryEncoder::ENCODER_MIN;
			encoder_2_pulse.data = re_2.pulse_count_;
		}
		else {
			re_2.pulse_count_++;
			encoder_2_pulse.data = re_2.pulse_count_;
		}
	}
	else {
		if(re_2.pulse_count_ == RotaryEncoder::ENCODER_MIN) {
			re_2.pulse_count_ = RotaryEncoder::ENCODER_MAX;
			encoder_2_pulse.data = re_2.pulse_count_;
		}
		else {
			re_2.pulse_count_--;
			encoder_2_pulse.data = re_2.pulse_count_;
		}
	}
}

// Wheel 3 encoder interrupt
void pg_ns::pulseInterruptC() {
	if(digitalRead(re_3.en_c2_) == LOW) {
		re_3.direction_cw_ = true;
	}
	else {
		re_3.direction_cw_ = false;
	}

	if(re_3.direction_cw_) {
		if(re_3.pulse_count_ == RotaryEncoder::ENCODER_MAX) {
			re_3.pulse_count_ = RotaryEncoder::ENCODER_MIN;
			encoder_3_pulse.data = re_3.pulse_count_;
		}
		else {
			re_3.pulse_count_++;
			encoder_3_pulse.data = re_3.pulse_count_;
		}
	}
	else {
		if(re_3.pulse_count_ == RotaryEncoder::ENCODER_MIN) {
			re_3.pulse_count_ = RotaryEncoder::ENCODER_MAX;
			encoder_3_pulse.data = re_3.pulse_count_;
		}
		else {
			re_3.pulse_count_--;
			encoder_3_pulse.data = re_3.pulse_count_;
		}
	}
}


