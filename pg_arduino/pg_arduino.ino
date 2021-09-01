// Copyright (c) 2021 EPROM PNUP
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom
// the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
//
// Author: Wahyu Mahardika


/////////////////////
// INCLUDE HEADERS //
/////////////////////

#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>

#include "src/motor/motor.hpp"
#include "src/rotary_encoder/rotary_encoder.hpp"
#include "src/cmps12/cmps12.hpp"


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
pg_ns::CMPS12 imu;


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

// IMU Publisher & msg
std_msgs::Int16MultiArray imu_data_raw_msg;
ros::Publisher imu_pub("imu_raw", &imu_data_raw_msg);


//////////////////////
// GLOBAL VARIABLES //
//////////////////////

static long pulse_counts_millis_track = 0;
static long imu_millis_track = 0;
static long current_millis = 0;

static pg_ns::ImuDataRaw imu_data_raw;


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

	// Initialize imu sensor object
	imu.init();
	
	// Initialize ROS node and baudrate
	nh.getHardware()->setBaud(115200);
	nh.initNode();

	// Initialize subscribers 
	nh.subscribe(motor_1_pwm_sub);
	nh.subscribe(motor_2_pwm_sub);
	nh.subscribe(motor_3_pwm_sub);

	// Initialize publishers
	nh.advertise(encoder_1_pulse_pub);
	nh.advertise(encoder_2_pulse_pub);
	nh.advertise(encoder_3_pulse_pub);
	nh.advertise(imu_pub);

	// Initialize imu msg data length
	int16_t temp_9[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
	imu_data_raw_msg.data_length = 9;
	imu_data_raw_msg.data = temp_9;
}


///////////////////////////
// ARDUINO LOOP FUNCTION //
///////////////////////////

void loop() {
	nh.spinOnce();

	current_millis = millis();
	
	// Publish encoders pulse counts every 20 ms
	if(current_millis - pulse_counts_millis_track > 20) {
		pulse_counts_millis_track = current_millis;

		encoder_1_pulse_pub.publish(&encoder_1_pulse);
		encoder_2_pulse_pub.publish(&encoder_2_pulse);
		encoder_3_pulse_pub.publish(&encoder_3_pulse);
	}

	// Publish imu sensor raw data every 20 ms
	if(current_millis - imu_millis_track > 20) {
		imu_millis_track = current_millis;

		imu_data_raw = imu.getRawData();

		imu_data_raw_msg.data[0] = imu_data_raw.bearing_;
		imu_data_raw_msg.data[1] = imu_data_raw.pitch_;
		imu_data_raw_msg.data[2] = imu_data_raw.roll_;

		imu_data_raw_msg.data[3] = imu_data_raw.gyro_x_;
		imu_data_raw_msg.data[4] = imu_data_raw.gyro_y_;
		imu_data_raw_msg.data[5] = imu_data_raw.gyro_z_;

		imu_data_raw_msg.data[6] = imu_data_raw.accel_x_;
		imu_data_raw_msg.data[7] = imu_data_raw.accel_y_;
		imu_data_raw_msg.data[8] = imu_data_raw.accel_z_;

		imu_pub.publish(&imu_data_raw_msg);
	}
}


/////////////////////////////////
// ENCODERS FUNCTION INTERRUPT //
/////////////////////////////////

// Wheel A interrupt
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

// Wheel C interrupt
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


