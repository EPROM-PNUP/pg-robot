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

#include <ros/ros.h>
#include <ros/console.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>

#include "pg_hardware/motor_driver.hpp"

class MotorDriverWrapper {
	private:
	ros::Subscriber controller_output_sub_;
	ros::Subscriber encoder_pulse_sub_;

	ros::Publisher motor_pwm_pub_;
	ros::Publisher motor_state_pub_;

	std_msgs::Int16 motor_pwm_msg_;
	std_msgs::Float64 motor_state_msg_;

	pg_ns::MotorDriver motor_driver_;

	double node_frequency_;

	public:
	MotorDriverWrapper(ros::NodeHandle &nh) {
		ROS_INFO("Running motor_driver");
		
		// Load parameters from parameter server.
		if (loadParameters()) {
			ROS_INFO("Successfully loaded all parameters");
		}
		else {
			ROS_WARN("Unable to load all parameters, using defaults");
		}

		controller_output_sub_ = nh.subscribe("controller_output",
			10, &MotorDriverWrapper::controllerOutputCallback, this);

		encoder_pulse_sub_ = nh.subscribe("encoder_pulse",
			10, &MotorDriverWrapper::encoderPulseCallback, this);

		motor_pwm_pub_ = nh.advertise<std_msgs::Int16>("motor_pwm", 10);
		motor_state_pub_ = nh.advertise<std_msgs::Float64>("motor_state", 10);
	}

	// LOAD PARAMETERS FUNCTION.
	// Load necessary parameters from parameter server
	// (set to defaults if failed).
	bool loadParameters() {
		if (ros::param::has("~max_pwm")) {
			int max_pwm;
			ros::param::get("~max_pwm", max_pwm);
			motor_driver_.setMaxPWM(max_pwm);
			ROS_INFO("Loaded max_pwm");
		}
		else {
			ROS_WARN("Failed to load max_pwm parameter");
			motor_driver_.setMaxPWM(255);
			return false;
		}

		if (ros::param::has("~frequency")) {
			ros::param::get("~frequency", node_frequency_);
			ROS_INFO("Loaded node frequency/rate");
		}
		else {
			ROS_WARN("Failed to load node frequency");
			node_frequency_ = 50;
			return false;
		}

		return true;
	}

	// Callback function for wheel velocity controller.
	void controllerOutputCallback(const std_msgs::Float64 &msg) {
		motor_driver_.setPWM(msg.data);
	}

	// Callback function for wheel encoder pulse.
	void encoderPulseCallback(const std_msgs::Int16 &msg) {
		motor_driver_.setEncoderPulse(msg.data);
	}

	// RUN FUNCTION
	// Continuously running, publishing controlled
	// pwm signal to motors every 20 ms (50 Hz).
	void run() {
		ros::Rate rate(node_frequency_);

		while (ros::ok()) {
			motor_state_msg_.data = motor_driver_.getState();
			motor_state_pub_.publish(motor_state_msg_);

			motor_pwm_msg_.data = motor_driver_.getPWM();
			motor_pwm_pub_.publish(motor_pwm_msg_);

			ros::spinOnce();
			rate.sleep();
		}
	}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "motor_driver");
	ros::NodeHandle nh;

	MotorDriverWrapper motor_driver_wrapper(nh);
	motor_driver_wrapper.run();

	return 0;
}

