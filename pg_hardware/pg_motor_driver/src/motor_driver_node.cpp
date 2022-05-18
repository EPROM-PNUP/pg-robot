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

#include "pg_motor_driver/motor_driver.hpp"

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
		
		controller_output_sub_ = nh.subscribe("controller_output",
			10, &MotorDriverWrapper::controllerOutputCallback, this);

		encoder_pulse_sub_ = nh.subscribe("encoder_pulse",
			10, &MotorDriverWrapper::encoderPulseCallback, this);

		motor_pwm_pub_ = nh.advertise<std_msgs::Int16>("motor_pwm", 10);
		motor_state_pub_ = nh.advertise<std_msgs::Float64>("motor_state", 10);
	}

	// Callback function for wheel velocity controller.
	void controllerOutputCallback(const std_msgs::Float64 &msg) {
		motor_driver_.setPWM(msg.data);

		motor_pwm_msg_.data = motor_driver_.getPWM();
		motor_pwm_pub_.publish(motor_pwm_msg_);
	}

	// Callback function for wheel encoder pulse.
	void encoderPulseCallback(const std_msgs::Int16 &msg) {
		motor_driver_.setEncoderPulse(msg.data);

		motor_state_msg_.data = motor_driver_.getState();
		motor_state_pub_.publish(motor_state_msg_);
	}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "motor_driver");
	ros::NodeHandle nh;

	MotorDriverWrapper motor_driver_wrapper(nh);
	ros::spin();

	return 0;
}

