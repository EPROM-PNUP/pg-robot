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

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <pg_hardware/motor_driver.hpp>

class MotorDriverWrapper {
	private:
	ros::Subscriber controller_output_sub_;
	rosLLSubscriber encoder_pulse_sub_;

	ros::Publisher motor_pwm_pub_;
	ros::Publisher motor_state_pub_;

	std_msgs::Int16 motor_pwm_msg_;
	std_msgs::Float64 motor_state_msg_;

	pg_ns::MotorDriver motor_driver_;

	public:
	MotorDriverWrapper(ros::NodeHandle &nh) {
		ROS_INFO("Running /motor_driver");
		
		int max_pwm;
		ros::param::get("motor_driver/max_pwm", max_pwm);

		// Set maximum pwm signal for each motor
		motor_driver_.setMaxPWM(max_pwm);

		controller_output_sub_ = nh.subscribe("controller_output",
			10, &MotorDriverWrapper::controllerOutputCallback, this);

		encoder_pulse_sub_ = nh.subscribe("encoder_pulse",
			10, &MotorDriverWrapper::encoderPulseCallback, this);

		motor_pwm_pub_ = nh.advertise<std_msgs::Int16>("motor_pwm", 10);
	}

	void controllerOutputCallback(const std_msgs::Int16 &msg) {
		motor_driver_.setPWM(msg.data);
	}

	void encoderPulseCallback(const std_msgs::Int16 &msg) {
		motor_driver_.setEncoderPulse(msg.data);
	}

	void run() {
		ros::Rate rate(10);

		while (ros::ok()) {
			motor_state_msg_.data = motor_driver_.getState();
			motor_state_pub_.publish(motor_state_msg_);

			motor_pwm_msg_.data = motor_driver_.getPWM();
			motor_pwm_pub_.publish(pwm_msg_);

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

