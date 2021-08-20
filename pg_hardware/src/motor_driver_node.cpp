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
#include <geometry_msgs/Twist.h>
#include <pg_hardware/motor_driver.hpp>
#include <pg_msgs/MotorCommand.h>
#include <pg_msgs/WheelVelocityCommand.h>


//////////////////////////
// MOTOR DRIVER WRAPPER //
//////////////////////////

class MotorDriverWrapper {
	private:
	ros::Subscriber refrence_velocity_sub_;
	ros::Publisher motor_pwm_pub_;

	pg_msgs::MotorCommand motor_pwm_msg_;

	std::vector<pg_ns::MotorDriver> motor_driver_;

	public:
	MotorDriverWrapper(ros::NodeHandle &nh) {
		ROS_INFO("Running /motor_driver");
		
		motor_driver_.assign(3, pg_ns::MotorDriver());

		double max_velocity;
		ros::param::get("motor_driver/max_velocity",
			max_velocity);

		// Set maximum velocity for each motor
		for (auto &i : motor_driver_) {
			i.setMaxVelocity(max_velocity);
		}

		refrence_velocity_sub_ = nh.subscribe("wheel_refrence_velocity",
			10, &MotorDriverWrapper::refrenceVelocityCallback, this);

		motor_pwm_pub_ = nh.advertise<pg_msgs::MotorCommand>("motor_pwm", 10);
	}

	void refrenceVelocityCallback(const pg_msgs::WheelVelocityCommand &msg) {
		for (uint8_t i = 0; i < 3; i++) {
			motor_driver_[i].setVelocity(msg.data[i]);
			motor_driver_[i].calcMotorPWM();
		}
	}

	void run() {
		ros::Rate rate(50);

		while (ros::ok()) {
			for (uint8_t i = 0; i < 3; i++) {
				motor_pwm_msg_.data[i] = motor_driver_[i].getMotorPWM();
			}

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

