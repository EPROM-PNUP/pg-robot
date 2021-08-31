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
#include <geometry_msgs/Twist.h>
#include <pg_hardware/motor_driver.hpp>
#include <pg_msgs/MotorCommand.h>
#include <pg_msgs/WheelVelocityCommand.h>

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
		ros::param::get("motor_driver/max_angular_velocity",
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
		}
	}

	void run() {
		ros::Rate rate(20);

		while (ros::ok()) {
			for (auto &i : motor_driver_) {
				i.calcMotorPWM();
			}

			for (uint8_t i = 0; i < 3; i++) {
				motor_pwm_msg_.data[i] = motor_driver_[i].getMotorPWM();
			}

			ROS_DEBUG_NAMED("motor_pwm",
				"PWM:=   %3i   %3i   %3i",
				motor_driver_[0].getMotorPWM(),
				motor_driver_[1].getMotorPWM(),
				motor_driver_[2].getMotorPWM()
			);

			ROS_DEBUG_NAMED("motor_vel",
				"VEL:=   %6.3lf   %6.3lf   %6.3lf",
				motor_driver_[0].getVelocity(),
				motor_driver_[1].getVelocity(),
				motor_driver_[2].getVelocity()
			);

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

