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
#include <std_msgs/Bool.h>

#include "pg_dribbler_driver/dribbler_driver.hpp"

class DribblerDriverWrapper {
	private:
	ros::Subscriber ball_in_range_sub_;
	
#ifdef __aarch64__

	pg_ns::DribblerDriver dribbler_driver_left_;
	pg_ns::DribblerDriver dribbler_driver_right_;

#else

	ros::Publisher cmd_dribble_pub_;
	std_msgs::Bool cmd_dribble_msg_;
	pg_ns::DribblerDriver dribbler_driver_;

#endif

	double node_frequency_;

	public:
	DribblerDriverWrapper(ros::NodeHandle &nh) {
		ROS_INFO("Running dribbler_driver ...");
		
		// Load parameters from parameter server.
		if (loadParameters()) {
			ROS_INFO("Succesfully loaded all parameters");
		}
		else {
			ROS_WARN("Unable to load all parameters, using defaults");
		}

		ball_in_range_sub_ = nh.subscribe("/dribbler/ball_in_range",
			10, &DribblerDriverWrapper::ballInRangeCallback, this);

#ifdef __aarch64__
		dribbler_driver_left_.init();
		dribbler_driver_right_.init();
#else
		cmd_dribble_pub_ = nh.advertise<std_msgs::Bool>("cmd_dribble", 10);
#endif

	}

	// LOAD PARAMETERS FUNCTION.
	// Load necessary parameters from parameter server
	// (set to defaults if failed).
	bool loadParameters() {
		if (ros::param::has("~frequency")) {
			ros::param::get("~frequency", node_frequency_);
			ROS_INFO("Loaded node frequency/rate");
		}
		else {
			ROS_WARN("Failed to load node frequency");
			node_frequency_ = 50;
			return false;
		}

#ifdef __aarch64__
		if (ros::param::has("/gpio/dribbler/left")) {
			int pin_a, pin_b;
			ros::param::get("/gpio/dribbler/left/a", pin_a);
			ros::param::get("/gpio/dribbler/left/b", pin_b);
			dribbler_driver_left_.setupPin(pin_a, pin_b);
			ROS_INFO("Loaded left dribbler pin parameters");
		}
		else {
			ROS_WARN("Failed to load left dribbler parameters");
			dribbler_driver_left_.setupPin(21, 22);
			return false;
		}

		if (ros::param::has("/gpio/dribbler/right")) {
			int pin_a, pin_b;
			ros::param::get("/gpio/dribbler/right/a", pin_a);
			ros::param::get("/gpio/dribbler/right/b", pin_b);
			dribbler_driver_right_.setupPin(pin_a, pin_b);
			ROS_INFO("Loaded right dribbler pin parameters");
		}
		else {
			ROS_WARN("Failed to load right dribbler parameters");
			dribbler_driver_right_.setupPin(23, 24);
			return false;
		}
#endif

		return true;
	}

	// Callback function for ball in range topic.
	void ballInRangeCallback(const std_msgs::Bool &msg) {
#ifdef __aarch64__
		if (msg.data) {
			dribbler_driver_left_.dribble();
			dribbler_driver_right_.dribble();
		}
		else {
			dribbler_driver_left_.stop();
			dribbler_driver_right_.stop();
		}
#else
		if (msg.data) {
			dribbler_driver_.setDribble();
		}
		else {
			dribbler_driver_.stop();
		}
#endif
	}

#ifndef __aarch64__
	// RUN FUNCTION.
	// Continuously running, publishing pwm signal to
	// dribbler motor every 20 ms (50 Hz).
	void run() {
		ros::Rate rate(node_frequency_);

		while (ros::ok()) {
			cmd_dribble_msg_.data = dribbler_driver_.getCommandDribble();
			cmd_dribble_pub_.publish(cmd_dribble_msg_);

			ros::spinOnce();
			rate.sleep();
		}
	}
#endif
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "dribbler_driver");
	ros::NodeHandle nh;
	DribblerDriverWrapper dribbler_driver_wrapper(nh);
#ifdef __aarch64__
	ros::spin();
#else
	dribbler_driver_wrapper.run();
#endif
	return 0;
}
