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
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int16.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>

#include "pg_localization/odometry.hpp"

using std::vector;

class OdometryWrapper {
	private:
	ros::Subscriber encoder_1_pulse_sub_;
	ros::Subscriber encoder_2_pulse_sub_;
	ros::Subscriber encoder_3_pulse_sub_;

	ros::Publisher odom_pub_;

	ros::Time current_time_, previous_time_;

	tf::TransformBroadcaster odom_broadcaster_;

	pg_ns::Odometry odometry_;

	public:
	OdometryWrapper(ros::NodeHandle &nh) {
		ROS_INFO("Running /odometry");

		current_time_ = ros::Time::now();
		previous_time_ = ros::Time::now();
		
		if (loadParameters()) {
			ROS_INFO("Successfully loaded all parameters");
		}
		else {
			ROS_WARN("Unable to load all parameters, using defaults");
		}

		encoder_1_pulse_sub_ = nh.subscribe("wheel_1/encoder_pulse", 1, 
			&OdometryWrapper::encoder1PulseCallback, this);

		encoder_2_pulse_sub_ = nh.subscribe("wheel_2/encoder_pulse", 1, 
			&OdometryWrapper::encoder1PulseCallback, this);

		encoder_3_pulse_sub_ = nh.subscribe("wheel_3/encoder_pulse", 1, 
			&OdometryWrapper::encoder1PulseCallback, this);

		odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 1);
	}

	// LOAD PARAMETERS FUNCTION.
	// Load necessary parameters from parameter server
	// (set to defaults if failed).
	bool loadParameters() {
		if (ros::param::has("base_diameter")) {
			float base_diameter;
			ros::param::get("base_diameter", base_diameter);
			odometry_.setBaseDiameter(base_diameter);
			ROS_INFO("Loaded base_diameter");
		}
		else {
			ROS_WARN("Failed to load base_diameter parameter");
			odometry_.setBaseDiameter(0.2);
			return false;
		}

		if (ros::param::has("wheel_radius")) {
			float wheel_radius;
			ros::param::get("wheel_radius", wheel_radius);
			odometry_.setWheelRadius(wheel_radius);
			ROS_INFO("Loaded wheel_radius");
		}
		else {
			ROS_WARN("Failed to load wheel_radius paramter");
			odometry_.setWheelRadius(0.05);
			return false;
		}

		if (ros::param::has("rotary_encoder/pulse_per_meter")) {
			float pulse_per_meter;
			ros::param::get("rotary_encoder/pulse_per_meter", pulse_per_meter);
			odometry_.setPulsePerMeter(pulse_per_meter);
			ROS_INFO("Load pulse_per_meter");
		}
		else {
			ROS_WARN("Failed to load pulse_per_meter parameter");
			odometry_.setPulsePerMeter(426);
			return false;
		}
		return true;
	}
	
	// Callback function for encoders pulse counts
	void encoder1PulseCallback(const std_msgs::Int16 &msg) {
		odometry_.setPulseCounts(msg.data, 0);
	}

	void encoder2PulseCallback(const std_msgs::Int16 &msg) {
		odometry_.setPulseCounts(msg.data, 1);
	}

	void encoder3PulseCallback(const std_msgs::Int16 &msg) {
		odometry_.setPulseCounts(msg.data, 2);
	}

	// RUN FUNCTION
	// Continuously running, updating odometry info
	// every 500 ms.
	void run() {
		ros::Rate rate(10);
		
		while(ros::ok()) {
			current_time_ = ros::Time::now();

			// update odometry for current cycle.
			odometry_.setDeltaTime((current_time_ - previous_time_).toSec());
			odometry_.update();

			vector<double> base_pose = odometry_.getBasePose();
			vector<double> base_twist = odometry_.getBaseTwist();
			
			// Get quaternion from theta.
			geometry_msgs::Quaternion odom_quat = 
				tf::createQuaternionMsgFromYaw(base_pose[2]);

			// Broadcast transformation.
			geometry_msgs::TransformStamped odom_trans;
			
			odom_trans.header.stamp = current_time_;
			odom_trans.header.frame_id = "odom";
			odom_trans.child_frame_id = "base_link";

			odom_trans.transform.translation.x = base_pose[0];
			odom_trans.transform.translation.y = base_pose[1];
			odom_trans.transform.translation.z = 0.0;
			odom_trans.transform.rotation = odom_quat;

			odom_broadcaster_.sendTransform(odom_trans);

			// Publish to /odom topic.
			nav_msgs::Odometry odom;

			odom.header.stamp = current_time_;
			odom.header.frame_id = "odom";
			odom.child_frame_id = "base_link";
			

			odom.pose.pose.position.x = base_pose[0];
			odom.pose.pose.position.y = base_pose[1];
			odom.pose.pose.position.z = 0.0;
			odom.pose.pose.orientation = odom_quat;

			odom.twist.twist.linear.x = base_twist[0];
			odom.twist.twist.linear.y = base_twist[1];
			odom.twist.twist.angular.z = base_twist[2];

			odom_pub_.publish(odom);

			// Set previous time value to current time.
			previous_time_ = current_time_;

			// ROS Logging
			vector<int16_t> delta_pulse_log = odometry_.getDeltaPulse();
			vector<double> wheel_distance_log = odometry_.getWheelDistance();
			vector<double> displacement_log = odometry_.getDisplacement();

			ROS_DEBUG_NAMED(
				"delta_pulse",
				"WHEEL 1 : %i  WHEEL 2 : %i  WHEEL 3 : %i",
				delta_pulse_log[0],
				delta_pulse_log[1],
				delta_pulse_log[2]
				);

			ROS_DEBUG_NAMED(
				"wheel_distance",
				"WHEEL 1 : %lf  WHEEL 2 : %lf WHEEL 3 : %lf",
				wheel_distance_log[0],
				wheel_distance_log[1],
				wheel_distance_log[2]
				);

			ROS_DEBUG_NAMED(
				"displacement",
				"X : %lf  Y : %lf  delta_theta : %lf",
				displacement_log[0],
				displacement_log[1],
				displacement_log[2]
				);

			ros::spinOnce();
			rate.sleep();
		}
	}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "odometry");
	ros::NodeHandle nh;
	
	OdometryWrapper odometry_wrapper(nh);
	odometry_wrapper.run();

	return 0;
}
