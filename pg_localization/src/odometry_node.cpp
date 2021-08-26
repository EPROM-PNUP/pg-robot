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
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Int16MultiArray.h>
#include "pg_localization/odometry.hpp"

class OdometryWrapper {
	private:
	ros::Subscriber pulse_counts_sub;
	ros::Publisher odom_pub;

	ros::Time current_time_, previous_time_;

	tf::TransformBroadcaster odom_broadcaster;

	pg_ns::Odometry odometry;

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

		pulse_counts_sub = nh.subscribe("encoders_pulse_count", 1, 
			&OdometryWrapper::pulseCountsCallback, this);

		odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
	}

	bool loadParameters() {
		if (ros::param::has("base_diameter")) {
			float base_diameter;
			ros::param::get("base_diameter", base_diameter);
			odometry.setBaseDiameter(base_diameter);
			ROS_INFO("Loaded base_diameter");
		}
		else {
			ROS_WARN("Failed to load base_diameter parameter");
			odometry.setBaseDiameter(0.2);
			return false;
		}

		if (ros::param::has("wheel_radius")) {
			float wheel_radius;
			ros::param::get("wheel_radius", wheel_radius);
			odometry.setWheelRadius(wheel_radius);
			ROS_INFO("Loaded wheel_radius");
		}
		else {
			ROS_WARN("Failed to load wheel_radius paramter");
			odometry.setWheelRadius(0.05);
			return false;
		}

		if (ros::param::has("rotary_encoder/pulse_per_meter")) {
			float pulse_per_meter;
			ros::param::get("rotary_encoder/pulse_per_meter", pulse_per_meter);
			odometry.setPulsePerMeter(pulse_per_meter);
			ROS_INFO("Load pulse_per_meter");
		}
		else {
			ROS_WARN("Failed to load pulse_per_meter parameter");
			odometry.setPulsePerMeter(426);
			return false;
		}
		return true;
	}
	
	// Callback function for encoders pulse counts
	void pulseCountsCallback(const std_msgs::Int16MultiArray &msg) {
		odometry.setPulseCounts(msg.data);
	}

	// Main loop to run
	void run() {
		ros::Rate rate(5);
		
		while(ros::ok()) {
			current_time_ = ros::Time::now();

			odometry.setDeltaTime((current_time_ - previous_time_).toSec());
			odometry.update();
			
			// Get quaternion from theta
			geometry_msgs::Quaternion odom_quat = 
				tf::createQuaternionMsgFromYaw(odometry.getRobotPose(2));

			// Broadcast transformation
			geometry_msgs::TransformStamped odom_trans;
			
			odom_trans.header.stamp = current_time_;
			odom_trans.header.frame_id = "odom";
			odom_trans.child_frame_id = "base_link";

			odom_trans.transform.translation.x = odometry.getRobotPose(0);
			odom_trans.transform.translation.y = odometry.getRobotPose(1);
			odom_trans.transform.translation.z = 0.0;
			odom_trans.transform.rotation = odom_quat;

			odom_broadcaster.sendTransform(odom_trans);

			// Publish to /odom topic
			nav_msgs::Odometry odom;

			odom.header.stamp = current_time_;
			odom.header.frame_id = "odom";
			odom.child_frame_id = "base_link";

			odom.pose.pose.position.x = odometry.getRobotPose(0);
			odom.pose.pose.position.y = odometry.getRobotPose(1);
			odom.pose.pose.position.z = 0.0;
			odom.pose.pose.orientation = odom_quat;

			odom.twist.twist.linear.x = odometry.getRobotVelocity(0);
			odom.twist.twist.linear.y = odometry.getRobotVelocity(1);
			odom.twist.twist.angular.z = odometry.getRobotVelocity(2);

			odom_pub.publish(odom);

			// Set previous time value to current time
			previous_time_ = current_time_;

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
