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

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Int16MultiArray.h>
#include "pg_localization/odometry.hpp"


///////////////////////////////
// ODOMETRY CLASS DEFINITION //
///////////////////////////////

class OdometryWrapper {
	private:
	ros::Subscriber pulse_counts_sub;
	ros::Publisher odom_pub;

	ros::Time current_time_, previous_time_;

	tf::TransformBroadcaster odom_broadcaster;

	pg_ns::Odometry odometry;

	public:
	OdometryWrapper(ros::NodeHandle &nh) {
		ROS_INFO("Running /odometry_pub");

		current_time_ = ros::Time::now();
		previous_time_ = ros::Time::now();

		pulse_counts_sub = nh.subscribe("/encoders_pulse_count", 1, 
			&OdometryWrapper::pulseCountsCallback, this);

		odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
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


///////////////////
// MAIN FUNCTION //
///////////////////

int main(int argc, char** argv) {
	ros::init(argc, argv, "odometry_pub");
	ros::NodeHandle nh;
	
	OdometryWrapper od(nh);
	od.run();

	return 0;
}
