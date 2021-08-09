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


// INCLUDE HEADERS //
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Int16MultiArray.h>
#include <cmath>

// PARAMETERS //
#define BASE 0.2
#define WHEEL_RADIUS 0.05
#define PULSE_PER_METER 426 
#define PI 3.141592

// WHEEL INDEX //
#define WHEEL_A 0
#define WHEEL_B 1
#define WHEEL_C 2

// ODOMETRY CLASS DEFINITION //
class Odometry {

	private:
	
	ros::Subscriber pulse_count_sub;
	ros::Publisher odom_pub;

	ros::Time current_time_, previous_time_;

	tf::TransformBroadcaster odom_broadcaster;

	// Robot position in global frame
	double x = 0.0;
	double y = 0.0;
	double theta = 0.0;

	// Robot displacement
	double dx = 0;
	double dy = 0;
	double dth = 0;
	double dt = 0;

	// Robot velocity
	double vx = 0.0;
	double vy = 0.0;
	double w = 0.0;

	// Distance covered a = wheel_a, b = wheel_b, c = wheel_c
	double da = 0.0;
	double db = 0.0;
	double dc = 0.0;

	// Encoders pulse counts
	int16_t pulse_counts_[3] = {0, 0, 0};
	int16_t last_pulse_counts_[3] = {0, 0, 0};
	int16_t delta_pulse_[3] = {0, 0, 0};

	public:

	// Object constructor
	Odometry(ros::NodeHandle &nh) {
		ROS_INFO("Running /odometry_pub");

		current_time_ = ros::Time::now();
		previous_time_ = ros::Time::now();

		pulse_count_sub = nh.subscribe("/encoders_pulse_count", 1, 
			&Odometry::pulseCountsCallback, this);

		odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
	}
	
	// Callback function for encoders pulse counts
	void pulseCountsCallback(const std_msgs::Int16MultiArray &msg) {
		pulse_counts_[WHEEL_A] = msg.data[WHEEL_A];
		pulse_counts_[WHEEL_B] = msg.data[WHEEL_B];
		pulse_counts_[WHEEL_C] = msg.data[WHEEL_C];
	}

	// Main loop to run
	void run() {
		ros::Rate rate(5);
		
		while(ros::ok()) {

			current_time_ = ros::Time::now();
			
			// Calculate delta pulse for current cycle
			delta_pulse_[WHEEL_A] = pulse_counts_[WHEEL_A] - last_pulse_counts_[WHEEL_A];
			delta_pulse_[WHEEL_B] = pulse_counts_[WHEEL_B] - last_pulse_counts_[WHEEL_B];
			delta_pulse_[WHEEL_C] = pulse_counts_[WHEEL_C] - last_pulse_counts_[WHEEL_C];
			
			// Calculate distance travelled for each wheel
			da = (static_cast<float>(delta_pulse_[WHEEL_A]) / static_cast<float>(PULSE_PER_METER));
			db = (static_cast<float>(delta_pulse_[WHEEL_B]) / static_cast<float>(PULSE_PER_METER));
			dc = (static_cast<float>(delta_pulse_[WHEEL_C]) / static_cast<float>(PULSE_PER_METER));

			// Calculate robot displacement vector
			dx = (da * 0.8660254037844) - (db * 0.8660254037844);
			dy = (da * 0.5) + (db * 0.5) - dc;
			dth = (da / BASE) + (db / BASE) + (dc / BASE);
			dt = (current_time_ - previous_time_).toSec();
			
			// Calculate robot pose in the global frame
			x += (dx * cos(theta)) + (dy * sin(theta));
			y += (dx * (-sin(theta))) + (dy * cos(theta));
			theta += dth;
			
			// Calculate robot velocity
			if(dt > 0) {
				vx = dx / dt;
				vy = dy / dt;
				w = dth / dt;
			}
			
			/*
			x += dx;
			y += dy;
			theta += dth;
			*/
			
			// Make sure theta is in the correct radian range
			if(theta > PI) {
				theta -= 2 * PI;
			}
			else if(theta < -PI) {
				theta += 2 * PI;
			}
			
			// Get quaternion from theta
			geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

			// Broadcast transformation
			geometry_msgs::TransformStamped odom_trans;
			
			odom_trans.header.stamp = current_time_;
			odom_trans.header.frame_id = "odom";
			odom_trans.child_frame_id = "base_link";

			odom_trans.transform.translation.x = x;
			odom_trans.transform.translation.y = y;
			odom_trans.transform.translation.z = 0.0;
			odom_trans.transform.rotation = odom_quat;

			odom_broadcaster.sendTransform(odom_trans);

			// Publish to /odom topic
			nav_msgs::Odometry odom;

			odom.header.stamp = current_time_;
			odom.header.frame_id = "odom";
			odom.child_frame_id = "base_link";

			odom.pose.pose.position.x = x;
			odom.pose.pose.position.y = y;
			odom.pose.pose.position.z = 0.0;
			odom.pose.pose.orientation = odom_quat;

			odom.twist.twist.linear.x = vx;
			odom.twist.twist.linear.y = vy;
			odom.twist.twist.angular.z = w;

			odom_pub.publish(odom);

			// Set previous time value to current time
			previous_time_ = current_time_;

			// Set last pulse counts value to current pulse values
			for(int8_t i = 0; i < 3; i++) {
				last_pulse_counts_[i] = pulse_counts_[i];
			}

			ros::spinOnce();
			rate.sleep();
		}
	}
};

// MAIN FUNCTION //
int main(int argc, char** argv) {
	
	ros::init(argc, argv, "odometry_pub");
	ros::NodeHandle nh;
	
	Odometry od(nh);
	od.run();

	return 0;
}
