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
#include <std_msgs/Int16MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include "pg_hardware/imu_filter.hpp"


////////////////////////
// IMU Filter Wrapper //
////////////////////////

class ImuFilterWrapper {
	private:
	ros::Subscriber imu_raw_sub;
	ros::Publisher imu_filtered_pub;

	tf2::Quaternion quat_tf;
	geometry_msgs::Quaternion quat;
	sensor_msgs::Imu imu_filtered_msg;

	pg_ns::ImuData imu_filtered;
	pg_ns::ImuFilter imu_filter;

	public:
	ImuFilterWrapper(ros::NodeHandle &nh) {
		ROS_INFO("Running /imu_filter");

		imu_raw_sub = nh.subscribe("/imu_raw", 1,
			&ImuFilterWrapper::imuRawCallback, this);

		imu_filtered_pub = nh.advertise<sensor_msgs::Imu>("imu_filtered", 10);
	}

	void imuRawCallback(const std_msgs::Int16MultiArray &msg) {
		imu_filter.filterOrientation(msg.data[0], msg.data[1], msg.data[2]);
		imu_filter.filterAccel(msg.data[4], msg.data[5], msg.data[6]);
		imu_filter.filterGyro(msg.data[7], msg.data[8], msg.data[9]);
	}

	void run() {
		ros::Rate rate(5);

		while (ros::ok()) {
			// Create Quaternion representation from Euler Angles
			quat_tf.setRPY(
				imu_filter.getRoll(),
				imu_filter.getPitch(),
				imu_filter.getBearing()
			);

			quat = tf2::toMsg(quat_tf);

			// Pass IMU filtered data to msg object
			imu_filtered_msg.orientation = quat;

			imu_filtered_msg.angular_velocity.x = imu_filter.getGyroX();
			imu_filtered_msg.angular_velocity.y = imu_filter.getGyroY();
			imu_filtered_msg.angular_velocity.z = imu_filter.getGyroZ();

			imu_filtered_msg.linear_acceleration.x = imu_filter.getAccelX();
			imu_filtered_msg.linear_acceleration.y = imu_filter.getAccelY();
			imu_filtered_msg.linear_acceleration.z = imu_filter.getAccelZ();

			// Publish filtered IMU data
			imu_filtered_pub.publish(imu_filtered_msg);

			ros::spinOnce();
			rate.sleep();
		}
	}
};


///////////////////
// MAIN FUNCTION //
///////////////////

int main(int argc, char** argv) {
	ros::init(argc, argv, "imu_filter");
	ros::NodeHandle nh;

	ImuFilterWrapper imu_filter_wrapper(nh);
	imu_filter_wrapper.run();

	return 0;
}

