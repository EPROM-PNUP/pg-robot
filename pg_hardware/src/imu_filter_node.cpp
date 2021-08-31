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
#include <std_msgs/Int16MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include "pg_hardware/imu_filter.hpp"

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

		imu_raw_sub = nh.subscribe("imu_raw", 1,
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

int main(int argc, char** argv) {
	ros::init(argc, argv, "imu_filter");
	ros::NodeHandle nh;

	ImuFilterWrapper imu_filter_wrapper(nh);
	imu_filter_wrapper.run();

	return 0;
}

