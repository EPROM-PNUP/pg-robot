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
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include "pg_hardware/imu.hpp"

class ImuWrapper {
	private:
	ros::Subscriber cmps12_orientation_sub_;
	ros::Subscriber magnetometer_sub_;
	ros::Subscriber accelerometer_sub_;
	ros::Subscriber gyroscope_sub_;

	ros::Publisher imu_raw_pub_;
	ros::Publisher mag_pub_;

	tf2::Quaternion quat_tf_;
	geometry_msgs::Quaternion quat_;

	sensor_msgs::Imu imu_raw_msg_;
	sensor_msgs::MagneticField mag_msg_;

	pg_ns::Imu imu_;

	public:
	ImuWrapper(ros::NodeHandle &nh) {
		ROS_INFO("Running /imu_node");

		cmps12_orientation_sub_ = nh.subscribe("orientation", 1,
			&ImuWrapper::orientationCallback, this);

		magnetometer_sub_ = nh.subscribe("magnetometer", 1,
			&ImuWrapper::magnetometerCallback, this);

		accelerometer_sub_ = nh.subscribe("accelerometer", 1,
			&ImuWrapper::accelerometerCallback, this);

		gyroscope_sub_ = nh.subscribe("gyroscope", 1,
			&ImuWrapper::gyroscopeCallback, this);

		imu_raw_pub_ = nh.advertise<sensor_msgs::Imu>("imu_raw", 10);
		mag_pub_ = nh.advertise<sensor_msgs::MagneticField>("mag", 10);
	}

	void orientationCallback(const std_msgs::Int16MultiArray &msg) {
		imu_.setCMPS12Orientation(msg.data);
	}

	void magnetometerCallback(const std_msgs::Int16MultiArray &msg) {
		imu_.setMagReading(msg.data);
	}

	void accelerometerCallback(const std_msgs::Int16MultiArray &msg) {
		imu_.setAccelReading(msg.data);
	}

	void gyroscopeCallback(const std_msgs::Int16MultiArray &msg) {
		imu_.setGyroReading(msg.data);
	}

	void run() {
		ros::Rate rate(5);

		while (ros::ok()) {
			// Get raw data from cmps12 sensor reading
			vector<double> orientation = imu_.getCMPS12Orientation();
			vector<double> magnetic_field = imu_.getMagneticField();
			vector<double> acceleration = imu_.getAcceleration();
			vector<double> angular_velocity = imu_.getAngularVelocity();

			// Create Quaternion representation from Euler Angles
			quat_tf_.setRPY(
				orientation[2],
				orientation[1],
				orientation[0]
			);

			quat_ = tf2::toMsg(quat_tf_);

			// Pass IMU data to msg object
			imu_raw_msg_.orientation = quat_;

			imu_raw_msg_.angular_velocity.x = angular_velocity[0];
			imu_raw_msg_.angular_velocity.y = angular_velocity[1];
			imu_raw_msg_.angular_velocity.z = angular_velocity[2];

			imu_raw_msg_.linear_acceleration.x = acceleration[0];
			imu_raw_msg_.linear_acceleration.y = acceleration[1]; 
			imu_raw_msg_.linear_acceleration.z = acceleration[2];

			// Pass Magnetic Field reading to msg object
			mag_msg_.magnetic_field.x = magnetic_field[0];
			mag_msg_.magnetic_field.y = magnetic_field[1];
			mag_msg_.magnetic_field.z = magnetic_field[2];

			// Publish IMU data & magnetic field
			imu_raw_pub_.publish(imu_raw_msg_);
			mag_pub_.publish(mag_msg_);

			ros::spinOnce();
			rate.sleep();
		}
	}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "imu_node");
	ros::NodeHandle nh;

	ImuWrapper imu_wrapper(nh);
	imu_wrapper.run();

	return 0;
}

