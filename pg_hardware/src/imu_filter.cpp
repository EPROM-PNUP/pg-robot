#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


/* IMU DATA STRUCT */
struct ImuData {
	// Roll, Pitch, Yaw orientation 
	float bearing_;
	float pitch_;
	float roll_;

	// Linear acceleration
	float accel_x_;
	float accel_y_;
	float accel_z_;

	// Angular velocity
	float gyro_x_;
	float gyro_y_;
	float gyro_z_;
};


/* IMU FILTER CLASS DEFINITION */
class ImuFilter {
	private:
	ros::Subscriber imu_raw_sub;
	ros::Publisher imu_data_pub;

	tf2::Quaternion quat_tf;
	geometry_msgs::Quaternion quat; 
	sensor_msgs::Imu imu_data_msg;

	// Filtered IMU data
	ImuData imu_data;

	const float ACCEL_SCALE_ = 1.0f/100.f;
	const float GYRO_SCALE_ = 1.0/16.f;
	const float PI = 3.141592;
	const float DEG2RAD = PI/180;

	public:

	ImuFilter(ros::NodeHandle &nh) {
		ROS_INFO("Running /imu_filter");

		imu_raw_sub = nh.subscribe("/imu_raw", 1, 
			&ImuFilter::imuRawCallback, this);

		imu_data_pub = nh.advertise<sensor_msgs::Imu>("imu_data", 10);
	}

	// Callback function for raw data from IMU sensor
	void imuRawCallback(const std_msgs::Int16MultiArray &msg) {
		imu_data.bearing_ = static_cast<float>(msg.data[0]);
		imu_data.pitch_ = static_cast<float>(msg.data[1]);
		imu_data.roll_ = static_cast<float>(msg.data[2]);

		imu_data.accel_x_ = static_cast<float>(msg.data[3]) * ACCEL_SCALE_;
		imu_data.accel_y_ = static_cast<float>(msg.data[4]) * ACCEL_SCALE_;
		imu_data.accel_z_ = static_cast<float>(msg.data[5]) * ACCEL_SCALE_;

		imu_data.gyro_x_ = static_cast<float>(msg.data[6]) * GYRO_SCALE_;
		imu_data.gyro_y_ = static_cast<float>(msg.data[7]) * GYRO_SCALE_;
		imu_data.gyro_z_ = static_cast<float>(msg.data[8]) * GYRO_SCALE_;
	}

	// Main loop to run
	void run() {
		ros::Rate rate(5);

		while(ros::ok()) {
			// Convert orientation from degree to radian
			imu_data.bearing_ *= DEG2RAD;
			imu_data.pitch_ *= DEG2RAD;
			imu_data.roll_ *= DEG2RAD;
			
			// Convert orientation from euler to quaternion
			quat_tf.setRPY(imu_data.roll_, imu_data.pitch_, imu_data.bearing_);
			quat = tf2::toMsg(quat_tf);

			// Copy filtered data to message
			imu_data_msg.orientation = quat;

			imu_data_msg.angular_velocity.x = imu_data.gyro_x_;
			imu_data_msg.angular_velocity.y = imu_data.gyro_y_;
			imu_data_msg.angular_velocity.z = imu_data.gyro_z_;

			imu_data_msg.linear_acceleration.x = imu_data.accel_x_;
			imu_data_msg.linear_acceleration.y = imu_data.accel_y_;
			imu_data_msg.linear_acceleration.z = imu_data.accel_z_;

			// Publish filtered IMU data
			imu_data_pub.publish(imu_data_msg);

			ros::spinOnce();
			rate.sleep();
		}
	}

};


/* MAIN FUNCTION */
int main(int argc, char** argv) {

	ros::init(argc, argv, "imu_filter");
	ros::NodeHandle nh;

	ImuFilter imuf(nh);
	imuf.run();

	return 0;
}
