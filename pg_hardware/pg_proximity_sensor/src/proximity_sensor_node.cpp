#include <ros/ros.h>
#include <ros/console.h>

#include <std_msgs/Bool.h>

#include "pg_proximity_sensor/proximity_sensor.hpp"

#ifdef __aarch64__
class ProxiWrapper {
	private:
		ros::Publisher ball_state_pub_;
		ros::Publisher ball_possess_pub_;
		std_msgs::Bool ball_state_msg_;
		std_msgs::Bool ball_possess_msg_;
		pg_ns::Proxi long_proxi_;
		pg_ns::Proxi short_proxi_;
		int long_proxi_pin_ = 5;
		int short_proxi_pin_ = 6;
	
	public:
		ProxiWrapper(ros::NodeHandle &nh) {
			ROS_INFO("Running proxi");
			ball_state_pub_ = nh.advertise<std_msgs::Bool>("/dribbler/ball_in_range", 10);
			ball_possess_pub_ = nh.advertise<std_msgs::Bool>("/dribbler/ball_in_possession", 10);

			if (loadParameters()) {
				ROS_INFO("Successfuly loaded all parameters");
			}
			else {
				ROS_WARN("Unable to load parameters");
			}

			long_proxi_.init(long_proxi_pin_);
			short_proxi_.init(short_proxi_pin_);
		}

		bool loadParameters() {
			if (ros::param::has("/gpio/proximity/long")) {
				ros::param::get("/gpio/proximity/long", long_proxi_pin_);
				ROS_INFO("Loaded long range proximity sensor pin");
			}
			else {
				ROS_WARN("Failed to load long range proximity pin parameter");
				return false;
			}
			if (ros::param::has("/gpio/proximity/short")) {
				ros::param::get("/gpio/proximity/short", short_proxi_pin_);
				ROS_INFO("Loaded short range proximity sensor pin");
			}
			else {
				ROS_WARN("Failed to load short range proximity pin parameter");
				return false;
			}
			return true;
		}

		void run() {
			ros::Rate rate(50);
			while (ros::ok()) {
				ball_state_msg_.data = long_proxi_.ballInRange();
				ball_possess_msg_.data = short_proxi_.ballInRange();
				ball_state_pub_.publish(ball_state_msg_);
				ball_possess_pub_.publish(ball_possess_msg_);
				ros::spinOnce();
				rate.sleep();
			}
		}
};
#else
class ProxiWrapper {
};
#endif

int main(int argc, char** argv) {
	ros::init(argc, argv, "proximity");
	ros::NodeHandle nh;

#ifdef __aarch64__
	ProxiWrapper pw(nh);
	pw.run();
#endif

	return 0;
}

