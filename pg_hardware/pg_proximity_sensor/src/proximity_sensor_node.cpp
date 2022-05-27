#include <ros/ros.h>
#include <ros/console.h>

#include <std_msgs/Bool.h>

#include "pg_proximity_sensor/proximity_sensor.hpp"

#ifdef __aarch64__
class ProxiWrapper {
	private:
		ros::Publisher ball_state_pub_;
		std_msgs::Bool ball_state_msg_;
		pg_ns::Proxi proxi;
		int proxi_pin_ = 25;
	
	public:
		ProxiWrapper(ros::NodeHandle &nh) {
			ROS_INFO("Running proxi");
			ball_state_pub_ = nh.advertise<std_msgs::Bool>("/dribbler/ball_in_range", 10);

			if (loadParameters()) {
				ROS_INFO("Successfuly loaded all parameters");
			}
			else {
				ROS_WARN("Unable to load parameters");
			}

			proxi.init(proxi_pin_);
		}

		bool loadParameters() {
			if (ros::param::has("/gpio/proximity")) {
				ros::param::get("/gpio/proximity", proxi_pin_);
				ROS_INFO("Loaded proximity sensor pin");
			}
			else {
				ROS_WARN("Failed to load proximity pin parameter");
				return false;
			}
			return true;
		}

		void run() {
			ros::Rate rate(50);
			while (ros::ok()) {
				ball_state_msg_.data = proxi.ballInRange();
				ball_state_pub_.publish(ball_state_msg_);
				ROS_DEBUG("ball in range : %d", proxi.ballInRange());
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

