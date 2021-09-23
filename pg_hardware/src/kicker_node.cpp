#include <ros/ros.h>
#include <ros/console.h>

#include <std_msgs/Bool.h>

#include <pg_msgs/KickBall.h>

#ifdef __arm__

class KickerWrapper {
	private:
	ros::Subscriber ball_in_range_sub_;
	ros::ServiceServer kick_service_;

	pg_ns::Kicker kicker(28, 29);

	bool ball_is_in_range_ = false;
	
	public:
	KickerWrapper(ros::NodeHandle &nh) {
		ROS_INFO("Running kicker ...");
		
		ball_in_range_sub_ = nh.subscribe("/dribbler/ball_in_range",
			10, &KickerWrapper::ballInRangeCallback, this);

		kick_service_ = nh.advertiseService("kick", kick);

		prepareKicker();
	}

	void ballInRangeCallback(const std_msgs::Bool &msg) {
		ball_is_in_range_ = msg.data;
	}

	void prepareKicker() {
		ros::Time begin_time = ros::Time::now();
		ros::Duration charge_duration(3.0);

		kicker.charge();

		while (true) {
			ros::Time current_time = ros::Time::now();

			if (current_time - begin_time < charge_duration.to_sec()) {
				ROS_INFO("Charging ...");
			}
			else {
				kicker.setReady();
				ROS_INFO("Charged up");
				break;
			}
		}
	}

	bool kick(
		pg_msgs::KickBall::Request &req,
		pg_msgs::KickBall::Response &res
		)
	{
		if (req.kick == true) {
			if (kicker.isReady()) {
				kicker.release();
				res.kicked = true;
				prepareKicker();
				return true;
			}
			else {
				ROS_ERROR("Kicker is not ready");
				return false;
			}
		}
		else {
			return false;
		}
	}
};

#else

class KickerWrapper {
};

#endif


int main(int argc, char**argv) {
	ros::init(argc, argv, "kicker");
	ros::NodeHandle nh;

#ifdef __arm__
	KickerWrapper kicker_wrapper(nh);

	ros::spin();
#else

#endif
}
