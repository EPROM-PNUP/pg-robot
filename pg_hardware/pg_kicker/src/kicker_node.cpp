#include <ros/ros.h>
#include <ros/console.h>

#include <std_msgs/Bool.h>
#include <pg_msgs/PrepareKicker.h>
#include <pg_msgs/KickBall.h>

#include "pg_kicker/kicker.hpp"

#ifdef __aarch64__

class KickerWrapper {
	private:
	ros::Subscriber ball_in_range_sub_;
	ros::ServiceServer prepare_kicker_service_;
	ros::ServiceServer kick_service_;
	pg_ns::Kicker kicker_;
	int charge_pin_;
	int release_pin_;
	bool ball_is_in_range_ = false;
	
	public:
	KickerWrapper(ros::NodeHandle &nh) {
		ROS_INFO("Running kicker_node ...");
		
		ball_in_range_sub_ = nh.subscribe("/dribbler/ball_in_range",
			10, &KickerWrapper::ballInRangeCallback, this);

		if (loadParameters()) {
			ROS_INFO("Succesfully loaded all parameters");
		}
		else {
			ROS_WARN("Unable to load parameters");
		}

		kicker_.init(charge_pin_, release_pin_);

		prepare_kicker_service_ = nh.advertiseService("prepare_kicker",
			&KickerWrapper::prepareKicker, this);

		kick_service_ = nh.advertiseService("kick", 
			&KickerWrapper::kick, this);

		startKicker();
	}

	bool loadParameters() {
		if (ros::param::has("/gpio/kicker/charge")) {
			ros::param::get("/gpio/kicker/charge", charge_pin_);
			ROS_INFO("Loaded kicker charge pin");
		}
		else {
			ROS_WARN("Failed to load kicker charge pin parameter");
			return false;
		}

		if (ros::param::has("/gpio/kicker/release")) {
			ros::param::get("/gpio/kicker/release", release_pin_);
			ROS_INFO("Loaded kicker release pin");
		}
		else {
			ROS_WARN("Failed to load kicker release pin parameter");
			return false;
		}
		return true;
	}


	void ballInRangeCallback(const std_msgs::Bool &msg) {
		ball_is_in_range_ = msg.data;
	}

	void startKicker() {
		ros::Time begin_time = ros::Time::now();
		ros::Duration charge_duration(6.0);

		kicker_.charge();

		while (true) {
			ros::Time current_time = ros::Time::now();

			if (
				current_time.toSec() - 
				begin_time.toSec() < 
				charge_duration.toSec()
				) 
			{
				ROS_INFO("Charging ...");
			}
			else {
				kicker_.setReady();
				ROS_INFO("Charged up");
				break;
			}
		}
	}

	bool prepareKicker(
		pg_msgs::PrepareKicker::Request &req,
		pg_msgs::PrepareKicker::Response &res
		) 
	{
		ros::Time begin_time = ros::Time::now();
		ros::Duration charge_duration(req.charge_duration);

		kicker_.charge();

		while (true) {
			ros::Time current_time = ros::Time::now();

			if (
				current_time.toSec() - 
				begin_time.toSec() < 
				charge_duration.toSec()
				) 
			{
				ROS_INFO("Charging ...");
			}
			else {
				kicker_.setReady();
				ROS_INFO("Charged up");
				break;
			}
		}
		return true;
	}

	bool kick(
		pg_msgs::KickBall::Request &req,
		pg_msgs::KickBall::Response &res
		)
	{
		while(!ball_is_in_range_);

		if (req.kick == true) {
			if (kicker_.isReady()) {
				kicker_.release();
				res.kicked = true;
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

#ifdef __aarch64__
	KickerWrapper kicker_wrapper(nh);

	ros::spin();
#else

#endif
}
