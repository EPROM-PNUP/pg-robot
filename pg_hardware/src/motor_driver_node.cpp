#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <pg_hardware/motor_driver.hpp>
#include <pg_msgs/MotorCommand.h>
#include <pg_msgs/WheelVelocityCommand.h>

class MotorDriverWrapper {
	private:
	ros::Subscriber refrence_velocity_sub;
	ros::Publisher motor_pwm_pub;

	pg_msgs::MotorCommand motor_pwm_msg;

	pg_ns::MotorDriver motor_a;
	pg_ns::MotorDriver motor_b;
	pg_ns::MotorDriver motor_c;

	public:
	MotorDriverWrapper(ros::NodeHandle &nh) {
		ROS_INFO("Running /motor_driver");
		
		/*
		motor_pwm_msg.data.clear();
		for (int8_t i = 0; i < 3; i++) {
			motor_pwm_msg.data.push_back(0);
		}
		*/

		double max_velocity;
		ros::param::get("/marco/motor_driver/max_velocity",
			max_velocity);

		motor_a.setMaxVelocity(max_velocity);
		motor_b.setMaxVelocity(max_velocity);
		motor_c.setMaxVelocity(max_velocity);

		refrence_velocity_sub = nh.subscribe("/wheel_refrence_velocity",
			10, &MotorDriverWrapper::refrenceVelocityCallback, this);

		motor_pwm_pub = nh.advertise<pg_msgs::MotorCommand>("motor_pwm", 10);
	}

	void refrenceVelocityCallback(const pg_msgs::WheelVelocityCommand &msg) {
		motor_a.setVelocity(msg.data[0]);
		motor_b.setVelocity(msg.data[1]);
		motor_c.setVelocity(msg.data[2]);

		motor_a.calcMotorPWM();
		motor_b.calcMotorPWM();
		motor_c.calcMotorPWM();
	}

	void run() {
		ros::Rate rate(50);

		while (ros::ok()) {
			motor_pwm_msg.data[0] = motor_a.getMotorPWM();
			motor_pwm_msg.data[1] = motor_b.getMotorPWM();
			motor_pwm_msg.data[2] = motor_c.getMotorPWM();

			motor_pwm_pub.publish(motor_pwm_msg);

			ros::spinOnce();
			rate.sleep();
		}
	}
};

int main(int argc, char** argv) {
	
	ros::init(argc, argv, "motor_driver");
	ros::NodeHandle nh;

	MotorDriverWrapper motor_driver_wrapper(nh);
	motor_driver_wrapper.run();

	return 0;
}

