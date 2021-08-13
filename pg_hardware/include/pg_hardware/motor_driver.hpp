#ifndef PG_MOTOR_DRIVER_HPP
#define PG_MOTOR_DRIVER_HPP

namespace pg_ns {

class MotorDriver {
	private:
	double max_angular_velocity_;
	double angular_velocity_;
	uint8_t pwm_;

	public:
	MotorDriver();

	void init();
	void setMaxAngularVelocity(float max_velocity);
	void setAngularVelocity(float velocity);
	void calcMotorPWM();
	void getMotorPWM();
};

}

#endif

