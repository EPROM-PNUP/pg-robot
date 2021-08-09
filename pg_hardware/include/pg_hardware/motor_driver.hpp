#ifndef PG_MOTOR_DRIVER_HPP
#define PG_MOTOR_DRIVER_HPP

namespace pg_ns {

class MotorDriver {
	private:
	double max_angular_velocity;
	double shaft_angular_velocity_[3];
	uint8_t pwm_[3];

	public:
	MotorDriver();
	void init();
	void calcMotorPWM();
};

}

#endif

