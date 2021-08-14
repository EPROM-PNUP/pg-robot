#ifndef PG_MOTOR_DRIVER_HPP
#define PG_MOTOR_DRIVER_HPP

#include <cstdint>

namespace pg_ns {

class MotorDriver {
	private:
	double max_velocity_;
	double velocity_;
	int16_t pwm_;

	public:
	MotorDriver();

	void init();
	void setMaxVelocity(double max_velocity);
	void setVelocity(double velocity);
	void calcMotorPWM();
	int16_t getMotorPWM();
};

}

#endif

