#include "pg_hardware/motor_driver.hpp"

namespace pg_ns {

MotorDriver::MotorDriver() {
	this->max_velocity_ = 39.0;
	this->velocity_ = 0.0;
	this->pwm_ = 0.0;
}

void MotorDriver::setMaxVelocity(double max_velocity) {
	this->max_velocity_ = max_velocity;
}

void MotorDriver::setVelocity(double velocity) {
	this->velocity_ = velocity;
}

void MotorDriver::calcMotorPWM() {
	this->pwm_ = static_cast<int16_t>
	(
		(this->velocity_ / this->max_velocity_) * 255.0
	);
}

double MotorDriver::getVelocity() {
	return this->velocity_;
}

int16_t MotorDriver::getMotorPWM() {
	return this->pwm_;
}

}
