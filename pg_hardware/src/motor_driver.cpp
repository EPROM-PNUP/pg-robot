#include "pg_hardware/motor_driver.hpp"

namespace pg_ns {

MotorDriver::MotorDriver() {
	init();
}

void MotorDriver::init() {
	this->velocity_ = 0;
	this->pwm_ = 0;
}

void MotorDriver::setMaxVelocity(double max_velocity) {
	this->max_velocity_ = max_velocity;
}

void MotorDriver::setVelocity(double velocity) {
	this->velocity_ = velocity;
}

void MotorDriver::calcMotorPWM() {
	this->pwm_ = (this->velocity_ / this->max_velocity_) * 255;	
}

int16_t MotorDriver::getMotorPWM() {
	return this->pwm_;
}

}
