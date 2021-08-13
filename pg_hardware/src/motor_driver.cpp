#include "pg_hardware/motor_driver.hpp"

namespace pg_ns {

MotorDriver::MotorDriver() {
	init();
}

void MotorDriver::init() {
	this->angular_velocity_ = 0;
	this->pwm_ = 0;
}

void MotorDriver::setMaxAngularVelocity(float max_velocity) {
	this->max_angular_velocity_ = max_velocity;
}

void MotorDriver::setAngularVelocity(float velocity) {
	this->angular_velocity_ = velocity;
}

void MotorDriver::calcMotorPWM() {
	this->pwm_ = (this->angular_velocity_ / this->max_angular_velocity_) * 255;	
}

uint8_t MotorDrvier::getMotorPWM() {
	return this->pwm_;
}

}
