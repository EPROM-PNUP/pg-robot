#include "pg_hardware/motor_driver.hpp"

namespace pg_ns {

MotorDriver::MotorDriver() {
	init();
}

void MotorDriver::init() {
	max_speed_ = 39.0f;

	for(int8_t i = 0; i < 3; i++) {
		speed_[i] = 0.0;
	}

	for(int8_t i = 0; i < 3; i++) {
		pwm_[i] = 0;
	}
}

void MotorDriver::calcMotorPWM() {
	
}

void 
