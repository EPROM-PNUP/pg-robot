#include "include/motor.hpp"

namespace pg_ns {

Motor::Motor(const byte &dir_pin_1, const byte &dir_pin_2) {
	this->_dir_pin_1 = dir_pin_1;
	this->_dir_pin_2 = dir_pin_2;
}

void Motor::init() {
	pinMode(this->_dir_pin_1, OUTPUT);
	pinMode(this->_dir_pin_2, OUTPUT);
}

int16_t Motor::_clip(const int16_t &value, const int16_t &minimum, const int16_t &maximum) {
	if(value > maximum) {
		return maximum;
	}
	else if(value < minimum) {
		return minimum;
	}
	return value;
}

int16_t Motor::getDutyCycle() {
	return this->_duty_cycle;
}

void Motor::move(const int16_t &speed) {
	this->_duty_cycle = _clip(abs(speed), 0, 255);
	if(speed < 0) {
		analogWrite(this->_dir_pin_1, this->_duty_cycle);
		analogWrite(this->_dir_pin_2, 0);
	}
	else {
		analogWrite(this->_dir_pin_1, 0);
		analogWrite(this->_dir_pin_2, this->_duty_cycle);
	}
}

}
