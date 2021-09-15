#include "dribbler.hpp"

namespace pg_ns {

Dribbler::Dribbler(byte dir_pin_1, byte dir_pin_2) {
	this->dir_pin_1_ = dir_pin_1;
	this->dir_pin_2_ = dir_pin_2;
}

// INIT FUNCTION
// Set both pins as OUTPUT to control dribbler motor.
void Dribbler::init() {
	pinMode(this->dir_pin_1_, OUTPUT);
	pinMode(this->dir_pin_2_, OUTPUT);
}

// CLIP FUNCTION
// Used to clip pwm value to given maximum and
// and minimum value.
int16_t Dribbler::clip(int16_t value, int16_t minimum, int16_t maximum) {
	if(value > maximum) {
		return maximum;
	}
	else if(value < minimum) {
		return minimum;
	}
	return value;
}

// DRIBBLE FUNCTION
// Dribble ball by spinning the dribbler motor.
void Dribbler::dribble(int16_t pwm) {
	// Clip pwm values to 0 - 255.
	this->duty_cycle_ = clip(abs(pwm), 0, 255);
	// Control rotation direction using if statement.
	if(pwm < 0) {
		analogWrite(this->dir_pin_1_, this->duty_cycle_);
		analogWrite(this->dir_pin_2_, 0);
	}
	else {
		analogWrite(this->dir_pin_1_, 0);
		analogWrite(this->dir_pin_2_, this->duty_cycle_);
	}
}

}
