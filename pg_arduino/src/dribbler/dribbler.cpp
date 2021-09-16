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

// DRIBBLE FUNCTION
// Dribble ball by spinning the dribbler motor.
void Dribbler::dribble() {
	analogWrite(this->dir_pin_1_, 255);
	analogWrite(this->dir_pin_2_, 0);
}

// STOP DRIBBLE
// Send 0 pwm signal to motor.
void Dribbler::stop() {
	analogWrite(this->dir_pin_1_, 0);
	analogWrite(this->dir_pin_2_, 0);
}

}
