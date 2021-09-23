#include "pg_hardware/dribbler_driver.hpp"

namespace pg_ns {

#ifdef __arm__

DribblerDriver::DribblerDriver(int8_t pin_a, int8_t pin_b) {
	this->pin_a_ = pin_a;
	this->pin_b_ = pin_b;

	init();
}

void DribblerDriver::init() {
	wiringPiSetupGpio();

	pinMode(this->pin_a_, OUTPUT);
	pinMode(this->pin_b_, OUTPUT);
}

void DribblerDriver::dribble() {
	digitalWrite(this->pin_a_, HIGH);
	digitalWrite(this->pin_b_, LOW);
}

void DribblerDriver::stop() {
	digitalWrite(this->pin_a_, LOW);
	digitalWrite(this->pin_b_, LOW);
}

#else

DribblerDriver::DribblerDriver() {
	cmd_dribble_ = false;
}

void DribblerDriver::setDribble() {
	cmd_dribble_ = true;
}

void DribblerDriver::stop() {
	cmd_dribble_ = false;
}

bool DribblerDriver::getCommandDribble() {
	return cmd_dribble_;
}

#endif

}

