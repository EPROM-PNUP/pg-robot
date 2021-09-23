#include "pg_hardware/kicker.hpp"

namespace pg_ns {

#ifdef __arm__

Kicker::Kicker(int8_t charge_pin, int8_t release_pin) {
	charge_pin_ = charge_pin;
	release_pin_ = release_pin;

	init();
}

void Kicker::init() {
	wiringPiSetupGpio();

	pinMode(charge_pin_, OUTPUT);
	pinMode(release_pin_, OUTPUT);
}

void Kicker::charge() {
	digitalWrite(release_pin_, HIGH);
	digitalWrite(charge_pin_, LOW);
}

void Kicker::setReady() {
	digitalWrite(charge_pin_, HIGH);
	ready_ = true;
}

void Kicker::release() {
	while (!ready);
	digitalWrite(release_pin_, LOW);
	ready_ = false;
}

#endif

}
