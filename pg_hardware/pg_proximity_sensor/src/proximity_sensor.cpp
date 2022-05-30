#include "pg_proximity_sensor/proximity_sensor.hpp"

namespace pg_ns {

#ifdef __aarch64__

Proxi::Proxi() {
}

void Proxi::init(uint8_t pin) {
	pin_ = pin;
	wiringPiSetupGpio();
	pinMode(pin_, INPUT);
}

bool Proxi::ballInRange() {
	if (digitalRead(pin_) == HIGH) {
		return true;
	}
	else {
		return false;
	}
}

#else

#endif

}
