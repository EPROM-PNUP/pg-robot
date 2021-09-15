#include "proximity.hpp"

namespace pg_ns {

Proximity::Proximity(byte pin) {
	this->pin_ = pin;
}

void Proximity::init() {
	pinMode(this->pin_, INPUT);
}

bool Proximity::ballIsInRange() {
	if (digitalRead(this->pin_)) {
		this->ball_in_range_ = false;
	}
	else {
		this->ball_in_range_ = true;
	}

	return this->ball_in_range_;
}

}
