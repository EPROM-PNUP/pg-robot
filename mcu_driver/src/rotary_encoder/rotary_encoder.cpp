#include "rotary_encoder.hpp"

namespace pg_ns {

RotaryEncoder::RotaryEncoder(const byte &en_c1, const byte &en_c2) {
	this->en_c1_ = en_c1;
	this->en_c2_ = en_c2;
}

void RotaryEncoder::init() {
	pinMode(this->en_c1_, INPUT_PULLUP);
	pinMode(this->en_c2_, INPUT);
}

byte RotaryEncoder::getC1Pin() {
	return this->en_c1_;
}

}
