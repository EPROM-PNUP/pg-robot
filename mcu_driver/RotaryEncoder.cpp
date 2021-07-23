#include "encoder/RotaryEncoder.hpp"

namespace pg_ns {

RotaryEncoder::RotaryEncoder(const byte &en_c1, const byte &en_c2) {
	this->_en_c1 = en_c1;
	this->_en_c2 = en_c2;
}

void RotaryEncoder::init() {
	pinMode(this->_en_c1, INPUT_PULLUP);
	pinMode(this->_en_c2, INPUT);
}

byte RotaryEncoder::getC1Pin() {
	return this->_en_c1;
}

}
