// Copyright (c) 2021 EPROM PNUP
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom
// the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
//
// Author: Wahyu Mahardika

#include "motor.hpp"

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

void Motor::move(int16_t pwm) {
	this->_duty_cycle = _clip(abs(pwm), 0, 255);
	if(pwm < 0) {
		analogWrite(this->_dir_pin_1, this->_duty_cycle);
		analogWrite(this->_dir_pin_2, 0);
	}
	else {
		analogWrite(this->_dir_pin_1, 0);
		analogWrite(this->_dir_pin_2, this->_duty_cycle);
	}
}

}
