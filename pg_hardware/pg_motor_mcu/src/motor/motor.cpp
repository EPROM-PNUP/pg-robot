// Copyright (c) 2021, EPROM PNUP
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
// 
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Author: Wahyu Mahardika

#include "motor.hpp"

namespace pg_ns {

Motor::Motor(const byte &dir_pin_1, const byte &dir_pin_2) {
	this->_dir_pin_1 = dir_pin_1;
	this->_dir_pin_2 = dir_pin_2;
}

// INIT FUNCTION
// Set both pins as OUTPUT to control motor.
void Motor::init() {
	pinMode(this->_dir_pin_1, OUTPUT);
	pinMode(this->_dir_pin_2, OUTPUT);
}

// CLIP FUNCTION
// Used to clip pwm value to given maximum and
// and minimum value.
int16_t Motor::clip(const int16_t &value, const int16_t &minimum, const int16_t &maximum) {
	if(value > maximum) {
		return maximum;
	}
	else if(value < minimum) {
		return minimum;
	}
	return value;
}

// MOVE FUNCTION
// Move motor by sending PWM signals to motor drivers.
void Motor::move(int16_t pwm) {
	// Clip pwm values to 0 - 255.
	this->_duty_cycle = clip(abs(pwm), 0, 255);
	// Control rotation direction using if statement.
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
