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

#include "pg_hardware/motor_driver.hpp"

namespace pg_ns {

MotorDriver::MotorDriver() {
	pwm_ = 0;
	encoder_pulse_ = 0;
	state_ = 0.0;
}

void MotorDriver::setMaxPWM(int16_t max_pwm) {
	max_pwm_ = max_pwm;
}

void MotorDriver::setPWM(int16_t pwm) {
	pwm_ = pwm;
}

void MotorDriver::setEncoderPulse(int16_t encoder_pulse) {
	encoder_pulse_ = encoder_pulse;
}

int16_t MotorDriver::getPWM() {
	return pwm_;
}

double MotorDriver::getState() {
	int16_t delta_pulse = encoder_pulse_ - previous_encoder_pulse_;
	state = static_cast<double>((delta_pulse * 600) / 134);
	previous_encoder_pulse_ = encoder_pulse_;

	return state_;
}

}
