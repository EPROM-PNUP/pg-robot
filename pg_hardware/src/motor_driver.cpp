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
	this->max_velocity_ = 39.0;
	this->velocity_ = 0.0;
	this->pwm_ = 0.0;
}

void MotorDriver::setMaxVelocity(double max_velocity) {
	this->max_velocity_ = max_velocity;
}

void MotorDriver::setVelocity(double velocity) {
	this->velocity_ = velocity;
}

void MotorDriver::calcMotorPWM() {
	this->pwm_ = static_cast<int16_t>
	(
		(this->velocity_ / this->max_velocity_) * 255.0
	);
}

double MotorDriver::getVelocity() {
	return this->velocity_;
}

int16_t MotorDriver::getMotorPWM() {
	return this->pwm_;
}

}
