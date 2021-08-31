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
	this->proportional_ = 0.1;
	this->integral_ = 0.0;
	this->derivative_ = 0.0;

	this->error_ = 0.0;
	this->previous_error_ = 0.0;
	this->sum_of_errors_ = 0.0;

	this->reference_velocity_ = 0.0;
	this->actual_velocity_ = 0.0;
}

void MotorDriver::setControllerConstants(
		double proportional,
		double integral,
		double derivative
		)
{
	this->proportional_ = proportional;
	this->integral_ = integral;
	this->derivative_ = derivative;
}

void MotorDriver::setReferenceVelocity(double reference_velocity) {
	this->reference_velocity_ = reference_velocity;
}

void MotorDriver::setActualVelocity(double actual_velocity) {
	this->actual_velocity_ = actual_velocity;
}

void MotorDriver::setMaxPWM(int16_t max_pwm) {
	this->max_pwm_ = max_pwm;
}

void MotorDriver::updateVelocityControl() {
	this->error_ = this->reference_velocity_ - this->actual_velocity_;

	double controlled_signal =
		(this->error_ * this->proportional_)
		+ (this->sum_of_errors_ * this->integral_)
		+ ((this->error_ - this->previous_error_) * this->derivative_);
	
	this->previous_error_ = this->error_;
	this->sum_of_errors_ += this->error_;

	if (this->sum_of_errors_ > 4000) {
		this->sum_of_errors_ = 4000;
	}
	else if (this->sum_of_errors_ < -4000) {
		this->sum_of_errors_ = -4000;
	}

	if (controlled_signal > this->max_pwm_) {
		controlled_signal = this->max_pwm_;
	}
	else if (controlled_signal < -this->max_pwm_) {
		controlled_signal = -this->max_pwm_;
	}

	this->pwm_ = static_cast<int16_t>(controlled_signal);
}

double MotorDriver::getErrorValue() {
	return this->error_;
}

int16_t MotorDriver::getControlledSignal() {
	return this->pwm_;
}

}
