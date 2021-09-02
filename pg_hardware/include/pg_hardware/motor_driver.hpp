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

#ifndef PG_MOTOR_DRIVER_HPP
#define PG_MOTOR_DRIVER_HPP

#include <cstdint>

namespace pg_ns {

class MotorDriver {
	private:
	int16_t pwm_;
	int16_t max_pwm_;

	int16_t encoder_pulse_;
	int16_t previous_encoder_pulse_;

	double state_;

	public:
	MotorDriver();

	void setMaxPWM(int16_t max_pwm);
	void setPWM(int16_t pwm);
	void setEncoderPulse(int16_t encoder_pulse);

	int16_t getPWM();
	double getState();
};

}

#endif

