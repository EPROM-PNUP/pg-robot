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

#include "pg_motor_driver/motor_driver.hpp"

namespace pg_ns {

MotorDriver::MotorDriver() {
	pwm_ = 0;
	encoder_pulse_ = 0;
	state_ = 0.0;
	previous_state_ = 0.0;
	filtered_state_ = 0.0;
}

void MotorDriver::setPWM(double pwm) {
	pwm_ = static_cast<int16_t>(pwm);
}

void MotorDriver::setEncoderPulse(int16_t encoder_pulse) {
	encoder_pulse_ = encoder_pulse;
}

int16_t MotorDriver::getPWM() {
	// Set pwm value to 0 when 
	// pwm value drops below 10.
	// if (abs(pwm_) < 16) {
	//  	pwm_ = 0;
	// }

	return pwm_;
}

// GET STATE FUNCTION.
// Calculate motor state (angular velocity)
double MotorDriver::getState() {
	int16_t delta_pulse = encoder_pulse_ - previous_encoder_pulse_;
	state_ = static_cast<double>(delta_pulse / (40/1.0e3));
	state_ = (state_ / 134) * 60.0;
	state_ = (state_ / 60) * (2 * 3.14159265358);
	
	// Low-pass filter 25 Hz cutoff
	filtered_state_ =
		(0.854*filtered_state_)+
		(0.0728*state_)+
		(0.0728*previous_state_);

	previous_encoder_pulse_ = encoder_pulse_;
	previous_state_ = state_;

	return filtered_state_;
}

}
