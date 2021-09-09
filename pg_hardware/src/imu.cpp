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

#include "pg_hardware/imu.hpp"

namespace pg_ns {

Imu::Imu() {
	cmps12_orientation_.assign(3, 0.0);
	magnetometer_raw_.assign(3, 0.0);
	accelerometer_raw_.assign(3, 0.0);
	gyroscope_raw_.assign(3, 0.0);
}

void Imu::setCMPS12Reading(const vector<int16_t> &cmps12_orientation) {
	for (uint8_t i = 0; i < 3; i++) {
		cmps12_orientation_[i] = static_cast<double>(cmps12_orientation[i]);
	}

	for (auto &i : cmps12_orientation_) {
		i *= DEG2RAD;
	}
}

void Imu::setMagReading(const vector<int16_t> &mag) {
	for (uint8_t i = 0; i < 3; i++) {
		magnetometer_raw_[i] = static_cast<double>(mag[i]);
	}

	for (auto &i : magnetometer_raw_) {
		i *= MAG_SCALE_;
	}
}

void Imu::setAccelReading(const vector<int16_t> &accel) {
	for (uint8_t i = 0; i < 3; i++) {
		accelerometer_raw_[i] = static_cast<double>(accel[i]);
	}

	for (auto &i : accelerometer_raw_) {
		i *= ACCEL_SCALE_;
	}
}

void Imu::setGyroReading(const vector<int16_t> &gyro) {
	for (uint8_t i = 0; i < 3; i++) {
		gyroscope_raw_[i] = static_cast<double>(gyro[i]);
	}

	for (auto &i : gyroscope_raw_) {
		i *= GYRO_SCALE_;
	}
}

vector<double> Imu::getCMPS12Orientation() {
	return cmps12_orientation_;
}

vector<double> Imu::getMagneticField() {
	return magnetometer_raw_;
}

vector<double> Imu::getAcceleration() {
	return accelerometer_raw_;
}

vector<double> Imu::getAngularVelocity() {
	return gyroscope_raw_;
}

}

