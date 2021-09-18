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

#ifndef IMU_FILTER_HPP
#define IMU_FILTER_HPP

#include <cstdint>
#include <vector>

using std::vector;

namespace pg_ns {

class Imu {
	private:
	vector<double> cmps12_orientation_;
	vector<double> magnetometer_raw_;
	vector<double> accelerometer_raw_;
	vector<double> gyroscope_raw_;

	const double MAG_SCALE_ = 1.0/16.0;
	const double ACCEL_SCALE_ = 1.0/100.0;
	const double GYRO_SCALE_ = 1.0/16.0;
	const double PI = 3.141592;
	const double DEG2RAD = PI/180;

	public:
	Imu();

	void setCMPS12Reading(const vector<int16_t> &cmps12_orientation);
	void setMagReading(const vector<int16_t> &mag);
	void setAccelReading(const vector<int16_t> &accel);
	void setGyroReading(const vector<int16_t> &gyro);

	vector<double> getCMPS12Orientation();
	vector<double> getMagneticField();
	vector<double> getAcceleration();
	vector<double> getAngularVelocity();
};

}

#endif
