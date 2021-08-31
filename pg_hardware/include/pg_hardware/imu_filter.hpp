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

namespace pg_ns {

struct ImuData {
	float bearing;
	float pitch;
	float roll;

	float accel_x;
	float accel_y;
	float accel_z;

	float gyro_x;
	float gyro_y;
	float gyro_z;
};

class ImuFilter {
	private:
	ImuData filtered_data_;
	const float ACCEL_SCALE_ = 1.0f/100.f;
	const float GYRO_SCALE_ = 1.0/16.f;
	const float PI = 3.141592;
	const float DEG2RAD = PI/180;

	public:
	ImuFilter();

	void filterOrientation(int16_t bearing, int16_t pitch, int16_t roll);
	void filterAccel(int16_t accel_x, int16_t accel_y, int16_t accel_z);
	void filterGyro(int16_t gyro_x, int16_t gyro_y, int16_t gyro_z);

	float getBearing();
	float getPitch();
	float getRoll();

	float getAccelX();
	float getAccelY();
	float getAccelZ();

	float getGyroX();
	float getGyroY();
	float getGyroZ();
};

}

#endif
