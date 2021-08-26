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
