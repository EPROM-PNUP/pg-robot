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

#include "pg_hardware/imu_filter.hpp"

namespace pg_ns {

ImuFilter::ImuFilter() {
}

void ImuFilter::filterOrientation(int16_t bearing, int16_t pitch, int16_t roll) {
	filtered_data_.bearing = static_cast<float>(bearing);
	filtered_data_.pitch = static_cast<float>(pitch);
	filtered_data_.roll = static_cast<float>(roll);

	filtered_data_.bearing *= DEG2RAD;
	filtered_data_.pitch *= DEG2RAD;
	filtered_data_.roll *= DEG2RAD;
}

void ImuFilter::filterAccel(int16_t accel_x, int16_t accel_y, int16_t accel_z) {
	filtered_data_.accel_x = static_cast<float>(accel_x) * ACCEL_SCALE_;
	filtered_data_.accel_y = static_cast<float>(accel_y) * ACCEL_SCALE_;
	filtered_data_.accel_z = static_cast<float>(accel_z) * ACCEL_SCALE_;
}

void ImuFilter::filterGyro(int16_t gyro_x, int16_t gyro_y, int16_t gyro_z) {
	filtered_data_.gyro_x = static_cast<float>(gyro_x) * GYRO_SCALE_;
	filtered_data_.gyro_y = static_cast<float>(gyro_y) * GYRO_SCALE_;
	filtered_data_.gyro_z = static_cast<float>(gyro_z) * GYRO_SCALE_;
}

float ImuFilter::getBearing() {return filtered_data_.bearing;}
float ImuFilter::getPitch() {return filtered_data_.pitch;}
float ImuFilter::getRoll() {return filtered_data_.roll;}

float ImuFilter::getAccelX() {return filtered_data_.accel_x;}
float ImuFilter::getAccelY() {return filtered_data_.accel_y;}
float ImuFilter::getAccelZ() {return filtered_data_.accel_z;}

float ImuFilter::getGyroX() {return filtered_data_.gyro_x;}
float ImuFilter::getGyroY() {return filtered_data_.gyro_y;}
float ImuFilter::getGyroZ() {return filtered_data_.gyro_z;}

}

