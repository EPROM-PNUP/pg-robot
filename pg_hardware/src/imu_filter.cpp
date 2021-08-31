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

