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

#ifndef PG_CMPS12
#define PG_CMPS12

#include <Arduino.h>

// REGISTER ADDRESS.
// Addresses used to request data from sensor reading.
#define BEARING_REG 0x13
#define PITCH_REG 0x14
#define ROLL_REG 0x15
#define MAG_REG 0x19
#define ACCEL_REG 0x20
#define GYRO_REG 0x21

// DATA SIZE.
// Used to check the number of data received in the serial buffer
// from the sensor.
#define BYTE 1
#define WORD 2


namespace pg_ns {

// RAW DATA STRUCTURE.
// Used to store raw data from sensor reading.
struct ImuDataRaw {
	int16_t bearing_;
	int8_t pitch_;
	int8_t roll_;

	int16_t mag_x_;
	int16_t mag_y_;
	int16_t mag_z_;

	int16_t accel_x_;
	int16_t accel_y_;
	int16_t accel_z_;

	int16_t gyro_x_;
	int16_t gyro_y_;
	int16_t gyro_z_;
};

class CMPS12 {
	private:
	int8_t byte_high_;
	int8_t byte_low_;

	ImuDataRaw data_raw;

	public:
	CMPS12();

	void init();

	void getBearing();
	void getPitch();
	void getRoll();
	void getMag();
	void getAccel();
	void getGyro();

	ImuDataRaw getRawData();
};

}

#endif

