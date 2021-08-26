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

#ifndef PG_CMPS12
#define PG_CMPS12

#include <Arduino.h>

// REGISTER ADDRESS.
// Addresses used to request data from sensor reading.
#define BEARING_REG 0x13
#define PITCH_REG 0x14
#define ROLL_REG 0x15
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
	void getAccel();
	void getGyro();

	ImuDataRaw getRawData();
};

}

#endif

