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

#include "cmps12.hpp"

namespace pg_ns {

CMPS12::CMPS12() {
}

void CMPS12::init() {
	Serial3.begin(9600);
}

void CMPS12::getBearing() {
	Serial3.write(BEARING_REG);
	while(Serial3.available() < WORD);

	byte_high_ = Serial3.read();
	byte_low_ = Serial3.read();

	bearing_ = ((byte_high_ << 8) + byte_low_) / 10;
}

void CMPS12::getPitch() {
	Serial3.write(PITCH_REG);
	while(Serial3.available() < BYTE);

	pitch_ = Serial3.read();
}

void CMPS12::getRoll() {
	Serial3.write(ROLL_REG);
	while(Serial3.available() < BYTE);

	roll_ = Serial3.read();
}

void CMPS12::getAccel() {
	Serial3.write(ACCEL_REG);
	while(Serial3.available() < 6);

	byte_high_ = Serial3.read();
	byte_low_ = Serial3.read();

	data_raw.accel_x_ = ((byte_high_ << 8) + byte_low_);

	byte_high_ = Serial3.read();
	byte_low_ = Serial3.read();

	data_raw.accel_y_ = ((byte_high_ << 8) + byte_low_);

	byte_high_ = Serial3.read();
	byte_low_ = Serial3.read();

	data_raw.accel_z_ = ((byte_high_ << 8) + byte_low_);
}

void CMPS12::getGyro() {
	Serial3.write(GYRO_REG);
	while(Serial3.available() < 6);

	byte_high_ = Serial3.read();
	byte_low_ = Serial3.read();

	data_raw.gyro_x_ = ((byte_high_ << 8) + byte_low_);

	byte_high_ = Serial3.read();
	byte_low_ = Serial3.read();

	data_raw.gyro_y_ = ((byte_high_ << 8) + byte_low_);

	byte_high_ = Serial3.read();
	byte_low_ = Serial3.read();

	data_raw.gyro_z_ = ((byte_high_ << 8) + byte_low_);
}

ImuDataRaw CMPS12::getRawData() {
	getBearing();
	getPitch();
	getRoll();

	getAccel();

	getGyro();

	return data_raw;
}

}

