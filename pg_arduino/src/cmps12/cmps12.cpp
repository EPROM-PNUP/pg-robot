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

// INIT FUNCTION.
// Starts UART communication on Serial3 with
// 9600 baudrate.
void CMPS12::init() {
	Serial3.begin(9600);
	delay(10);
}

// GET BEARING READING.
// Read 1 byte at a time (total of 2 bytes).
void CMPS12::getBearing() {
	Serial3.write(BEARING_REG);
	while(Serial3.available() < WORD);

	byte_high_ = Serial3.read();
	byte_low_ = Serial3.read();

	// Shift fist byte by 8 then add the second byte.
	data_raw.bearing_ = ((byte_high_ << 8) + byte_low_) / 10;
}

// GET PITCH READING
// Read 1 byte (total of 1 byte).
void CMPS12::getPitch() {
	Serial3.write(PITCH_REG);
	while(Serial3.available() < BYTE);

	data_raw.pitch_ = Serial3.read();
}

// GET ROLL READING.
// Read 1 byte (total of 1 byte).
void CMPS12::getRoll() {
	Serial3.write(ROLL_REG);
	while(Serial3.available() < BYTE);

	data_raw.roll_ = Serial3.read();
}

// GET ACCELEROMETER READING.
// Read 1 byte at a time. 2 bytes for acceleration in x axis,
// 2 for y axis and 2 for z axis (total 6 bytes).
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

// GET GYRO READING.
// Read 1 byte at a time. 2 bytes for gyro in x axis,
// 2 for y axis and 2 for z axis (total 6 bytes).
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

// GET ALL DATA
// Used to return all data from reading by calling all
// 'get' function.
ImuDataRaw CMPS12::getRawData() {
	getBearing();
	getPitch();
	getRoll();

	getAccel();

	getGyro();

	return data_raw;
}

}

