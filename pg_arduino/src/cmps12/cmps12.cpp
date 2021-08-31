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

