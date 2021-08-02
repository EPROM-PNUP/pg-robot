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

	return ((byte_high_ << 8) + byte_low_) / 10;
}

void CMPS12::getPitch() {
	Serial3.write(PITCH_REG);
	while(Serial3.available() < BYTE);

	return Serial3.read();
}

void CMPS12::getRoll() {
	Serial3.write(ROLL_REG);
	while(Serial3.available() < BYTE);

	return Serial3.read();
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

