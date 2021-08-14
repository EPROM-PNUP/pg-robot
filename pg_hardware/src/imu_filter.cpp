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

