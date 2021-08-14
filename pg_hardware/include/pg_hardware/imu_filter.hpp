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
