#include "pg_hardware/odometry.hpp"

namespace pg_ns {

Odometry::Odometry() {
	pose_.assign(3, 0.0);
	velocity_.assign(3, 0.0);
	displacement_.assign(3, 0.0);

	T =
	{
		{cos(PI/6), -cos(PI/6), 0},
		{sin(PI/6),  sin(PI/6), 0},
		{1.0/base_diameter_, 1.0/base_diameter_, 1.0/base_diameter_}
	};

	pulse_counts_.assign(3, 0.0);
	last_pulse_counts_.assign(3, 0.0);
	delta_pulse_.assign(3, 0.0);
	wheel_distance_.assign(3, 0.0);

	delta_time_ = 0.0;
}

void Odometry::setDeltaTime(double delta_time) delta_time_ = delta_time;
void Odometry::setBaseDiameter(float base_diameter) base_diameter_ = base_diameter;
void Ododmetry::setWheelRadius(float wheel_radius) wheel_radius_ = wheel_radius;
void Odometry::setPulsePerMeter(uint16_t pulse_per_meter) pulse_per_meter_ = pulse_per_meter;

void Odometry::setPulseCounts(int16_t a, int16_t b, int16_t c) {
	pulse_counts_[0] = a;
	pulse_counts_[1] = b;
	pulse_counts_[2] = c;
}

void Odometry::calcDistanceTravelled() {
	for (int8_t i = 0; i < 3; i++) {
		delta_pulse_[i] = pulse_counts_[i] - last_pulse_counts_[i];
	}

	for (auto i : delta_pulse_) {
		wheel_distance_[i] = static_cast<double>(i) / static_cast<double>(pulse_per_meter_);
	}
}

void Odometry::calcRobotDisplacement() {
	for (uint8_t i = 0; i < 3; i++) {
		for (uint8_t j = 0; j < 3; j++) {
			displacement_[i] += (T[i][j] * wheel_distance_[j]);
		}
	}
}

void Odometry::calcRobotVelocity() {
	if (delta_time_ > 0) {
		for (uint8_t i = 0; i < 3; i++) {
			velocity_[i] = displacement_[i] / delta_time_;
		}
	}
}

void Odometry::calcRobotGlobalPose() {
	pose_[0] += displacement_[0] * cos(pose[2]);
	pose_[0] += displacement_[1] * (-cos(pose[2]);
	pose_[1] += displacement_[0] * sin(pose[2]);
	pose_[1] += displacement_[1] * cos(pose[2]);
	pose_[2] += displacement_[2];

	if (pose[2] > PI) pose[2] -= 2 * PI;
	else if (pose[2] < -PI) pose[2] += 2 * PI;

	for (uint8_t i = 0; i < 3; i++) {
		last_pulse_counts_[i] = pulse_counts_[i];
	}
}

void Odometry::update() {
	calcDistanceTravelled();
	calcRobotDisplacement();
	calcRobotVelocity();
	calcRobotGlobalPose();
}

vector<double> Odometry::getRobotPose(uint8_t index) return pose_[i];
vector<double> Odometry::getRobotVelocity(uint8_t index) return velocity_[i];

}
