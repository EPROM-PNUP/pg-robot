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

#include <iostream>
#include "pg_localization/odometry.hpp"

namespace pg_ns {

Odometry::Odometry() {
	// Set all attributes to 0.
	pose_.assign(3, 0.0);
	twist_.assign(3, 0.0);
	displacement_.assign(3, 0.0);

	pulse_counts_.assign(3, 0.0);
	last_pulse_counts_.assign(3, 0.0);
	delta_pulse_.assign(3, 0.0);
	wheel_distance_.assign(3, 0.0);

	delta_time_ = 0.0;
}

void Odometry::setDeltaTime(double delta_time) {
	delta_time_ = delta_time;
}

void Odometry::setBaseDiameter(float base_diameter) {
	base_diameter_ = base_diameter;
}

void Odometry::setWheelRadius(float wheel_radius) {
	wheel_radius_ = wheel_radius;
}

void Odometry::setPulsePerMeter(uint16_t pulse_per_meter) {
	pulse_per_meter_ = pulse_per_meter;
}

// SET PULSE.
// Store pulse counts from encoders readings.
void Odometry::setPulseCounts(int16_t pulse_count, int8_t index) {
	pulse_counts_[index] = pulse_count;
}

// WHEEL TRAVELLED DISTANCE.
// Calculate distance travelled by each wheels.
void Odometry::calcWheelDistance() {
	for (uint8_t i = 0; i < 3; i++) {
		delta_pulse_[i] = pulse_counts_[i] - last_pulse_counts_[i];
	}

	for (uint8_t i = 0; i < 3; i++) {
		wheel_distance_[i] = static_cast<double>(delta_pulse_[i]) / 
			static_cast<double>(pulse_per_meter_);
	}

	for (uint8_t i = 0; i < 3; i++) {
		last_pulse_counts_[i] = pulse_counts_[i];
	}
}

// BASE LINK DISPLACEMENT.
// Calculate base_link displacement at one cycle.
void Odometry::calcBaseDisplacement() {
	vector<vector<double>> T =
	{
		{cos(PI/6), -cos(PI/6), 0},
		{sin(PI/6),  sin(PI/6),-1},
		{1.0/base_diameter_, 1.0/base_diameter_, 1.0/base_diameter_}
	};

	for (auto &i : displacement_) {
		i = 0.0;
	}

	for (uint8_t i = 0; i < 3; i++) {
		for (uint8_t j = 0; j < 3; j++) {
			displacement_[i] += (T[i][j] * wheel_distance_[j]);
		}
	}
}

// BASE LINK TWIST.
// Calculate base_link twist at one cycle.
void Odometry::calcBaseTwist() {
	if (delta_time_ > 0) {
		for (uint8_t i = 0; i < 3; i++) {
			twist_[i] = displacement_[i] / delta_time_;
		}
	}
}

// BASE LINK POSE.
// Calculate base_link pose in the odom frame.
void Odometry::calcBasePose() {
	vector<vector<double>> R =
	{
		{cos(pose_[2]), -sin(pose_[2]), 0},
		{sin(pose_[2]),  cos(pose_[2]), 0},
		{0, 0, 1}
	};

	for (uint8_t i = 0; i < 3; i++) {
		for (uint8_t j = 0; j < 3; j++) {
			pose_[i] += R[i][j] * displacement_[j];
		}
	}

	// Make sure theta is in the correct range.
	if (pose_[2] > PI) {
		pose_[2] -= 2 * PI;
	}
	else if (pose_[2] < -PI) {
		pose_[2] += 2 * PI;
	}
}

// UPDATE FUNCTION.
// Calculate all odometry info by calling all
// 'calc' functions.
void Odometry::update() {
	calcWheelDistance();
	calcBaseDisplacement();
	calcBaseTwist();
	calcBasePose();
}

vector<double> Odometry::getBasePose() {
	return pose_;
}

vector<double> Odometry::getBaseTwist() {
	return twist_;
}

}
