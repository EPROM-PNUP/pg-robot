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

#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

#include <cmath>
#include <vector>
#include <cstdint>

using std::vector;

namespace pg_ns {

class Odometry {
	private:
	vector<double> pose_;
	vector<double> twist_;
	vector<double> displacement_;

	double delta_time_;

	vector<int16_t> pulse_counts_;
	vector<int16_t> last_pulse_counts_;
	vector<int16_t> delta_pulse_;

	vector<double> wheel_distance_;

	float base_diameter_;
	float wheel_radius_;
	uint16_t pulse_per_meter_;

	const double PI = 3.141592;

	public:
	Odometry();

	void setDeltaTime(double delta_time);
	void setBaseDiameter(float base_diameter);
	void setWheelRadius(float wheel_radius);
	void setPulsePerMeter(uint16_t pulse_per_meter);
	void setPulseCounts(const vector<int16_t> &pulse_counts);

	void calcWheelDistance();
	void calcBaseDisplacement();
	void calcBaseTwist();
	void calcBasePose();

	void update();

	vector<double> getBasePose();
	vector<double> getBaseTwist();
};

}

#endif
