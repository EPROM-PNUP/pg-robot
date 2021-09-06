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
	void setPulseCounts(int16_t pulse_count, int8_t index);

	void calcWheelDistance();
	void calcBaseDisplacement();
	void calcBaseTwist();
	void calcBasePose();

	void update();

	vector<int16_t> getDeltaPulse();
	vector<double> getWheelDistance();
	vector<double> getDisplacement();
	vector<double> getBasePose();
	vector<double> getBaseTwist();

};

}

#endif
