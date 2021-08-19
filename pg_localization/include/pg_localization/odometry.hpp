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
	vector<double> velocity_;
	vector<double> displacement_;

	vector<vector<double>> T;

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

	void calcDistanceTravelled();
	void calcRobotDisplacement();
	void calcRobotVelocity();
	void calcRobotGlobalPose();

	void update();

	double getRobotPose(uint8_t index);
	double getRobotVelocity(uint8_t index);
};

}

#endif
