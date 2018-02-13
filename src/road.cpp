#include <algorithm>
#include "road.h"

using namespace std;

Road::Road() {}

Road::Road(double width, int numberOfLanes, double speedLimit) {
	_width = width;
	_numberOfLanes = numberOfLanes;
	_speedLimit = speedLimit;
}

Road::~Road() {}

double Road::SpeedLimit() const { return _speedLimit; }

int Road::Lanes() const { return _numberOfLanes; }

double Road::Width() const { return _width; }

bool Road::is_lane_free(vector<vector<Car>> const &carsByLane, Car const &car, int lane) const {
	if (lane < 0 || lane > carsByLane.size() - 1) {
		return false;
	}

	auto carsInLane = carsByLane[lane];
	double ds, dv, dsdv;
	for (auto&& otherCar : carsInLane) {
		ds = otherCar.s() - car.s();
		dv = otherCar.v() - car.v();
		dsdv = dv != 0 ? ds / dv : 0;

		if (ds < 0 && ds > BACK_SAFE_DISTANCE && dsdv > -1.5) {
			return false;
		}

		if (ds >= 0 && ds < FRONT_SAFE_DISTANCE && dsdv > -1.5) {
			return false;
		}
	}

	return true;
}

int Road::get_best_free_lane(vector<vector<Car>> const &carsByLane, Car const &car) const {
	int current_lane = lane(car);

	double ls = is_lane_free(carsByLane, car, current_lane - 1) ? safe_speed(carsByLane, current_lane - 1, car.s()) : -1;
	double cs = is_lane_free(carsByLane, car, current_lane) ? safe_speed(carsByLane, current_lane, car.s()) : -1;
	double rs = is_lane_free(carsByLane, car, current_lane + 1) ? safe_speed(carsByLane, current_lane + 1, car.s()) : -1;

	return cs >= ls ? (cs >= rs ? current_lane : current_lane + 1) : (ls >= rs ? current_lane - 1 : current_lane + 1);
}

int Road::lane(Car const &car) const {
	return lane(car.d());
}

int Road::lane(double d) const {
	auto lane_width = _width / _numberOfLanes;
	return (int)(d / lane_width);
}

double Road::last_target_center_lane(Car const &car) const {
	return center_lane(lane(car.target_d()));
}

double Road::center_lane(Car const &car) const {
	return center_lane(lane(car));
}

double Road::center_lane(int lane) const {
	auto lane_width = _width / _numberOfLanes;
	return lane * lane_width + lane_width / 2.0;
}

double Road::safe_speed(vector<vector<Car>> const &carsByLane, Car const &car) const {
	return safe_speed(carsByLane, lane(car), car.s());
}

double Road::safe_speed(vector<vector<Car>> const &carsByLane, int lane, double s) const {
	auto carsInLane = carsByLane[lane];
	double distance, total = 0.0;
	int counter = 0;

	for (auto&& otherCar : carsInLane) {
		distance = otherCar.s() - s;
		if (distance > 0 && distance < 80) {
			total += otherCar.v();
			counter++;
		}
	}

	return counter > 0 ? min(total / counter, _speedLimit) : _speedLimit;
}