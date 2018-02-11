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

double Road::SpeedLimit() { return _speedLimit; }
int Road::Lanes() { return _numberOfLanes; }
double Road::Width() { return _width; }

bool Road::is_lane_free(Car& car, vector<vector<Car>>& carsByLane, int lane) {
	if (lane < 0 || lane > carsByLane.size() - 1) {
		return false;
	}

	auto carsInLane = carsByLane[lane];
	for (int i = 0; i < carsInLane.size(); i++) {
		double distance = carsInLane[i].s() - car.s();
		if (distance > BACK_SAFE_DISTANCE && distance < FRONT_SAFE_DISTANCE) {
			return false;
		}
	}

	return true;
}

int Road::get_free_lane(Car& car, vector<vector<Car>>& carsByLane) {
	int current_lane = lane(car);

	if (is_lane_free(car, carsByLane, current_lane - 1)) {
		if (is_lane_free(car, carsByLane, current_lane + 1)) {
			double avg = average_speed(current_lane - 1, carsByLane);
			if (average_speed(current_lane + 1, carsByLane) > avg) {
				return current_lane + 1;
			}
		}
		return current_lane - 1;
	}
	
	return current_lane;
}

int Road::lane(Car& car)
{
	return lane(car.d());
}

int Road::lane(double d)
{
	auto lane_width = _width / _numberOfLanes;
	return (int)(d / lane_width);
}

double Road::last_target_center_lane(Car& car) {
	return center_lane(lane(car.previous_d()[0]));
}

double Road::center_lane(Car& car) {
	return center_lane(lane(car));
}

double Road::center_lane(int lane) {
	auto lane_width = _width / _numberOfLanes;
	return lane * lane_width + lane_width / 2.0;
}

double Road::safe_speed(Car& car, vector<vector<Car>>& carsByLane) {
	auto inLane = carsByLane[lane(car)];
	double total = 0.0;
	int counter = 0;
	for (int i = 0; i < inLane.size(); i++) {
		double distance = inLane[i].s() - car.s();
		if (distance > 0 && distance < 80) {
			total += inLane[i].v();
			counter++;
		}
	}

	return counter > 0 ? min(max(total / counter, average_speed(carsByLane)), _speedLimit) : _speedLimit;
}

double Road::average_speed(vector<vector<Car>>& carsByLane) {
	double total = 0.0;
	int counter = 0;
	for (int i = 0; i < carsByLane.size(); i++) {
		for (int j = 0; j < carsByLane[i].size(); j++) {
			total += carsByLane[i][j].v();
			counter++;
		}
	}

	return (counter == 0) ? _speedLimit : total / counter;
}

double Road::average_speed(int lane, vector<vector<Car>>& carsByLane) {
	auto inLane = carsByLane[lane];
	double avg = 0.0;
	for (int i = 0; i < inLane.size(); i++) {
		avg += inLane[i].v() / inLane.size();
	}

	return (avg == 0.0) ? _speedLimit : avg;
}