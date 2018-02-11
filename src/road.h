#ifndef ROAD_H
#define ROAD_H

#include <vector>
#include "car.h"

using namespace std;

const double FRONT_SAFE_DISTANCE = 40.0; // m
const double BACK_SAFE_DISTANCE = -20.0; // m

class Road {
public:
	Road();
	Road(double width, int numberOfLanes, double speedLimit);
	~Road();

	double SpeedLimit();
	int Lanes();
	double Width();
	bool is_lane_free(Car& car, vector<vector<Car>>& carsByLane, int lane);
	int get_free_lane(Car& car, vector<vector<Car>>& carsByLane);
	int lane(Car& car);
	int lane(double d);
	double last_target_center_lane(Car& car);
	double center_lane(Car& car);
	double center_lane(int lane);

private:
	double _width;
	double _numberOfLanes;
	double _speedLimit;
};

#endif