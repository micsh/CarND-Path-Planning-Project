#ifndef ROAD_H
#define ROAD_H

#include <vector>
#include "car.h"

using namespace std;

const double FRONT_SAFE_DISTANCE = 30.0; // m
const double BACK_SAFE_DISTANCE = -15.0; // m

class Road {
public:
	Road();
	Road(double width, int numberOfLanes, double speedLimit);
	~Road();

	double SpeedLimit() const;
	int Lanes() const;
	double Width() const;
	bool is_lane_free(vector<vector<Car>> const &carsByLane, Car const &car, int lane) const;
	int get_best_free_lane(vector<vector<Car>> const &carsByLane, Car const &car) const;
	int lane(Car const &car) const;
	int lane(double d) const;
	double last_target_center_lane(Car const &car) const;
	double center_lane(Car const &car) const;
	double center_lane(int lane) const;
	double safe_speed(vector<vector<Car>> const &carsByLane, int lane, double s) const;
	double safe_speed(vector<vector<Car>> const &carsByLane, Car const &car) const;

private:
	double _width;
	double _numberOfLanes;
	double _speedLimit;
};

#endif