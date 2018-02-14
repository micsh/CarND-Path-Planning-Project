#ifndef DRIVER_H
#define DRIVER_H

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include <vector>

#include "car.h"
#include "road.h"
#include "map.h"

using namespace std;

enum class STATE { START, KEEP_LANE, CHANGING_LANE };
const double AT = 0.02;  // s
const double POINTS = 50;
const double CYCLES = 2;

const double TRACK_DISTANCE = 6945.564;

class Driver {

public:
	Driver(Map const &map, Road const &road);
	~Driver();

	void create_trajectory(vector<vector<Car>> const &carsByLane, Car &car, vector<vector<double>>& trajectory);

private:
	vector<double> start_driving(Car& car);
	vector<double> keep_lane(Car& car, double safe_speed_in_lane);
	vector<double> decrease_speed(Car& car);
	vector<double> change_lane(Car& car, int target_lane);
	vector<double> execute_state(vector<vector<Car>> const &carsByLane, Car &car);
	void create_new_trajectory_points(vector<double> const &start, vector<double> const &end, vector<vector<double>>& trajectory) const;
	vector<double> JerkMinimizingTrajectory(vector<double> const &start, vector<double> const &end, double T) const;

	double _n;
	STATE _state;
	Road _road;
	Map _map;
};

#endif