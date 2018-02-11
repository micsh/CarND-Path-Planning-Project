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
	Driver(Map& map, Road& road, Car& car);
	~Driver();

	void create_trajectory(vector<vector<Car>>& carsByLane, vector<vector<double>>& trajectory);

private:
	vector<double> JerkMinimizingTrajectory(vector<double> start, vector<double> end, double T);
	void create_new_trajectory_points(vector<vector<double>>& trajectory);
	void start_driving();
	void keep_in_lane();
	void decrease_speed();
	void change_lane(int target_lane);
	void update_state(int current_lane, int target_lane);

	double _n;
	STATE _state;
	vector<double> _start_s;
	vector<double> _end_s;
	vector<double> _start_d;
	vector<double> _end_d;
	Road _road;
	Map _map;
	Car _car;
};

#endif