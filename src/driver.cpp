#include "driver.h"

using namespace std;

Driver::Driver(Map& map, Road& road, Car& car) {
	this->_state = STATE::START;
	this->_map = map;
	this->_road = road;
	this->_car = car;
}

Driver::~Driver() {}

void Driver::create_trajectory(vector<vector<Car>>& carsByLane, vector<vector<double>> &trajectory) {
	if (trajectory[0].size() < POINTS)
	{
		if (this->_state == STATE::START)
		{
			this->start_driving();
		}
		else if (this->_state == STATE::KEEP_LANE)
		{
			if (_road.is_lane_free(_car, carsByLane, _road.lane(_car)))
			{
				this->keep_in_lane();
			}
			else
			{
				int target_lane = _road.get_free_lane(_car, carsByLane);
				if (target_lane == _road.lane(_car))
				{
					this->decrease_speed();
				}
				else
				{
					this->change_lane(target_lane);
				}
			}
		}
		else
		{
			int last_lane = _road.lane(_road.last_target_center_lane(_car));
			if (_road.is_lane_free(_car, carsByLane, last_lane))
			{
				this->keep_in_lane();
			}
			else
			{
				this->decrease_speed();
			}
		}

		this->create_new_trajectory_points(trajectory);
	}
}

void Driver::create_new_trajectory_points(vector<vector<double>> &trajectory) {
	double T = _n * AT;
	auto poly_s = this->JerkMinimizingTrajectory(this->_start_s, this->_end_s, T);
	auto poly_d = this->JerkMinimizingTrajectory(this->_start_d, this->_end_d, T);

	double t, next_s, next_d, mod_s, mod_d;
	vector<double> XY;

	for (int i = 0; i < _n; ++i)
	{
		t = AT * i;
		next_s = 0.0;
		next_d = 0.0;

		for (int j = 0; j < poly_s.size(); ++j)
		{
			next_s += poly_s[j] * pow(t, j);
			next_d += poly_d[j] * pow(t, j);
		}

		XY = _map.getXY(fmod(next_s, TRACK_DISTANCE), fmod(next_d, _road.Width()));

		trajectory[0].push_back(XY[0]);
		trajectory[1].push_back(XY[1]);
	}
}

void Driver::start_driving() {
	_n = 8 * POINTS;
	double target_v = _road.SpeedLimit() * 0.7;
	double target_s = _car.s() + _n * AT * target_v;

	_start_s = { _car.s(), _car.v(), 0.0 };
	_end_s = { target_s, target_v, 0.0 };

	double target_d = _road.center_lane(_car);

	_start_d = { target_d, 0.0, 0.0 };
	_end_d = { target_d, 0.0, 0.0 };

	update_state(_road.lane(_car), _road.lane(_car));
}

void Driver::keep_in_lane() {
	_n = CYCLES * POINTS;
	double target_v = min(_car.previous_s()[1] * 1.2, _road.SpeedLimit());
	double target_s = _car.previous_s()[0] + _n * AT * target_v;

	_start_s = { _car.previous_s()[0], _car.previous_s()[1], 0.0 };
	_end_s = { target_s, target_v, 0.0 };

	double target_d = _road.last_target_center_lane(_car);

	_start_d = { target_d, 0.0, 0.0 };
	_end_d = { target_d, 0.0, 0.0 };

	update_state(_road.lane(target_d), _road.lane(target_d));
}

void Driver::decrease_speed() {
	_n = CYCLES * POINTS;
	double target_v = max(_car.previous_s()[1] * 0.9, _road.SpeedLimit() / 2.0);
	double target_s = _car.previous_s()[0] + _n * AT * target_v;

	_start_s = { _car.previous_s()[0], _car.previous_s()[1], 0.0 };
	_end_s = { target_s, target_v, 0.0 };

	double target_d = _road.last_target_center_lane(_car);

	_start_d = { target_d, 0.0, 0.0 };
	_end_d = { target_d, 0.0, 0.0 };

	update_state(_road.lane(target_d), _road.lane(target_d));
}

void Driver::change_lane(int target_lane) {
	_n = CYCLES * POINTS;
	double target_v = _car.previous_s()[1];
	double target_s = _car.previous_s()[0] + _n * AT * target_v;

	_start_s = { _car.previous_s()[0], target_v, 0.0 };
	_end_s = { target_s, target_v, 0.0 };

	double current_d = _road.last_target_center_lane(_car);
	double target_d = _road.center_lane(target_lane);
	
	_start_d = { current_d, 0.0, 0.0 };
	_end_d = { target_d, 0.0, 0.0 };

	update_state(_road.lane(current_d), _road.lane(target_d));
}

void Driver::update_state(int current_lane, int target_lane) {
	_car.previous_s(this->_end_s);
	_car.previous_d(this->_end_d);

	if (current_lane == target_lane)
	{
		this->_state = STATE::KEEP_LANE;
	}
	else
	{
		this->_state = STATE::CHANGING_LANE;
	}
}

vector<double> Driver::JerkMinimizingTrajectory(vector<double> start, vector<double> end, double T) {
	Eigen::MatrixXd A(3, 3);
	Eigen::MatrixXd B(3, 1);

	A << T * T * T, T * T * T * T, T * T * T * T * T,
		3 * T * T, 4 * T * T * T, 5 * T * T * T * T,
		6 * T, 12 * T * T, 20 * T * T * T;

	B << end[0] - (start[0] + start[1] * T + .5 * start[2] * T * T),
		end[1] - (start[1] + start[2] * T),
		end[2] - start[2];

	Eigen::MatrixXd Ai = A.inverse();
	Eigen::MatrixXd C = Ai * B;

	return { start[0], start[1], .5 * start[2], C.data()[0], C.data()[1], C.data()[2] };
}