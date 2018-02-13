#include "driver.h"
using namespace std;

Driver::Driver(Map const &map, Road const &road) {
	_state = STATE::START;
	_map = map;
	_road = road;
}

Driver::~Driver() {}

void Driver::create_trajectory(vector<vector<Car>> const &carsByLane, Car& car, vector<vector<double>>& trajectory) {
	if (trajectory[0].size() < POINTS) {

		vector<double> start;
		if (_state == STATE::START) {
			start = start_driving(car);
		}
		else if (_state == STATE::KEEP_LANE) {
			int current_lane = _road.lane(car);
			if (_road.is_lane_free(carsByLane, car, current_lane)) {
				// is there a better lane ?
				double ss = _road.safe_speed(carsByLane, car);
				int target_lane = current_lane;
				bool change = false;

				if (ss < _road.SpeedLimit()) {
					target_lane = _road.get_best_free_lane(carsByLane, car);
					change = target_lane != current_lane && _road.safe_speed(carsByLane, target_lane, car.s()) > ss + 2.0;
				}

				if (change) {
					start = change_lane(car, target_lane);
				}
				else {
					start = keep_lane(car, ss);
				}
			}
			else {
				cout << "Lane ahead not free... ";
				int target_lane = _road.get_best_free_lane(carsByLane, car);
				if (target_lane == current_lane) {
					start = decrease_speed(car);
				}
				else {
					start = change_lane(car, target_lane);
				}
			}
		}
		else {
			int last_lane = _road.lane(_road.last_target_center_lane(car));
			if (_road.is_lane_free(carsByLane, car, last_lane)) {
				start = keep_lane(car, _road.safe_speed(carsByLane, car));
			}
			else {
				start = decrease_speed(car);
			}
		}


		create_new_trajectory_points(start, { car.target_s(), car.target_d(), car.target_v() }, trajectory);

	}
}

vector<double> Driver::start_driving(Car& car) {
	_n = 4 * CYCLES * POINTS;
	_state = STATE::KEEP_LANE;

	double target_v = 0.6 * _road.SpeedLimit();
	double target_s = car.s() + _n * AT * target_v;
	double target_d = _road.center_lane(car);

	vector<double> start = { car.s(), target_d, car.v() };
	car.update_targets(target_s, target_d, target_v);

	return start;
}

vector<double> Driver::keep_lane(Car& car, double safe_speed_in_lane) {
	_n = CYCLES * POINTS;
	_state = STATE::KEEP_LANE;

	double target_v = min((safe_speed_in_lane + car.target_v()) / 2.0, car.target_v() * 1.3);
	double target_s = car.target_s() + _n * AT * target_v;
	double target_d = _road.last_target_center_lane(car);

	vector<double> start = { car.target_s(), _road.last_target_center_lane(car), car.target_v() };
	car.update_targets(target_s, target_d, target_v);

	return start;
}

vector<double> Driver::decrease_speed(Car& car) {
	_n = CYCLES * POINTS;
	_state = STATE::KEEP_LANE;

	double target_v = 0.8 * car.target_v();
	double target_s = car.target_s() + _n * AT * target_v;
	double target_d = _road.last_target_center_lane(car);

	vector<double> start = { car.target_s(), target_d, car.target_v() };
	car.update_targets(target_s, target_d, target_v);
	
	return start;
}

vector<double> Driver::change_lane(Car& car, int target_lane) {
	_n = CYCLES * POINTS;
	_state = STATE::CHANGING_LANE;

	double target_v = car.target_v();
	double target_s = car.target_s() + _n * AT * target_v;
	double target_d = _road.center_lane(target_lane);

	vector<double> start = { car.target_s(), _road.last_target_center_lane(car), target_v };
	car.update_targets(target_s, target_d, target_v);

	return start;
}

void Driver::create_new_trajectory_points(vector<double> const &start, vector<double> const &end, vector<vector<double>>& trajectory) const {
	double T = _n * AT;
	auto poly_s = JerkMinimizingTrajectory({ start[0], start[2], 0.0 }, { end[0], end[2], 0.0 }, T);
	auto poly_d = JerkMinimizingTrajectory({ start[1], 0.0, 0.0 }, { end[1], 0.0, 0.0 }, T);

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

vector<double> Driver::JerkMinimizingTrajectory(vector<double> const &start, vector<double> const &end, double T) const {
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