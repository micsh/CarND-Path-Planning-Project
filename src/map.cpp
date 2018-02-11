#include "map.h"

using namespace std;

Map::Map() {}

Map::Map(vector<double> map_x, vector<double> map_y, vector<double> map_s, vector<double> map_dx, vector<double> map_dy) {
	_spline_x.set_points(map_s, map_x);
	_spline_y.set_points(map_s, map_y);
	_spline_dx.set_points(map_s, map_dx);
	_spline_dy.set_points(map_s, map_dy);
}

Map::~Map() {}

vector<double> Map::getXY(double s, double d) {
	double next_x = _spline_x(s) + _spline_dx(s) * d;
	double next_y = _spline_y(s) + _spline_dy(s) * d;

	return { next_x, next_y };
}