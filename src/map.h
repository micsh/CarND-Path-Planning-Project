#ifndef MAP_H
#define MAP_H

#include "spline.h"
#include <vector>

using namespace std;
using namespace tk;

class Map {
public:
	Map();
	Map(vector<double> map_x, vector<double> map_y, vector<double> map_s, vector<double> map_dx, vector<double> map_dy);
	~Map();

	vector<double> getXY(double s, double d) const;

private:
	spline _spline_x;
	spline _spline_y;
	spline _spline_dx;
	spline _spline_dy;
};

#endif