#include "car.h"

Car::Car() {
    this->_id = -1;
}

Car::Car(int id, double s, double d, double v) {
    _id = id;
    _s = s;
    _d = d;
	_v = v;
}

Car::~Car() {}

double Car::s() const { return _s; }

double Car::d() const { return _d; }

double Car::v() const { return _v; }

double Car::target_s() const { return _target_s; }

double Car::target_d() const { return _target_d; }

double Car::target_v() const { return _target_v; }

void Car::update(double s, double d, double v) {
    _s = s;
    _d = d;
	_v = v;
}

void Car::update_targets(double s, double d, double v) {
	_target_s = s;
	_target_d = d;
	_target_v = v;
}