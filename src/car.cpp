#include "car.h"

Car::Car() {
    this->_id = -1;
}

Car::Car(int id, double s, double d, double v) {
    this->_id = id;
    this->_s = s;
    this->_d = d;
	this->_v = v;
}

Car::~Car() {}

double Car::s() { return this->_s; }
double Car::d() { return this->_d; }
double Car::v() { return this->_v; }
vector<double> Car::previous_s() { return this->_previous_s; }
void Car::previous_s(vector<double> previous_s) { this->_previous_s = previous_s; }
vector<double> Car::previous_d() { return this->_previous_d; }
void Car::previous_d(vector<double> previous_d) { this->_previous_d = previous_d; }

void Car::update(double s, double d, double v) {
    this->_s = s;
    this->_d = d;
	this->_v = v;
}