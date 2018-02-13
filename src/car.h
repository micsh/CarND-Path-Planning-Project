#ifndef CAR_H
#define CAR_H

#include <vector>

using namespace std;

class Car
{
  public:
    Car();
    Car(int id, double s, double d, double v);
    ~Car();

    double s() const;
    double d() const;
	double v() const;
	double target_s() const;
	double target_d() const;
	double target_v() const;
    void update(double s, double d, double v);
	void update_targets(double s, double d, double v);

  private:
    int _id;
    double _s, _d, _v, _target_s, _target_d, _target_v;
};

#endif