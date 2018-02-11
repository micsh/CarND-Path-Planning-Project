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

    double s();
    double d();
	double v();
    vector<double> previous_s();
    void previous_s(vector<double> previous_s);
    vector<double> previous_d();
    void previous_d(vector<double> previous_d);
    void update(double s, double d, double v);

  private:
    int _id;
    double _s, _d, _v;
    vector<double> _previous_s, _previous_d;
};

#endif