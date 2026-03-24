#ifndef CAR_H
#define CAR_H

struct Point {
    double x, y;
};

struct CarState {
    double x, y, theta;
    CarState();
    CarState(double x_, double y_, double t_);
};

class PID {
private:
    double kp, ki, kd;
    double last_error;
    double integral;
    double max_integral;
public:
    PID(double p, double i, double d);
    double calculate(double error, double dt);
    void reset();
};

double getDistance(Point a, Point b);
CarState updateCar(CarState now, double v, double w, double dt);

#endif