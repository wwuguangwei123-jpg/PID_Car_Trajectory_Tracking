#include "car.h"
#include <cmath>

using namespace std;
#define PI 3.14159265

CarState::CarState() : x(0), y(0), theta(0) {}
CarState::CarState(double x_, double y_, double t_) : x(x_), y(y_), theta(t_) {}

double getDistance(Point a, Point b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return sqrt(dx*dx + dy*dy);
}

CarState updateCar(CarState now, double v, double w, double dt) {
    CarState next;
    next.x = now.x + v * cos(now.theta) * dt;
    next.y = now.y + v * sin(now.theta) * dt;
    next.theta = now.theta + w * dt;

    if (next.theta > PI) next.theta -= 2*PI;
    if (next.theta < -PI) next.theta += 2*PI;
    return next;
}

PID::PID(double p, double i, double d)
    : kp(p), ki(i), kd(d), last_error(0), integral(0), max_integral(10) {}

double PID::calculate(double error, double dt) {
    integral += error * dt;
    if (integral > max_integral) integral = max_integral;
    if (integral < -max_integral) integral = -max_integral;

    double deriv = (error - last_error) / dt;
    double out = kp*error + ki*integral + kd*deriv;
    last_error = error;
    return out;
}

void PID::reset() {
    last_error = 0;
    integral = 0;
}