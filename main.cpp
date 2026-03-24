#include <iostream>
#include "car.h"
#include <cmath>
using namespace std;


int main() {
    CarState car(0, 0, 0);
    Point target = {3, 3};
    const double dt = 0.1;
    const double arrive_thresh = 0.05;

    PID pid_v(1.2, 0.08, 0.15);
    PID pid_w(2.5, 0.05, 0.2);

    int step = 0;
    while (step < 2000) {
        Point car_p = {car.x, car.y};
        double dist = getDistance(car_p, target);

        if (dist < arrive_thresh) {
            cout << "\n 到达目标 (3,3)！总步数：" << step << endl;
            break;
        }

        double dx = target.x - car.x;
        double dy = target.y - car.y;
        double theta_des = atan2(dy, dx);
        double angle_err = theta_des - car.theta;

        double v = pid_v.calculate(dist, dt);
        double w = pid_w.calculate(angle_err, dt);

        v = (v > 1.5) ? 1.5 : (v < -1.5 ? -1.5 : v);
        w = (w > 3.0) ? 3.0 : (w < -3.0 ? -3.0 : w);

        car = updateCar(car, v, w, dt);

        cout << "第" << step << "步 | x=" << car.x
             << " y=" << car.y << " 距离：" << dist << endl;
        step++;
    }
    system("pause"); // 仅限 Windows，让程序跑完停一下
    return 0;
}