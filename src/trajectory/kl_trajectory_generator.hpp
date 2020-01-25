#ifndef PP_KL_TRAJECTORY_GENERATOR_H
#define PP_KL_TRAJECTORY_GENERATOR_H

#include "../state.hpp"
#include <vector>
#include <utility>
#include "../car.hpp"

class KLTrajectoryGenerator
{
public:
    KLTrajectoryGenerator();
    virtual ~KLTrajectoryGenerator() {}
    const std::pair<std::vector<double>, std::vector<double>> generate(
        const MainCar &main_car,
        const std::vector<OtherCar> &other_cars,
        const std::vector<double> &previous_path_x,
        const std::vector<double> &previous_path_y,
        const std::vector<double> &map_waypoints_x,
        const std::vector<double> &map_waypoints_y,
        const std::vector<double> &map_waypoints_s,
        const std::vector<double> &map_waypoints_dx,
        const std::vector<double> &map_waypoints_dy);

private:
    double adjust_speed(const double &speed,
                        const double &object_ahead_distance,
                        const double &object_ahead_speed);
    double adjust_d(const double &d);
};

#endif