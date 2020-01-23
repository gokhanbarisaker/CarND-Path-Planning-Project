#ifndef PP_TRAJECTORY_GENERATOR_H
#define PP_TRAJECTORY_GENERATOR_H

#include "../state.hpp"
#include "kl_trajectory_generator.hpp"
#include <vector>
#include <utility>
#include "../car.hpp"

class Trajectory
{
public:
    static const std::pair<std::vector<double>, std::vector<double>> of(
        const State &state,
        const MainCar &main_car,
        const std::vector<OtherCar> &other_cars,
        const std::vector<double> &previous_path_x,
        const std::vector<double> &previous_path_y,
        const std::vector<double> &map_waypoints_x,
        const std::vector<double> &map_waypoints_y,
        const std::vector<double> &map_waypoints_s,
        const std::vector<double> &map_waypoints_dx,
        const std::vector<double> &map_waypoints_dy)
    {
        switch (state)
        {
        case KL:
        {
            KLTrajectoryGenerator generator;

            return generator.generate(
                main_car,
                other_cars,
                previous_path_x,
                previous_path_y,
                map_waypoints_x,
                map_waypoints_y,
                map_waypoints_s,
                map_waypoints_dx,
                map_waypoints_dy);
        }
        default:
        {
            return {{}, {}};
        }
        }
    };
};

#endif