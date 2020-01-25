#include "lcr_trajectory_generator.hpp"
#include <iostream>
#include "../helpers.h"
#include "constants.hpp"
#include "../spline.h"
#include <cmath>
#include <limits>

LCRTrajectoryGenerator::LCRTrajectoryGenerator()
{
}

const std::pair<std::vector<double>, std::vector<double>> LCRTrajectoryGenerator::generate(
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
    const size_t path_size_available = previous_path_x.size();
    const size_t path_size_missing = PATH_SIZE - path_size_available;

    if (path_size_missing == 0)
    {
        std::cout << "Path is already full" << std::endl;
        return {previous_path_x, previous_path_y};
    }
    else
    {
        std::cout << "Will generate #: " << path_size_missing << std::endl;
    }

    std::vector<double> x_plan{previous_path_x};
    std::vector<double> y_plan{previous_path_y};

    double ref_x = main_car.x;
    double ref_y = main_car.y;
    double ref_yaw = deg2rad(main_car.yaw);
    double ref_speed = main_car.speed * MPH_TO_METERPS;

    // Sample

    std::vector<double> x_samples;
    std::vector<double> y_samples;

    if (path_size_available < 2)
    {
        double previous_car_x = ref_x - std::cos(ref_yaw);
        double previous_car_y = ref_y - std::sin(ref_yaw);

        x_samples.push_back(previous_car_x);
        x_samples.push_back(ref_x);

        y_samples.push_back(previous_car_y);
        y_samples.push_back(ref_y);
    }
    else
    {
        ref_x = previous_path_x[path_size_available - 1];
        ref_y = previous_path_y[path_size_available - 1];

        double previous_ref_x = previous_path_x[path_size_available - 2];
        double previous_ref_y = previous_path_y[path_size_available - 2];

        double diff_x = ref_x - previous_ref_x;
        double diff_y = ref_y - previous_ref_y;

        ref_yaw = atan2(diff_y, diff_x);
        ref_speed = distance(0, 0, diff_x, diff_y) / POINT_DELTA_T;

        x_samples.push_back(previous_ref_x);
        x_samples.push_back(ref_x);
        y_samples.push_back(previous_ref_y);
        y_samples.push_back(ref_y);
    }

    // TODO: This value is given with `end_path_s` & `end_path_d`
    std::vector<double> ref_frenet = getFrenet(ref_x, ref_y, ref_yaw, map_waypoints_x, map_waypoints_y);
    const double ref_s = ref_frenet[0];
    const double ref_d = ref_frenet[1];

    // Modification turning requires less horizon/range
    // Modification s step is defined by math
    // Motification d is not constant. Similar to the s.
    constexpr double i_count = 3;
    const double s_range = std::max(main_car.speed * 2, 30.0);
    const double s_step = s_range / i_count;
    const double d_range = adjust_d(ref_d + LANE_WIDTH) - ref_d;
    const double d_step = d_range / i_count;

    double s_dist = s_step;
    double d_dist = d_step;

    for (int i = i_count; i <= i_count; i++)
    {
        vector<double> xy = getXY(ref_s + s_dist, ref_d + d_dist, map_waypoints_s, map_waypoints_x, map_waypoints_y);
        x_samples.push_back(xy[0]);
        y_samples.push_back(xy[1]);

        s_dist += s_step;
        d_dist += d_step;
    }

    // Transform car position to the origin and heading to the y-axis
    transform_fw(x_samples, y_samples, -ref_x, -ref_y, -ref_yaw);

    tk::spline s;
    s.set_points(x_samples, y_samples);

    double x_travelled = 0;
    double speed = ref_speed;

    const OtherCar *car_ahead = get_car_ahead(main_car, other_cars);

    const double object_ahead_distance = car_ahead == nullptr ? std::numeric_limits<double>::max() : distance(main_car.x, main_car.y, car_ahead->x, car_ahead->y);
    const double object_ahead_speed = car_ahead == nullptr ? 0.0 : distance(0, 0, car_ahead->vel_x, car_ahead->vel_y) * MPH_TO_METERPS * POINT_DELTA_T;

    for (int i = 1; i <= path_size_missing; ++i)
    {
        speed = adjust_speed(speed, object_ahead_distance, object_ahead_speed);

        double x = x_travelled + (speed * POINT_DELTA_T);
        double y = s(x);

        x_travelled = x;

        transform_bw(x, y, ref_x, ref_y, ref_yaw);

        x_plan.push_back(x);
        y_plan.push_back(y);
    }

    return {x_plan, y_plan};
}

double LCRTrajectoryGenerator::adjust_speed(const double &speed,
                                            const double &object_ahead_distance,
                                            const double &object_ahead_speed)
{
    bool slower_object_ahead_detected = object_ahead_distance < (speed * 20) && object_ahead_speed < speed;
    double delta_speed = MAX_ACCELERATION *
                         POINT_DELTA_T *
                         (slower_object_ahead_detected ? -1 : 1);

    return std::min(
        speed + delta_speed,
        MAX_SPEED);
}

double LCRTrajectoryGenerator::adjust_d(const double &d)
{
    // | L1  | L2  | L3  |     | L1  | L2  | L3  |
    // |...x.|.....|.....| --> |..x..|.....|.....|

    return std::floor(d / LANE_WIDTH) * LANE_WIDTH + (LANE_WIDTH * 0.5);
}