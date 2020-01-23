#include "kl_trajectory_generator.hpp"
#include <iostream>
#include "../helpers.h"
#include "constants.hpp"
#include "../spline.h"
#include <cmath>
#include <limits>

KLTrajectoryGenerator::KLTrajectoryGenerator()
{
}

const std::pair<std::vector<double>, std::vector<double>> KLTrajectoryGenerator::generate(
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
    constexpr size_t path_size_total = 10;
    const size_t path_size_available = previous_path_x.size();
    const size_t path_size_missing = path_size_total - path_size_available;

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
    double ref_speed = main_car.speed * Constant::MPH_TO_METERPS;

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
        ref_speed = distance(0, 0, diff_x, diff_y) / Constant::POINT_DELTA_T;

        x_samples.push_back(previous_ref_x);
        x_samples.push_back(ref_x);
        y_samples.push_back(previous_ref_y);
        y_samples.push_back(ref_y);
    }

    double s_range = 90.;
    double s_step = 30.;

    // TODO: This value is given with `end_path_s` & `end_path_d`
    std::vector<double> ref_frenet = getFrenet(ref_x, ref_y, ref_yaw, map_waypoints_x, map_waypoints_y);
    double ref_s = ref_frenet[0];
    const double ref_d = adjust_d(ref_frenet[1]);

    for (double s_dist = s_step; s_dist <= s_range; s_dist += s_step)
    {
        vector<double> xy = getXY(ref_s + s_dist, ref_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
        x_samples.push_back(xy[0]);
        y_samples.push_back(xy[1]);
    }

    // Transform car position to the origin and heading to the y-axis
    transform_fw(x_samples, y_samples, -ref_x, -ref_y, -ref_yaw);

    tk::spline s;
    s.set_points(x_samples, y_samples);

    double x_travelled = 0;
    double speed = ref_speed;

    const OtherCar *car_ahead = get_car_ahead(main_car, other_cars);

    const double object_ahead_distance = car_ahead == nullptr ? std::numeric_limits<double>::max() : distance(main_car.x, main_car.y, car_ahead->x, car_ahead->y);
    const double object_ahead_speed = car_ahead == nullptr ? 0.0 : distance(0, 0, car_ahead->vel_x, car_ahead->vel_y) * Constant::MPH_TO_METERPS * Constant::POINT_DELTA_T;

    const double cax = car_ahead == nullptr ? -1 : car_ahead->x;
    const double cay = car_ahead == nullptr ? -1 : car_ahead->y;
    const double cas = car_ahead == nullptr ? -1 : car_ahead->s;
    const double cad = car_ahead == nullptr ? -1 : car_ahead->d;

    for (int i = 1; i <= path_size_missing; ++i)
    {
        speed = adjust_speed(speed, object_ahead_distance, object_ahead_speed);

        double x = x_travelled + (speed * Constant::POINT_DELTA_T);
        double y = s(x);

        x_travelled = x;

        transform_bw(x, y, ref_x, ref_y, ref_yaw);

        x_plan.push_back(x);
        y_plan.push_back(y);
    }

    return {x_plan, y_plan};
}

const OtherCar *KLTrajectoryGenerator::get_car_ahead(const MainCar &main_car, const std::vector<OtherCar> &other_cars)
{
    const OtherCar *car_ahead = nullptr;

    for (auto &car : other_cars)
    {
        double d_diff = std::abs(main_car.d - car.d);

        // Assumption derives from the knowledge shared on the Q&A section
        // A single lane is 4 units wide in d plane. With this knowledge, we could
        // assume a car width would be ~3 units. And for both cars to align their
        // center distance in d plane needs to be at most 3
        if (d_diff > Constant::LANE_WIDTH * 0.5)
        {
            continue;
        }

        if (car.s < main_car.s)
        {
            continue;
        }

        if (car_ahead == nullptr || car_ahead->s > car.s)
        {
            car_ahead = &car;
        }
    }

    return car_ahead;
}

double KLTrajectoryGenerator::adjust_speed(const double &speed,
                                           const double &object_ahead_distance,
                                           const double &object_ahead_speed)
{
    bool slower_object_ahead_detected = object_ahead_distance < (speed * 5) && object_ahead_speed < speed;
    double delta_speed = Constant::MAX_ACCELERATION *
                         Constant::POINT_DELTA_T *
                         (slower_object_ahead_detected ? -1 : 1);

    return std::min(
        speed + delta_speed,
        Constant::MAX_SPEED);
}

double KLTrajectoryGenerator::adjust_d(const double &d)
{
    // | L1  | L2  | L3  |     | L1  | L2  | L3  |
    // |...x.|.....|.....| --> |..x..|.....|.....|

    return std::floor(d / Constant::LANE_WIDTH) * Constant::LANE_WIDTH + (Constant::LANE_WIDTH * 0.5);
}