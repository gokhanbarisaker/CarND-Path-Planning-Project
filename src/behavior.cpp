#include "behavior.hpp"
#include "helpers.h"
#include "trajectory/constants.hpp"
#include <iostream>
#include <cmath>
#include <limits>

const std::set<State> Behavior::get_successor_states(const State &state)
{
    switch (state)
    {
    case KL:
    {
        return {
            KL,
            LCL,
            LCR,

        };
    }
    case LCL:
    {
        return {
            KL,
            LCL,

        };
    }
    case LCR:
    {
        return {
            KL,
            LCR,

        };
    }
    default:
    {
        throw std::invalid_argument("Unknown state detected");
    }
    }
}

const State Behavior::get_successor_state(const State &state, const MainCar &main_car, const std::vector<OtherCar> &other_cars)
{
    auto &successor_states = get_successor_states(state);
    State best_successor_state = KL;
    double best_successor_state_cost = std::numeric_limits<double>::max();

    const OtherCar *ahead_car = get_car_ahead(main_car, other_cars);
    const OtherCar *left_car = get_car_left(main_car, other_cars);
    const OtherCar *right_car = get_car_right(main_car, other_cars);

    int main_car_lane = get_lane(main_car.d);
    double main_car_speed = main_car.speed * MPH_TO_METERPS;
    double ahead_car_speed = (ahead_car) == nullptr ? MAX_SPEED : distance(0, 0, ahead_car->vel_x, ahead_car->vel_y);
    double left_car_speed = (left_car) == nullptr ? MAX_SPEED : distance(0, 0, left_car->vel_x, left_car->vel_y);
    double right_car_speed = (right_car) == nullptr ? MAX_SPEED : distance(0, 0, right_car->vel_x, right_car->vel_y);

    double ahead_car_s = (ahead_car == nullptr) ? 0 : ahead_car->s;
    double left_car_s = (left_car == nullptr) ? 0 : left_car->s;
    double right_car_s = (right_car == nullptr) ? 0 : right_car->s;

    Position ahead_car_position = (ahead_car == nullptr) ? Position::UNKNOWN : Position::AHEAD;
    Position left_car_position = (left_car == nullptr) ? Position::UNKNOWN : get_car_position(main_car, *left_car);
    Position right_car_position = (right_car == nullptr) ? Position::UNKNOWN : get_car_position(main_car, *right_car);

    for (auto &successor_state : successor_states)
    {
        double other_car_speed = 0;
        double other_car_s = std::numeric_limits<double>::max();
        Position other_car_positon = Position::UNKNOWN;

        if (successor_state == KL)
        {
            other_car_speed = ahead_car_speed;
            other_car_positon = ahead_car_position;
            other_car_s = ahead_car_s;
        }
        if (successor_state == LCL)
        {
            other_car_speed = left_car_speed;
            other_car_positon = left_car_position;
            other_car_s = left_car_s;
        }
        if (successor_state == LCR)
        {
            other_car_speed = right_car_speed;
            other_car_positon = right_car_position;
            other_car_s = right_car_s;
        }

        double successor_state_cost = get_cost(successor_state, main_car_lane, main_car.s, main_car_speed, other_car_s, other_car_speed, other_car_positon);

        if (best_successor_state_cost > successor_state_cost)
        {
            best_successor_state = successor_state;
            best_successor_state_cost = successor_state_cost;
        }
    }

    if (DEBUG)
    {
        std::cout << "== State ==" << std::endl;
        std::cout << "Best: " << best_successor_state << std::endl;
        std::cout << "Lane: " << main_car_lane << std::endl;

        std::cout << "== Radar ==" << std::endl;
        std::cout << "|.." << (left_car_position == Position::AHEAD ? "x" : ".") << "..|.." << (ahead_car_position == Position::AHEAD ? "x" : ".") << "..|.." << (right_car_position == Position::AHEAD ? "x" : ".") << "..|" << std::endl;
        std::cout << "|.." << (left_car_position == Position::NEARBY ? "x" : ".") << "..|..o..|.." << (right_car_position == Position::NEARBY ? "x" : ".") << "..|" << std::endl;
        std::cout << "|.." << (left_car_position == Position::BEHIND ? "x" : ".") << "..|.....|.." << (right_car_position == Position::BEHIND ? "x" : ".") << "..|" << std::endl;
    }

    return best_successor_state;
}

// TODO: Rename nearby to side
double Behavior::get_cost(const State &state, const int lane, const double main_car_s, const double main_car_speed, const double other_car_s, const double other_car_speed, const Position &other_car_positon)
{
    // DONE: Award empty lanes
    // TODO: Smooth out the lane change based on velocity

    const double speed_cost = get_speed_cost(state, main_car_speed, other_car_speed, other_car_positon);
    const double lane_cost = get_lane_cost(state, lane, other_car_positon);
    // const double collision_cost = get_collision_cost(main_car_s, main_car_speed, other_car_s, other_car_speed, other_car_positon);

    const double cost = speed_cost +
                        lane_cost; // +
                                   // 10 * collision_cost;

    std::cout << "== Cost for state " << state << " ==" << std::endl;
    std::cout << "Speed    : " << speed_cost << std::endl;
    std::cout << "Lane     : " << lane_cost << std::endl;
    // std::cout << "Collision: " << collision_cost << std::endl;
    std::cout << "Final    : " << cost << std::endl;

    return cost;
}

double Behavior::get_speed_cost(const State &state, const double main_car_speed, const double other_car_speed, const Position &other_car_position)
{
    switch (state)
    {
    case KL:
    {
        if (other_car_position != Position::AHEAD)
        {
            return 0.0;
        }

        return main_car_speed <= other_car_speed ? (other_car_speed < (MAX_SPEED * 0.9) ? 0.4 : 0.0) : 0.5;
    }
    case LCL:
    case LCR:
    {
        switch (other_car_position)
        {
        case AHEAD:
            return main_car_speed < other_car_speed ? 0.0 : 1.0;
        case NEARBY:
            return main_car_speed > other_car_speed ? 0.5 : 1.0;
        case BEHIND:
            return main_car_speed > other_car_speed ? 0.0 : 1.0;
        case UNKNOWN:
            return 0.0;
        }
    }
    }

    return 1.0;
}

// TODO: Penalize car collision if necessary
double Behavior::get_lane_cost(const State &state, const int lane, const Position &other_car_position)
{
    switch (state)
    {
    case KL:
    {
        return other_car_position == Position::AHEAD ? 0.4 : 0.0;
    }
    case LCL:
    {
        if (lane <= 0)
        {
            return 9999.0;
        }

        if (other_car_position == Position::NEARBY)
        {
            return 9999.0;
        }
        else if (other_car_position == Position::AHEAD)
        {
            return 0.5;
        }

        return 0.1;
    }
    case LCR:
    {
        if (lane >= 2)
        {
            return 9999.0;
        }

        if (other_car_position == Position::NEARBY)
        {
            return 9999.0;
        }
        else if (other_car_position == Position::AHEAD)
        {
            return 0.5;
        }

        return 0.1;
    }
    }

    return 1.0;
}

// double Behavior::get_collision_cost(const double main_car_s, const double main_car_speed, const double other_car_s, const double other_car_speed, const Position &other_car_positon)
// {
//     if (other_car_positon == Position::UNKNOWN)
//     {
//         return 0.0;
//     }

//     const double delta_distance_current = other_car_s - main_car_s;
//     const double delta_speed = other_car_speed - main_car_speed;

//     // Future reflection when the path is followed with constant speed on both objects moving on the same 1-D axis
//     const double delta_distance_predicted = delta_distance_current - (delta_speed * Constant::POINT_DELTA_T * 4);

//     return delta_distance_predicted > (Constant::CAR_LENGTH) ? 0.0 : 1.0;
// }
