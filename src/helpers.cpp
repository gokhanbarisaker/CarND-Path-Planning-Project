#include "helpers.h"

#include <cmath>
#include "trajectory/constants.hpp"
#include <limits>

// for convenience
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s)
{
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos)
    {
        return "";
    }
    else if (b1 != string::npos && b2 != string::npos)
    {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x,
                    const vector<double> &maps_y)
{
    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for (int i = 0; i < maps_x.size(); ++i)
    {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance(x, y, map_x, map_y);
        if (dist < closestLen)
        {
            closestLen = dist;
            closestWaypoint = i;
        }
    }

    return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x,
                 const vector<double> &maps_y)
{
    int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];

    double heading = atan2((map_y - y), (map_x - x));

    double angle = fabs(theta - heading);
    angle = std::min(2 * pi() - angle, angle);

    if (angle > pi() / 2)
    {
        ++closestWaypoint;
        if (closestWaypoint == maps_x.size())
        {
            closestWaypoint = 0;
        }
    }

    return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta,
                         const vector<double> &maps_x,
                         const vector<double> &maps_y)
{
    int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

    int prev_wp;
    prev_wp = next_wp - 1;
    if (next_wp == 0)
    {
        prev_wp = maps_x.size() - 1;
    }

    double n_x = maps_x[next_wp] - maps_x[prev_wp];
    double n_y = maps_y[next_wp] - maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
    double proj_x = proj_norm * n_x;
    double proj_y = proj_norm * n_y;

    double frenet_d = distance(x_x, x_y, proj_x, proj_y);

    //see if d value is positive or negative by comparing it to a center point
    double center_x = 1000 - maps_x[prev_wp];
    double center_y = 2000 - maps_y[prev_wp];
    double centerToPos = distance(center_x, center_y, x_x, x_y);
    double centerToRef = distance(center_x, center_y, proj_x, proj_y);

    if (centerToPos <= centerToRef)
    {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for (int i = 0; i < prev_wp; ++i)
    {
        frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
    }

    frenet_s += distance(0, 0, proj_x, proj_y);

    return {frenet_s, frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s,
                     const vector<double> &maps_x,
                     const vector<double> &maps_y)
{
    int prev_wp = -1;

    while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1)))
    {
        ++prev_wp;
    }

    int wp2 = (prev_wp + 1) % maps_x.size();

    double heading = atan2((maps_y[wp2] - maps_y[prev_wp]),
                           (maps_x[wp2] - maps_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s - maps_s[prev_wp]);

    double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
    double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

    double perp_heading = heading - pi() / 2;

    double x = seg_x + d * cos(perp_heading);
    double y = seg_y + d * sin(perp_heading);

    return {x, y};
}

void transform_fw(std::vector<double> &x_points, std::vector<double> &y_points, const double &x_shift, const double &y_shift, const double &rotate_rad)
{
    size_t point_size = x_points.size();

    if (point_size != y_points.size())
    {
        // Can't transform non-uniform points
        // TODO: Throw an exception???
        return;
    }

    for (size_t i = 0; i < point_size; ++i)
    {
        transform_fw(x_points[i], y_points[i], x_shift, y_shift, rotate_rad);
    }
}

void transform_bw(std::vector<double> &x_points, std::vector<double> &y_points, const double &x_shift, const double &y_shift, const double &rotate_rad)
{
    size_t point_size = x_points.size();

    if (point_size != y_points.size())
    {
        // Can't transform non-uniform points
        // TODO: Throw an exception???
        return;
    }

    for (size_t i = 0; i < point_size; ++i)
    {
        transform_bw(x_points[i], y_points[i], x_shift, y_shift, rotate_rad);
    }
}

// TODO: Generate transformation matrix > Inverse if necessary > apply
void transform_fw(double &x, double &y, const double &x_shift, const double &y_shift, const double &rotate_rad)
{
    double x_shifted = x + x_shift;
    double y_shifted = y + y_shift;

    x = (x_shifted * std::cos(rotate_rad) - y_shifted * std::sin(rotate_rad));
    y = (x_shifted * std::sin(rotate_rad) + y_shifted * std::cos(rotate_rad));
}

void transform_bw(double &x, double &y, const double &x_shift, const double &y_shift, const double &rotate_rad)
{
    double x_rotated = (x * std::cos(rotate_rad) - y * std::sin(rotate_rad));
    double y_rotated = (x * std::sin(rotate_rad) + y * std::cos(rotate_rad));

    x = x_rotated + x_shift;
    y = y_rotated + y_shift;
}

const bool is_in_lane(const double d, const int lane)
{
    const double left_limit = LANE_WIDTH * lane;
    const double right_limit = left_limit + LANE_WIDTH;

    return d > left_limit && d < right_limit;
}

const bool is_in_range(const double s_diff)
{
    return std::fabs(s_diff) < DETECT_RANGE;
}

const OtherCar *get_car_ahead(const MainCar &main_car, const std::vector<OtherCar> &other_cars)
{
    const OtherCar *car_ahead = nullptr;
    const double lane = get_lane(main_car.d);

    for (auto &car : other_cars)
    {
        double d_diff = std::abs(main_car.d - car.d);

        if (is_in_lane(car.d, lane) && is_in_range(car.s - main_car.s) && ((car.s - main_car.s) > 0))
        {
            if (car_ahead == nullptr || car_ahead->s > car.s)
            {
                car_ahead = &car;
            }
        }
    }

    return car_ahead;
}

// TODO: Move into a function that gets left limit & right limits
const OtherCar *get_car_left(const MainCar &main_car, const std::vector<OtherCar> &other_cars)
{
    const double left_lane = get_lane(main_car.d) - 1;

    if (left_lane < 0)
    {
        return nullptr;
    }

    const OtherCar *car_nearby = nullptr;

    double car_nearby_s_distance = std::numeric_limits<double>::max();

    for (auto &car : other_cars)
    {
        if (is_in_lane(car.d, left_lane) && is_in_range(car.s - main_car.s))
        {
            double s_distance = std::fabs(car.s - main_car.s);
            if (car_nearby == nullptr || s_distance < car_nearby_s_distance)
            {
                car_nearby = &car;
                car_nearby_s_distance = s_distance;
            }
        }
    }

    return car_nearby;
}

const OtherCar *get_car_right(const MainCar &main_car, const std::vector<OtherCar> &other_cars)
{
    const int right_lane = get_lane(main_car.d) + 1;

    if (right_lane > 2)
    {
        return nullptr;
    }

    const OtherCar *car_nearby = nullptr;
    double car_nearby_s_distance = std::numeric_limits<double>::max();

    for (auto &car : other_cars)
    {
        if (is_in_lane(car.d, right_lane) && is_in_range(car.s - main_car.s))
        {
            double s_distance = std::fabs(car.s - main_car.s);
            if (car_nearby == nullptr || s_distance < car_nearby_s_distance)
            {
                car_nearby = &car;
                car_nearby_s_distance = s_distance;
            }
        }
    }

    return car_nearby;
}

const Position get_car_position(const MainCar &main_car, const OtherCar &other_car)
{
    double s_diff = other_car.s - main_car.s;
    constexpr double nearby_range = CAR_LENGTH * 0.5;

    if (s_diff > nearby_range)
    {
        return Position::AHEAD;
    }
    else if (s_diff < -nearby_range)
    {
        return Position::BEHIND;
    }

    return Position::NEARBY;
}

const int get_lane(const double d)
{
    return std::floor(d / LANE_WIDTH);
}
