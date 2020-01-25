#ifndef HELPERS_H
#define HELPERS_H

#include "car.hpp"
#include <string>
#include <vector>
#include <iostream>
#include "state.hpp"

// for convenience
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s);

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2);

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x,
                    const vector<double> &maps_y);

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x,
                 const vector<double> &maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta,
                         const vector<double> &maps_x,
                         const vector<double> &maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s,
                     const vector<double> &maps_x,
                     const vector<double> &maps_y);

void transform_fw(std::vector<double> &x_points, std::vector<double> &y_points, const double &x_shift, const double &y_shift, const double &rotate_rad);

void transform_bw(std::vector<double> &x_points, std::vector<double> &y_points, const double &x_shift, const double &y_shift, const double &rotate_rad);

// TODO: Generate transformation matrix > Inverse if necessary > apply
void transform_fw(double &x, double &y, const double &x_shift, const double &y_shift, const double &rotate_rad);

void transform_bw(double &x, double &y, const double &x_shift, const double &y_shift, const double &rotate_rad);

const OtherCar *get_car_ahead(const MainCar &main_car, const std::vector<OtherCar> &other_cars);

const OtherCar *get_car_left(const MainCar &main_car, const std::vector<OtherCar> &other_cars);

const OtherCar *get_car_right(const MainCar &main_car, const std::vector<OtherCar> &other_cars);

/**
 * Provides the position of the other car relative to the main car.
 */
const Position get_car_position(const MainCar &main_car, const OtherCar &other_car);

const int get_lane(const double d);

#endif // HELPERS_H
