#ifndef PP_CONSTANTS_H
#define PP_CONSTANTS_H

#define MPH_TO_METERPS 0.44704
#define POINT_DELTA_T 0.2                       // second
#define MAX_SPEED (49.8 * MPH_TO_METERPS * 0.1) // meters per second
#define MAX_ACCELERATION (10.0 * 0.008)         // meters per second^2
#define DELTA_SPEED 0.01
#define MAX_POINT_INCREMENT (MAX_ACCELERATION * POINT_DELTA_T * POINT_DELTA_T) // meters
#define MAX_POINT_DISTANCE (MAX_SPEED * POINT_DELTA_T)                         // meters
#define LANE_WIDTH 4.0                                                         // meters
#define CAR_LENGTH 10.0
#define DETECT_RANGE 50.0 // Used to detect cars nearby
#define PATH_SIZE 10.0    // Number of path reference point included in the path
#define DEBUG true

#endif