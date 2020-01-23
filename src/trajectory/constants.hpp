#ifndef PP_CONSTANTS_H
#define PP_CONSTANTS_H

class Constant
{
public:
    static constexpr double MPH_TO_METERPS = 0.44704;
    static constexpr double POINT_DELTA_T = 0.2;                                                    // second
    static constexpr double MAX_SPEED = 49.8 * MPH_TO_METERPS * 0.1;                                // meters per second
    static constexpr double MAX_ACCELERATION = 10.0 * 0.008;                                        // meters per second^2
    static constexpr double MAX_POINT_INCREMENT = MAX_ACCELERATION * POINT_DELTA_T * POINT_DELTA_T; // meters
    static constexpr double MAX_POINT_DISTANCE = MAX_SPEED * POINT_DELTA_T;                         // meters
    static constexpr double LANE_WIDTH = 4.0;                                                       // meters
};

#endif