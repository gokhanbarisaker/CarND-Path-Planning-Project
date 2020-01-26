# **Path Planning** 

---

**Highway Driving Project**

The goals / steps of this project are the following:

* Follow Q&A section getting familiar with how system works and getting suggestion on some potential approaches on solving the path planning problem.
* Generate a path to follow the current lane the car is located at. (const speed, acceleration, decelerate when a slower object is detected)
* Create a loosely cost based behavior planner to decide on best possible outcome.
* Generate paths for changing lane behaviors
* Optimize the cost functions, acceleration, etc... based on simulator feedback, until the car drives longer than the required time without an error
* Summarize the results with a written report


[//]: # (Image References)

// TODO: Stack sample photos to support ideas

[image1]: ./examples/successful.png "Target distance exceeded"
[image2]: ./examples/speed.png "Car drives under the highway speed limit"
[image3]: ./examples/collision.png "Car driving without having any collision with other cars"

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/1971/view) individually and describe how I addressed each point in my implementation.  

---
### Compilation

#### 1. The code compiles correctly.

My project includes the following files:
* behavior.(c/h)pp, containing cost based behavior planner
* car.(c/h)pp, data classes for car state
* path_planner.(c/h)pp, a cost based best trajectory generator
* spline.h, a helper class for generating path polynomials
* state.hpp, a behavior state for the main car
* constants.hpp, various constants used across the project
* kl_trajectory_generator.(c/h)pp, a trajectory generator for keep lane state
* lcl_trajectory_generator.(c/h)pp, a trajectory generator for lane change left state
* lcr_trajectory_generator.(c/h)pp, a trajectory generator for lane change right state
* trajectory.hpp

and modifies:
* helpers.(c/h)pp, it wasn't possible to import the `helpers.h` in multiple source files. This change makes it possible
* main.cpp, included required changes to perform successful path planning
* CMakeLists.txt, to have class definition included in the build process

The project also includes some docker images and shell scripts to have a easy setup without changing anything in the running machine 
* Base.dockerfile, a docker image including all the project dependencies
* base.sh, a shell file to build base docker image
* Project.dockerfile, a docker image built on top of `Base.dockerfile`. It builds & run the project source code.
* run.sh, a shell file to build and run the `Project.dockerfile`
* kill.sh, a shell file for killing currently running docker image. (Has a lot of room for improvement :))




### Valid Trajectories

#### 1. The car is able to drive at least 4.32 miles without incident..

This was a challenge to deliver a solid 4.32 miles. There are lots of non-deterministic factors to take into account with lots of opportunities to explore. Especially, figuring out a working cost function was a challenge.

![A car in simulator completing 6.92 miles][image1]

#### 2. The car drives according to the speed limit.

The project implementation does respect the given speed restriction. 

![A car in simulator cruising in 49.48 miles per hour][image2]


However, to this day I am still unsure of what simulator expects. That problem could be observed on `constants.hpp` file. Where the speed limit [needs to be multiplied with `0.1` after converting from MPH to meters per second](https://github.com/gokhanbarisaker/CarND-Path-Planning-Project/blob/master/src/trajectory/constants.hpp#L6).

#### 3. Max Acceleration and Jerk are not Exceeded.

Done. Yet, speed and acceleration is related. Thus, [the vagueness in how I interpret speed applies to the acceleration as well.](https://github.com/gokhanbarisaker/CarND-Path-Planning-Project/blob/master/src/trajectory/constants.hpp#L7)

#### 4. Car does not have collisions.

The first step taken to solve this problem was to avoid a collision with a slower car ahead when cruising on the same lane. 

To solve it, I used following snippet to figure out above problem exists

```cpp
bool slower_object_ahead_detected = object_ahead_distance < (speed * 20) && object_ahead_speed < speed;
```

and calculated change in speed for that path increment with the following snippet

```cpp
double delta_speed = MAX_ACCELERATION *
 POINT_DELTA_T *
 (slower_object_ahead_detected ? -1 : 1);
```

Next challenge was to avoid collusion while changing lanes. Cost functions played a huge role in solving the problem. They penalized behaviors when there is a vehicle nearby.

#### 5. The car stays in its lane, except for the time between changing lanes.

Done. Increasing decisiveness and keeping the trajectory horizon short enough helped me in solving this problem.

#### 6. The car is able to change lanes.

Again, the cost functions played a major role in that. To be more specific, we could take a look into [`get_lane_cost`](https://github.com/gokhanbarisaker/CarND-Path-Planning-Project/blob/master/src/behavior.cpp#L174), while leaving room changing lanes when they are available.
