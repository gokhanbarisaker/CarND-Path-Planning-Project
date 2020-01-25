#ifndef PP_CAR_H
#define PP_CAR_H

class MainCar
{
public:
  MainCar() = default;
  MainCar(double, double, double, double, double, double);
  virtual ~MainCar() {}

  double x; // meters
  double y; // meters
  double s;
  double d;
  double yaw;   // deg
  double speed; // mph
};

class OtherCar
{
public:
  OtherCar();
  virtual ~OtherCar() {}

  int id;
  double x;     // meters
  double y;     // meters
  double vel_x; // meters per second
  double vel_y; // meters per second
  double s;     // s position
  double d;     // d position
};

enum Position
{
  AHEAD,
  NEARBY,
  BEHIND,
  UNKNOWN,

};

#endif
