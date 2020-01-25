#include "car.hpp"
#include "json.hpp"
#include "path_planner.hpp"
#include <fstream>
#include <iostream>
#include "helpers.h"
#include <string>
#include <uWS/uWS.h>
#include <vector>

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main()
{
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line))
  {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  PathPlanner planner(
      map_waypoints_x,
      map_waypoints_y,
      map_waypoints_s,
      map_waypoints_dx,
      map_waypoints_dy);

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy, &planner](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                                               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {

      auto s = hasData(data);

      if (s != "")
      {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry")
        {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          const MainCar main_car(
              car_x,
              car_y,
              car_s,
              car_d,
              car_yaw,
              car_speed);

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          std::vector<OtherCar> other_cars(sensor_fusion.size());

          for (size_t i = 0; i < sensor_fusion.size(); ++i)
          {
            auto &sf_data = sensor_fusion[i];
            auto &car = other_cars[i];

            car.id = sf_data[0];
            car.x = sf_data[1];
            car.y = sf_data[2];
            car.vel_x = sf_data[3];
            car.vel_y = sf_data[4];
            car.s = sf_data[5];
            car.d = sf_data[6];
          }

          json msgJson;

          // !!!: Simulated car does not have any limit O.o
          // !!!: Seriously, it seems to be holomic
          // TODO: Generate path in frenet

          // TODO: Convert generated path to cartesian

          auto &xyPath = planner.plan(
              main_car,
              other_cars,
              previous_path_x,
              previous_path_y,
              end_path_s,
              end_path_d);

          // Car moves to the next point in 20 ms (0.02)
          vector<double> next_x_vals = xyPath.first;
          vector<double> next_y_vals = xyPath.second;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        } // end "telemetry" if
      }
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}
