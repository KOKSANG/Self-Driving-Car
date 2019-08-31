#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "mapping.cpp"
#include "vehicle.cpp"
#include "behaviour.cpp"
#include "trajectory.cpp"
#include "jmt.h"
#include "helpers.h"
#include "constants.h"
#include "json.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::cout;
using std::endl;
using Eigen::ArrayXd;
using Eigen::MatrixXd;
using namespace Eigen;

/* CONSTANTS defined over here */
// max speed limit in miles per sec
double ref_vel = 0;
int lane = 1;

double distance_spacing;

double prev_x, prev_y, car_vx, car_vy, prev_vx, prev_vy, s, s_d, s_dd, d, d_d, d_dd;
vector<double> prev_speed;
Trajectory best_trajectory, prev_traj;
State state;

int main() {
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
  while (getline(in_map_, line)) {
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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    // Interpolating the track points with spline
    double separation = INTERPOLATION_DISTANCE; // in m
    Mapping map = Mapping(map_waypoints_s, map_waypoints_x, map_waypoints_y, map_waypoints_dx, map_waypoints_dy, separation, TRACK_LENGTH);
    vector<double> kinematics, vx, vy, previous_path_s, previous_path_d;

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];
          int car_lane = d2lane(car_d);
          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];
          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          json msgJson;
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          cout << "=========================================================" << endl;
          /*
          cout << "car_x | " << car_x << endl;
          cout << "car_y | " << car_y << endl;
          cout << "car_s | " << car_s << endl;
          cout << "car_d | " << car_d << endl;
           */

          int prev_size = previous_path_x.size();
          vector<double> points_x, points_y;
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // If car has just started or in INITIAL STATE
          if (prev_size < 2){
            // Retrieve previous two x and y coordinates and push into points
            double prev_ref_x = car_x - cos(car_yaw);
            double prev_ref_y = car_y - sin(car_yaw);
            points_x.push_back(prev_ref_x);
            points_x.push_back(ref_x);
            points_y.push_back(prev_ref_y);
            points_y.push_back(ref_y);
            // Initialize car initial state (READY)
            state = State("RDY", car_lane);
          }
          // If car has been moving for quite awhile
          else {
            int size = NUM_OF_PREVIOUS_POINTS;
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            double prev_ref_x = previous_path_x[prev_size - 2];
            double prev_ref_y = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);

            points_x.push_back(prev_ref_x);
            points_x.push_back(ref_x);
            points_y.push_back(prev_ref_y);
            points_y.push_back(ref_y);

            // Initialize state with previous final lane and id
            state = State("KL", 1);
            //state = State(prev_traj.state->id, prev_traj.state->final_lane);
          }
          // Transform sensor fusion into vehicle object
          // and classify them according to their position in the ego car coordinate system, front, back, left or right
          vector<Vehicle> surrounding_vehicles;
          for (int i = 0; i < sensor_fusion.size(); i++){
            int id = sensor_fusion[i][0];
            double x = sensor_fusion[i][1];
            double y = sensor_fusion[i][2];
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double s = sensor_fusion[i][5];
            double d = sensor_fusion[i][6];
            surrounding_vehicles.push_back(Vehicle(id, x, y, vx, vy, s, d, &map));
          }
          ref_vel += 0.224;
          Vehicle ego = Vehicle(000, ref_x, ref_y, car_s, car_d, ref_yaw, car_speed, previous_path_x, previous_path_y, surrounding_vehicles, &state, &map);
          Behaviour planner = Behaviour(&ego, ref_vel);

          vector<Trajectory> traj = planner.generate_trajectory(points_x, points_y);

          /*
          // Add Frenet of evenly spaced 30m
          vector<double> next_wp0 = map.getXY(car_s + 30, car_d);
          vector<double> next_wp1 = map.getXY(car_s + 60, car_d);
          vector<double> next_wp2 = map.getXY(car_s + 90, car_d);
          // push net waypoints x
          points_x.push_back(next_wp0[0]);
          points_x.push_back(next_wp1[0]);
          points_x.push_back(next_wp2[0]);
          // push net waypoints y
          points_y.push_back(next_wp0[1]);
          points_y.push_back(next_wp1[1]);
          points_y.push_back(next_wp2[1]);

          for (unsigned int i=0; i < points_x.size(); i++){
            // shift car reference angle to 0 degrees
            double shift_x = points_x[i] - ref_x;
            double shift_y = points_y[i] - ref_y;
            points_x[i] = shift_x*cos(0 - ref_yaw) - shift_y*sin(0 - ref_yaw);
            points_y[i] = shift_x*sin(0 - ref_yaw) + shift_y*cos(0 - ref_yaw);
          }

          tk::spline s;
          s.set_points(points_x, points_y);

          for (int i=0; i < previous_path_x.size(); i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Calculate how to break up spline points
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(pow(target_x, 2)+pow(target_y, 2));
          double x_add_on = 0;
          
          for (int i=0; i <= 50-previous_path_x.size(); i++){
            double N = target_dist/ (0.02*ref_vel*0.44704);
            double x_point = x_add_on + target_x/ N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
            y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point); 
          }
          */
          next_x_vals = traj[0].points_x;
          next_y_vals = traj[0].points_y;

          distance_spacing = 0.224;
          
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
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
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}