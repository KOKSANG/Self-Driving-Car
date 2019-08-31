#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include <numeric>
#include <algorithm>
#include "mapping.h"
#include "constants.h"
#include "spline.h"

// for convenience
using std::cout;
using std::endl;
using std::string;
using std::vector;
using std::accumulate;


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
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
float sigmoid(float x) { return 1/ (1 + exp(-x)); }
double miles2m(int miles){ return miles*0.44704; }
double m2miles(int m){ return m*2.24; }

int d2lane(double d){
    double new_d = d/ LANE_WIDTH;
    int lane;

    if (new_d < 1){
        lane = 0;
    }
    else if ((new_d >= 1) && (new_d < 2)){
        lane = 1;
    }
    else if (new_d >= 2){
        lane = 2;
    }
    return lane;
}

double lane2d(int lane){
  return (LANE_WIDTH/2) + LANE_WIDTH*lane;
}

// Calculate distance between two points
double get_distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

double get_theta(double vx, double vy){
	return atan2(vy, vx);
}

double calculate_speed(vector<double> value){
  double speed = (value.end() - value.begin())/ (value.size() * PATH_TIMESTEP);
  return speed;
}

double recalibrate_d(double d){
  return (LANE_WIDTH/2) + LANE_WIDTH*d2lane(d);
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s,
                     vector<double> maps_x,
                     vector<double> maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

vector<double> get_kinematics(vector<double> x, vector<double> y, double car_s, double car_d, vector<double> vx, vector<double> vy, vector<double> prev_speed, Mapping map){
  // Calculate previous points size and time taken to complete all previous points
  int size = x.size();
  double time_taken = size*PATH_TIMESTEP;
  // Calculate theta from speed vx, vy
  double theta_i = get_theta(vx[0], vy[0]);
  double theta_f = get_theta(vx[1], vy[1]);
  // Convert into frenet from inputs x, y and theta
  vector<double> frenet_i = map.getFrenet(x[0], y[0], theta_i);
  vector<double> frenet_f = map.getFrenet(x[size-1], y[size-1], theta_f);
  // Calculate s kinematics
  double s = frenet_f[0] - frenet_i[0];
  double s_d = s/ time_taken;
  double s_dd = (s_d - prev_speed[0])/ time_taken;
  // Calculate d kinematics
  double d = frenet_f[1] - frenet_i[1];
  double d_d = d/ time_taken;
  double d_dd = (d_d - prev_speed[1])/ time_taken;
  return {s + car_s, s_d, s_dd, d + car_d, d_d, d_dd};
}

vector<double> calculate_final_kinematics(vector<double> kinematics, double acc, double T){
  double final_jerk = (acc - kinematics[2])/T;
  //cout << "final jerk: final jerk: " << final_jerk << ", desired acc: " << acc << ", acc:" << kinematics[2] << endl;
  double final_acc = acc;
  double final_velocity = kinematics[1] + final_acc*T;
  if (final_velocity >= MAX_SPEED_MS){
    final_velocity = MAX_SPEED_MS;
  }
  double final_position = kinematics[0] + ((kinematics[1] + final_velocity)/2)*T;
  return {final_position, final_velocity, final_acc, final_jerk};
}

#endif  // HELPERS_H