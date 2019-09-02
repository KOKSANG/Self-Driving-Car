#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include <numeric>
#include <algorithm>
#include "mapping.h"
#include "constants.h"

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
    else if ((2 > new_d >= 1)){
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

#endif  // HELPERS_H