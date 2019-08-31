#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <numeric>
#include "mapping.h"
#include "spline.h"
#include "helpers.h"
#include "constants.h"

// for convenience
using std::string;
using std::vector;
using tk::spline;

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = get_distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

spline spline_s;
spline Mapping::spline_x = spline_s;
spline Mapping::spline_y = spline_s;
spline Mapping::spline_dy = spline_s;
spline Mapping::spline_dx = spline_s;

Mapping::Mapping(vector<double> waypoints_s, vector<double> waypoints_x, vector<double> waypoints_y,
         vector<double> waypoints_dx, vector<double> waypoints_dy, double separation,
         double length){

    this->interval = separation;    // interpolation separation
    this->track_length = length;      // track length
    this->points = round(this->track_length/ this->interval);
    for (int i = 0; i < this->points; i++){
        this->map_s.push_back(waypoints_s[0] + i * this->interval);
    }
    this->map_x = interpolate_points(1, waypoints_s, waypoints_x);
    this->map_y = interpolate_points(2, waypoints_s, waypoints_y);
    this->map_dx = interpolate_points(3, waypoints_s, waypoints_dx);
    this->map_dy = interpolate_points(4, waypoints_s, waypoints_dy);
}

vector<double> Mapping::interpolate_points(int mode, vector<double> waypoints_s, vector<double> waypoints){
  /* Function to interpolate waypoints using spline
  Inputs:
    (int) mode: integers representing d, x, dx or dy waypoints
    (vector<double>) waypoints: the waypoints to interpolate
   */
  spline s;
  s.set_points(waypoints_s, waypoints);
  switch (mode) {
    case 1:
      this->spline_x = s;
      break;
    case 2:
      this->spline_y = s;
      break;
    case 3:
      this->spline_dx = s;
      break;
    case 4:
      this->spline_dy = s;
      break;
  }

  vector<double> output;
  for (int i = 0; i < this->points; i++) {
    output.push_back(s(waypoints_s[0] + i * this->interval));
  }
  return output;
}

vector<double> Mapping::getXY(double s, double d){
  /* Function to get X,Y coordinates from s, d using spline
  Inputs:
    (double) s: s coord
    (double) d: d coord
  Returns:
    (vector<double>) {x coord, y coord}
   */
  s = fmod(s, this->track_length);
  double x = this->spline_x(s) + (d * this->spline_dx(s));
  double y = this->spline_y(s) + (d * this->spline_dy(s));
  return {x, y};
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> Mapping::getFrenet(double x, double y, double theta) {
  int next_wp = NextWaypoint(x, y, theta, this->map_x, this->map_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = this->map_x.size()-1;
  }

  double n_x = this->map_x[next_wp]-this->map_x[prev_wp];
  double n_y = this->map_y[next_wp]-this->map_y[prev_wp];
  double x_x = x - this->map_x[prev_wp];
  double x_y = y - this->map_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = get_distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-this->map_x[prev_wp];
  double center_y = 2000-this->map_y[prev_wp];
  double centerToPos = get_distance(center_x,center_y,x_x,x_y);
  double centerToRef = get_distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += get_distance(this->map_x[i],this->map_y[i],this->map_x[i+1],this->map_y[i+1]);
  }

  frenet_s += get_distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

Mapping::~Mapping() {}