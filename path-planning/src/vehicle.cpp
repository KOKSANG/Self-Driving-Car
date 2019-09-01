#include <iostream>
#include <math.h>
#include <utility>
#include <algorithm>
#include "vehicle.h"
#include "trajectory.h"
#include "constants.h"

using std::sort;
using namespace std;

bool Vehicle::sort_distance_to_ego(const Vehicle& left, const Vehicle& right){
    return get_distance(this->x, this->y, left.x, left.y) < get_distance(this->x, this->y, right.x, right.y);
}

bool Vehicle::sort_increment(const Vehicle& left, const Vehicle& right){
    return abs(this->s - left.s) < abs(this->s - right.s);
}

bool Vehicle::sort_decrement(const Vehicle& left, const Vehicle& right){
    return abs(this->s - left.s) > abs(this->s - right.s);
}

Vehicle::Vehicle(){}
Vehicle::Vehicle(int id, double x, double y, double vx, double vy, double s, double d, Mapping* map){
    this->id = id;
    this->x = x;
    this->y = y;
    this->vx = vx;
    this->vy = vy;
    this->s = s;
    this->d = d;
    this->map = map;
    this->theta = get_theta(vx, vy);
    this->lane = d2lane(this->d);
    this->speed = get_speed();
}

Vehicle::Vehicle(int id, double x, double y, double s, double d, double yaw, double speed,
                    vector<double> prev_x, vector<double> prev_y, vector<Vehicle> surroundings, State* state, Mapping* map){
    this->id = id;
    this->x = x;
    this->y = y;
    this->s = s;
    this->d = recalibrate_d(d);
    this->yaw = yaw;
    this->prev_x = prev_x;
    this->prev_y = prev_y;
    this->state = state;
    this->map = map;
    this->lane = d2lane(this->d);
    this->speed = miles2m(speed);
    // Sort all surroundings vehicles into left, right, ahead or behind
    vector<vector<Vehicle>> sorted_surroundings = sort_vehicles(surroundings);
    this->vehicles_ahead = sorted_surroundings[0];
    this->vehicles_behind = sorted_surroundings[1];
    this->vehicles_left = sorted_surroundings[2];
    this->vehicles_right = sorted_surroundings[3];
}

Vehicle::Vehicle(int id, double x, double y, double s, double d, double yaw){
    this->id = id;
    this->x = x;
    this->y = y;
    this->s = s;
    this->d = recalibrate_d(d);
    this->yaw = yaw;
 }

double Vehicle::get_speed() const {
    return sqrt(this->vx * this->vx + this->vy * this->vy);
}

vector<vector<Vehicle>> Vehicle::sort_vehicles(vector<Vehicle> surroundings){
    vector<Vehicle> vehicles_ahead, vehicles_behind, vehicles_left, vehicles_right;
    int left_lane = this->lane - 1;
    int right_lane = this->lane + 1;
    for (Vehicle& v: surroundings){
        if (v.lane == this->lane){
            if (v.s >= this->s) vehicles_ahead.push_back(v);
            else vehicles_behind.push_back(v);
        }
        else {
            if (v.lane == left_lane) vehicles_left.push_back(v);
            else if (v.lane == right_lane) vehicles_right.push_back(v);
        }
    }
    if (vehicles_ahead.size() > 0) sort(vehicles_ahead.begin(), vehicles_ahead.end(), [this](Vehicle left, Vehicle right){return sort_increment(left, right);});
    if (vehicles_behind.size() > 0) sort(vehicles_behind.begin(), vehicles_behind.end(), [this](Vehicle left, Vehicle right){return sort_decrement(left, right);});
    if (vehicles_behind.size() > 0) sort(vehicles_behind.begin(), vehicles_behind.end(), [this](Vehicle left, Vehicle right){return sort_decrement(left, right);});
    if (vehicles_behind.size() > 0) sort(vehicles_behind.begin(), vehicles_behind.end(), [this](Vehicle left, Vehicle right){return sort_decrement(left, right);});

    vector<vector<Vehicle>> sorted_vehicles = {vehicles_ahead, vehicles_behind, vehicles_left, vehicles_right};

    return sorted_vehicles;
}

Vehicle Vehicle::predict_position(double t){
    double new_x = this->x + (this->vx * t);
    double new_y = this->y + (this->vy * t);
    double theta = atan2(new_x - this->y, new_y - this->x);
    vector<double> frenet = this->map->getFrenet(new_x, new_y, theta);
    return Vehicle(this->id, new_x, new_y, this->vx, this->vy, frenet[0], frenet[1], this->map);
}

Vehicle Vehicle::next_ego(Trajectory* traj){
    double new_x = traj->points_x.back();
    double new_y = traj->points_y.back();
    double theta = atan2(new_x - this->x, new_y - this->y);
    vector<double> frenet = this->map->getFrenet(new_x, new_y, theta);
    return Vehicle(this->id, new_x, new_y, frenet[0], frenet[1], theta);
}

Vehicle::~Vehicle() {}