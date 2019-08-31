#include <iostream>
#include <math.h>
#include <string>
#include <vector>
#include <utility>
#include <algorithm>
#include "trajectory.h"
#include "helpers.h"
#include "behaviour.h"
#include "vehicle.h"
#include "mapping.h"
#include "constants.h"
#include "spline.h"

// for convenience
using std::string;
using std::vector;
using std::abs;
using std::cout;
using std::sort;

float lane_speed(vector<vector<Vehicle>> const surroundings, int lane, double s){
    // To get driving speed in selected lane
    float speed = MAX_SPEED_MS;
    for (int i = 0; i < surroundings.size(); i++){
        if (!surroundings[i].empty() && surroundings[i][0].lane == lane){
            // If found vehicle in the lane
            double max_distance_with_ego = 2*BUFFER_RANGE;
            double distance_with_ego = fabs(surroundings[i][0].s - s);
            if (surroundings[i][0].s > s && distance_with_ego <= max_distance_with_ego){
                // Only get the speed of closest vehicle
                speed = surroundings[i][0].speed;
                return speed;
            }
            else continue;
        }
        else continue;
    }
    // If found no vehicle in the lane
    return speed;
}

vector<vector<Vehicle>> prepare_cost(Vehicle* ego, vector<Vehicle> const others, double T){
    vector<vector<Vehicle>> surroundings;
    vector<Vehicle> vehicles_ahead = ego->ahead(others, T);
    vector<Vehicle> vehicles_behind = ego->behind(others, T);
    vector<Vehicle> vehicles_left = ego->side(others, T, 'L');
    vector<Vehicle> vehicles_right = ego->side(others, T, 'R');
    surroundings = {vehicles_ahead, vehicles_behind, vehicles_left, vehicles_right};
    return surroundings;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                    /////     Legality Cost      /////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
float subcost_Speed(Trajectory* trajectory, vector<vector<Vehicle>> const surroundings, double T, double weight){
    // cost function for maintaining highest speed/ speed limit possible
    float target_speed = MAX_SPEED_MS;
    float cost;
    double final_speed = trajectory->target[1];
    if (final_speed >= target_speed) cost = PUNISHMENT_MAX_COST;
    else if (final_speed <= target_speed) cost = 1 - (final_speed/ target_speed);

    return cost;
}

float subcost_OutOfLane(Trajectory* trajectory, vector<vector<Vehicle>> const surroundings, double T, double weight){
    float cost = 0;
    for (auto& d: trajectory->kinematics_d){
        if (d[0] <= 0){
            cost = PUNISHMENT_MAX_COST;
            return cost;
        }
    }
    return cost;
}

float costfunc_Legality(Trajectory* trajectory, vector<vector<Vehicle>> const surroundings, double T, double weight){
    float cost_1 = subcost_Speed(trajectory, surroundings, T, weight);
    float cost_2 = subcost_OutOfLane(trajectory, surroundings, T, weight);
    double cost = (cost_1 + cost_2)/2;
    return cost;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                    /////     Efficiency Cost      /////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
float subcost_LaneChange(Trajectory* trajectory, vector<vector<Vehicle>> const surroundings, double T, double weight){
    // Cost function for lane changing
    State* state = trajectory->state;
    int start_lane = state->current_lane;
    int intended_lane = state->intended_lane;
    int final_lane = state->final_lane;
    double next_ego_s = trajectory->kinematics_s.back()[0];
    float distance_to_goal = abs(TRACK_LENGTH - next_ego_s);
    float cost;

    float distance = start_lane - final_lane + start_lane - intended_lane;
    float exponent = exp(-(abs(distance)/ distance_to_goal));
    if ((state->intended_lane < 0) || (state->intended_lane > 2)) cost = PUNISHMENT_MAX_COST;
    else cost = 1 - exponent;

    return cost;
}

float subcost_SpeedChange(Trajectory* trajectory, vector<vector<Vehicle>> const surroundings, double T, double weight){
    // Penalize negative speed change
    State* state = trajectory->state;
    int start_lane = state->current_lane;
    int intended_lane = state->intended_lane;

    float target_speed  = MAX_SPEED_MS;
    float current_speed = trajectory->ego->speed;
    float new_speed = lane_speed(surroundings, intended_lane, trajectory->ego->s);
    float speed_change = new_speed - current_speed;

    float cost = (2.0*target_speed - current_speed - new_speed)/ (2.0*target_speed);
    //cout << ">>>> Speed: " << target_speed << ", " << current_speed << ", " << new_speed << endl;
    return cost;
}

float costfunc_Efficiency(Trajectory* trajectory, vector<vector<Vehicle>> const surroundings, double T, double weight){
    float cost_1 = subcost_LaneChange(trajectory, surroundings, T, weight);
    float cost_2 = subcost_SpeedChange(trajectory, surroundings, T, weight);
    float cost = (cost_1 + cost_2)/ 2;
    //cout << "---- Efficiency cost: " << cost_1 << ", " << cost_2 << endl;
    return cost;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

// TODO: cost functions for acceleration and jerk

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                    /////     Safety Cost      /////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
float subcost_BufferDistance(Trajectory* trajectory, vector<vector<Vehicle>> const surroundings, double T, double weight){
    // cost function for buffering a distance with other surroundings vehicles
    double buffer = BUFFER_RANGE;
    double next_ego_s = trajectory->kinematics_s.back()[0];
    vector<Vehicle> ahead = surroundings[0];
    vector<Vehicle> behind = surroundings[1];
    float cost_1, cost_2, cost;

    if (!ahead.empty()){
        if (next_ego_s == ahead[0].s) cost_1 = PUNISHMENT_MAX_COST;
        else cost_1 = exp(-((ahead[0].s - next_ego_s)/ buffer));
    }
    else if (ahead.empty()) cost_1 = 0;

    if (!behind.empty()){
        if (next_ego_s == ahead[0].s) cost_2 = PUNISHMENT_MAX_COST;
        else cost_2 = exp(-((next_ego_s - behind[0].s)/ buffer));
    }
    else if (behind.empty()) cost_2 = 0;

    cost = (cost_1 + cost_2)/2;
    return cost;
}

float subcost_LongitudinalCollision(Trajectory* trajectory, vector<vector<Vehicle>> const surroundings, double T, double weight){
    double buffer = BUFFER_RANGE;
    vector<Vehicle> vehicles_ahead = surroundings[0];
    vector<Vehicle> vehicles_behind = surroundings[1];
    double next_ego_s = trajectory->kinematics_s.back()[0];
    float cost_1, cost_2, cost;
    // distance, speed, predicted position
    if (!vehicles_ahead.empty()){
        if (next_ego_s == vehicles_ahead[0].s) cost_1 = PUNISHMENT_MAX_COST;
        else cost_1 = exp(-((vehicles_ahead[0].s - next_ego_s)/ buffer));
    }
    else if (vehicles_ahead.empty()) cost_1 = 0;

    if (!vehicles_behind.empty()){
        if (next_ego_s == vehicles_behind[0].s) cost_2 = PUNISHMENT_MAX_COST;
        else cost_2 = exp(-((next_ego_s - vehicles_behind[0].s)/ buffer));
    }
    else if (vehicles_behind.empty()) cost_2 = 0;

    cost = (cost_1 + cost_2)/2;
    return cost;
}

float subcost_LatitudinalCollision(Trajectory* trajectory, vector<vector<Vehicle>> const surroundings, double T, double weight, char mode){
    int start_lane = trajectory->state->current_lane;
    int intended_lane = trajectory->state->intended_lane;
    double buffer = BUFFER_RANGE*1.2;
    vector<Vehicle> vehicles_left = surroundings[2];
    vector<Vehicle> vehicles_right = surroundings[3];
    float total_distance = 0;
    int vehicles_count = 0;
    float cost;
    double next_ego_x = trajectory->x.back();
    double next_ego_y = trajectory->y.back();

    // distance, speed, predicted position
    if (mode == 'L'){
        if (!vehicles_left.empty()){
            for (Vehicle &v: vehicles_left){
                double distance = get_distance(next_ego_x, next_ego_y, v.x, v.y);
                if (distance >= buffer){
                    vehicles_count++;
                    total_distance += distance;
                }
            }
        }
        else if (vehicles_left.empty()){
            cost = 0;
            return cost;
        }
    }

    else if (mode == 'R'){
        if (!vehicles_right.empty()){
            for (Vehicle &v: vehicles_right){
                double distance = get_distance(next_ego_x, next_ego_y, v.x, v.y);
                if (distance >= buffer){
                    vehicles_count++;
                    total_distance += distance;
                }
            }
        }
        else if (vehicles_right.empty()){
            cost = 0;
            return cost;
        }
    }
    cost = exp(-(total_distance/(vehicles_count*buffer)));
    return cost;
}

float costfunc_Safety(Trajectory* trajectory, vector<vector<Vehicle>> const surroundings, double T, double weight){
    vector<Vehicle> vehicles_ahead = surroundings[0];
    vector<Vehicle> vehicles_behind = surroundings[1];
    float lat_cost, long_cost, cost_buffer, cost;
    char mode;

    if (trajectory->state->id == "KL"){
        lat_cost = 0;
        long_cost = subcost_LongitudinalCollision(trajectory, surroundings, T, weight);
    }
    else if (trajectory->state->id != "KL"){
        if (trajectory->state->id.back() == 'L'){ mode = 'L'; }
        else if (trajectory->state->id.back() == 'R'){ mode = 'R'; }
        lat_cost = subcost_LatitudinalCollision(trajectory, surroundings, T, weight, mode);
        long_cost = subcost_LongitudinalCollision(trajectory, surroundings, T, weight);
    }

    //cost_buffer = subcost_BufferDistance(trajectory, surroundings, T, weight);
    cost = (lat_cost + long_cost)/ 2;
    //cout << "safety cost: " << lat_cost << ", " << long_cost << endl;
    return cost;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                    /////     Comfort Cost      /////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
float subcost_Acceleration(Trajectory *trajectory, double weight){
    int kinematics_size = trajectory->kinematics_s.size();
    double acc_s, acc_d;
    float cost = 0;

    for (int i=0; i < kinematics_size; i++){
        acc_s = trajectory->kinematics_s[i][2];
        acc_d = trajectory->kinematics_d[i][2];

        if (acc_s >= MAX_ACCELERATION || acc_d >= MAX_ACCELERATION){
            //cout << "[ CHECK ] - s: " << trajectory->kinematics_s[i][0] << ", v: " << trajectory->kinematics_s[i][1] << ", acc: " << trajectory->kinematics_s[i][2] << ", jerk: " << trajectory->kinematics_s[i][3] << endl;   
            cost = PUNISHMENT_MAX_COST;
            return cost;
        }
    }

    return cost;
}

float subcost_Jerk(Trajectory* trajectory, double weight){
    int kinematics_size = trajectory->kinematics_s.size();
    double jerk_s, jerk_d;
    float cost = 0;

    for (int i=0; i < kinematics_size; i++){
        jerk_s = trajectory->kinematics_s[i][3];
        jerk_d = trajectory->kinematics_d[i][3];

        if (jerk_s >= MAX_JERK || jerk_d >= MAX_JERK){
            cost = PUNISHMENT_MAX_COST;
            return cost;
        }
    }

    return cost;
}

float costfunc_Comfort(Trajectory* trajectory, double weight){
    float cost_1, cost_2, cost;
    cost_1 = subcost_Acceleration(trajectory, weight);
    cost_2 = subcost_Acceleration(trajectory, weight);
    cost = (cost_1 + cost_2)/ 2;
    return cost;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////