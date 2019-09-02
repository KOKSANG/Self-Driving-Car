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

float lane_speed(vector<vector<Vehicle>> surroundings, int lane, double s){
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
        }
    }
    // If found no vehicle in the lane
    return speed;
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                    /////     Rules Cost      /////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
float subcost_Speed(Trajectory* trajectory){
    // cost function for maintaining highest speed/ speed limit possible
    float target_speed = MAX_SPEED_MILES;
    float cost = 0;
    double final_speed = trajectory->ref_vel;
    if (final_speed >= target_speed) cost = MAX_COST;
    else if (final_speed < target_speed) cost = 1 - (final_speed/ target_speed);
    return cost;
}

float subcost_Acceleration(double prev_vel, double final_vel, double T){
    double acc = (final_vel*0.44704 - prev_vel*0.44704)/ T;
    float cost = 0;
    if (acc > MAX_ACCELERATION) cost = MAX_COST;
    return cost;
}

float subcost_Lane(Trajectory* trajectory){
    int lane = trajectory->state->final_lane;
    float cost = 0;
    if (lane < 0 || lane > 2){
        cost = MAX_COST;
    }
    return cost;
}

float costfunc_Rules(Trajectory* trajectory, double prev_vel, double final_vel, double T){
    float cost_1 = subcost_Speed(trajectory);
    float cost_2 = subcost_Acceleration(prev_vel, final_vel, T);
    //float cost_3 = subcost_Lane(trajectory);
    //cout << "Rules cost - cost1: " << cost_1 << ", cost2: " << cost_2 << endl;
    double cost = (cost_1 + cost_2)/2;
    return cost;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                    /////     Efficiency Cost      /////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
float subcost_LaneChange(Trajectory* trajectory){
    // Cost function for lane changing
    State* state = trajectory->state;
    int start_lane = state->current_lane;
    int intended_lane = state->intended_lane;
    int final_lane = state->final_lane;
    double ego_s = trajectory->ego->s;
    float distance_to_goal = abs(TRACK_LENGTH - ego_s);
    float cost;

    float distance = start_lane - final_lane + start_lane - intended_lane;
    float exponent = exp(-(abs(distance)/ distance_to_goal));
    if ((intended_lane < 0) || (intended_lane > 2)) cost = MAX_COST;
    else cost = 1 - exponent;

    return cost;
}

float subcost_SpeedChange(Trajectory* trajectory, vector<vector<Vehicle>> surroundings){
    // Penalize negative speed change
    State* state = trajectory->state;
    int start_lane = state->current_lane;
    int final_lane = state->final_lane;

    float target_speed  = MAX_SPEED_MS;
    float current_speed = trajectory->ego->speed;
    float new_speed = lane_speed(surroundings, final_lane, trajectory->ego->s);// + trajectory->target_acc*trajectory->time_to_complete;
    float speed_change = new_speed - current_speed;

   // cout << "(-- SPEED --) - new: " << new_speed << ", current: " << current_speed << ", target: " << target_speed << endl;

    float cost = (2.0*target_speed - current_speed - new_speed)/ (2.0*target_speed);
    return cost;
}

float costfunc_Efficiency(Trajectory* trajectory, vector<vector<Vehicle>> surroundings){
    float cost_1 = subcost_LaneChange(trajectory);
    float cost_2 = subcost_SpeedChange(trajectory, surroundings);
    float cost = (cost_1 + cost_2)/ 2;
    //cout << "[EFFICIENCY ] > cost1: " << cost_1 << ", cost2: " << cost_2 << endl;
    return cost;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                    /////     Safety Cost      /////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
float subcost_Buffer(Trajectory* trajectory, Vehicle* next_ego, vector<vector<Vehicle>> surroundings){
    double buffer = BUFFER_RANGE;
    vector<Vehicle> vehicles_ahead = surroundings[0];
    vector<Vehicle> vehicles_behind = surroundings[1];
    double max_cost = 1 + trajectory->target_acc*SPEED_INCREMENT;
    double ego_s = trajectory->ego->s;
    float cost = numeric_limits<float>::min();
    // distance, speed, predicted position
    if (!vehicles_ahead.empty()){
        //cout << "Ahead s: " << vehicles_ahead[0].s << ", ego s: " << ego_s << endl;
        for (auto& v: vehicles_ahead){
            float distance = fabs(ego_s - vehicles_ahead[0].s);
            if (distance <= buffer){
                if (trajectory->target_acc >=0) cost = 1;
                else cost = 0.9;
            }
            else continue;
        }
    }
    //if (cost != 0) cost = (cost + trajectory->target_acc*SPEED_INCREMENT)/ max_cost;

    return cost;
}

float subcost_LatitudinalCollision(Trajectory* trajectory, Vehicle* next_ego, vector<vector<Vehicle>> surroundings, double T, char mode){
    vector<Vehicle> vehicles_left = surroundings[2];
    vector<Vehicle> vehicles_right = surroundings[3];
    vector<Vehicle> collected;
    int start_lane = trajectory->state->current_lane;
    int intended_lane = trajectory->state->intended_lane;
    double next_ego_s = next_ego->s;
    double buffer = BUFFER_RANGE*1.2;
    float total_distance = 0;
    float cost = 0;

    switch (mode){
        case 'L':
            if (!vehicles_left.empty()){
                if (vehicles_left.size() > 3) vehicles_left = {vehicles_left[0], vehicles_left[1], vehicles_left[2]};//, vehicles_left[4]};
                for (Vehicle &v: vehicles_left){
                    v = v.predict_position(T);
                    double distance_s = fabs(next_ego_s - v.s);
                    if (distance_s <= buffer){
                        if (trajectory->target_acc >= 0) return 0.9;
                        else return MAX_COST;
                        //collected.push_back(v);
                        //total_distance += distance_s;
                    }
                }
            }
            else if (vehicles_left.empty()) return cost;
        break;

        case 'R':
            if (!vehicles_right.empty()){
                if (vehicles_right.size() > 3) vehicles_right = {vehicles_right[0], vehicles_right[1], vehicles_right[2]};//, vehicles_right[3]};
                for (Vehicle &v: vehicles_right){
                    v = v.predict_position(T);
                    double distance_s = fabs(next_ego_s - v.s);
                    if (distance_s <= buffer){
                        if (trajectory->target_acc >= 0) return 0.9;
                        else return MAX_COST;
                        //collected.push_back(v);
                        //total_distance += distance_s;
                    }
                }
            }
            else if (vehicles_right.empty()) return cost;
        break;
    }

    //if (!collected.empty()) cost = exp(-(total_distance/(collected.size()*buffer)));

    //return cost;
}


float costfunc_Safety(Trajectory* trajectory, Vehicle* next_ego, vector<vector<Vehicle>> surroundings, double T){
    vector<Vehicle> vehicles_ahead = surroundings[0];
    vector<Vehicle> vehicles_behind = surroundings[1];
    //Vehicle* next_ego = next_ego;//trajectory->ego->next_ego(trajectory);
    float lat_cost, buffer_cost, cost_buffer, cost;
    string RDY("RDY"), KL("KL"), PLCL("PLCL"), PLCR("PLCR"), LCL("LCL"), LCR("LCR");
    char mode;

    if (trajectory->state->id.compare(KL)==0){
        lat_cost = 0;//subcost_LatitudinalCollision(trajectory, next_ego, surroundings, T, mode);
        buffer_cost = subcost_Buffer(trajectory, next_ego, surroundings);
        cost = buffer_cost;
    }
    else {
        if (trajectory->state->id.back() == 'L') mode = 'L';
        else if (trajectory->state->id.back() == 'R') mode = 'R';

        if (trajectory->state->id.compare(PLCL)==0 || trajectory->state->id.compare(PLCR)==0){
            lat_cost = subcost_LatitudinalCollision(trajectory, next_ego, surroundings, T, mode);
            buffer_cost = subcost_Buffer(trajectory, next_ego, surroundings);
            cost = (buffer_cost + lat_cost)/ 2;
        }

        else if (trajectory->state->id.compare(LCL)==0 || trajectory->state->id.compare(LCR)==0){
            lat_cost = subcost_LatitudinalCollision(trajectory, next_ego, surroundings, T, mode);
            buffer_cost = subcost_Buffer(trajectory, next_ego, surroundings);
            cost = (buffer_cost + lat_cost)/ 2;
        }
    }

    //cout << "(..SAFETY...) - lat cost: " << lat_cost << ", buffer cost: " << buffer_cost << endl;
    return cost;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////