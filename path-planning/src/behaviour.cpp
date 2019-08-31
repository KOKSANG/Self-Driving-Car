#include <iostream>
#include <algorithm>
#include <map>
#include <math.h>
#include <string>
#include <vector>
#include "behaviour.h"
#include "trajectory.h"
#include "vehicle.h"
#include "mapping.h"
#include "cost_functions.h"
#include "jmt.h"
#include "helpers.h"
#include "constants.h"
#include "spline.h"

// for convenience
using std::string;
using std::vector;
using std::cout;
using std::endl;
using std::sort;
using std::min_element;
using std::map;

State::State(){}

State::State(string id, int lane){
    this->id = id;
    this->current_lane = lane;
    if ((this->id == "KL") || (this->id == "RDY")){
        this->intended_lane = this->current_lane;
        this->final_lane = this->current_lane;
        this->time_ahead = KEEP_LANE_CHANGE_TIME;
    }
    else if ((this->id != "PLCL") || (this->id != "PLCR")){
        this->time_ahead = KEEP_LANE_CHANGE_TIME;
        this->final_lane = this->current_lane;
        if (this->id == "PLCL"){
            this->intended_lane = this->current_lane - 1;
        }
        else if (this->id == "PLCR"){
            this->intended_lane = this->current_lane + 1;
        }
    }
    else if (this->id == "LCL"){
        this->intended_lane = this->current_lane - 1;
        this->final_lane = this->intended_lane;
        this->time_ahead = MAX_LANE_CHANGE_TIME;
    }
    else if (this->id == "LCR"){
        this->intended_lane = this->current_lane + 1;
        this->final_lane = this->intended_lane;
        this->time_ahead = MAX_LANE_CHANGE_TIME;
    }
}

State::~State(){}

Behaviour::Behaviour(){}

Behaviour::Behaviour(Vehicle* ego, double ref_vel){
    this->ego = ego;
    this->ref_vel = ref_vel;
    this->state = ego->state;
    this->current_timestep = 0;
}
// states machine
vector<State> Behaviour::available_states(){
    vector<State> states;
    State* current_state = this->ego->state;

    if (current_state->id == "RDY"){ 
        states = {State("RDY", this->ego->lane), State("KL", this->ego->lane)};
    }
    else if ((current_state->id == "KL")){ 
        states = {State("KL", this->ego->lane), State("PLCL", this->ego->lane), State("PLCR", this->ego->lane)};
    }
    else if (current_state->id == "PLCL"){
        if (this->ego->lane > 0){ states = {State("KL", this->ego->lane), State("PLCL", this->ego->lane), State("LCL", this->ego->lane)}; }
        else if (this->ego->lane == 0){ states = {State("KL", this->ego->lane), State("PLCR", this->ego->lane)}; }
    }
    else if (current_state->id == "PLCR"){
        if (this->ego->lane < 2){ states = {State("KL", this->ego->lane), State("PLCR", this->ego->lane), State("LCR", this->ego->lane)}; }
        else if (this->ego->lane == 2){ states = {State("KL", this->ego->lane), State("PLCL", this->ego->lane)}; }
    }
    else if ((current_state->id == "LCL") || (current_state->id == "LCR")){
        states = {State("KL", this->ego->lane), State("PLCL", this->ego->lane), State("PLCR", this->ego->lane)};
    }

    return states;
}

vector<vector<double>> Behaviour::forecast_points(State* state, vector<double> points_x, vector<double> points_y){
    int lane = state->intended_lane;
    double buffer_1 = BUFFER_RANGE;
    double buffer_2 = 2.0*BUFFER_RANGE;
    double buffer_3 = 3.0*BUFFER_RANGE;
    // Add Frenet of evenly spaced 30m
    vector<double> next_wp0 = this->map->getXY(this->ego->s + buffer_1, this->ego->d);
    vector<double> next_wp1 = this->map->getXY(this->ego->s + buffer_2, this->ego->d);
    vector<double> next_wp2 = this->map->getXY(this->ego->s + buffer_3, this->ego->d);
    // push net waypoints x
    points_x.push_back(next_wp0[0]);
    points_x.push_back(next_wp1[0]);
    points_x.push_back(next_wp2[0]);
    // push net waypoints y
    points_y.push_back(next_wp0[1]);
    points_y.push_back(next_wp1[1]);
    points_y.push_back(next_wp2[1]);
    // Shifting
    for (unsigned int i=0; i < points_x.size(); i++){
        // shift car reference angle to 0 degrees
        double shift_x = points_x[i] - this->ego->x;
        double shift_y = points_y[i] - this->ego->y;
        points_x[i] = shift_x*cos(0 - this->ego->yaw) - shift_y*sin(0 - this->ego->yaw);
        points_y[i] = shift_x*sin(0 - this->ego->yaw) + shift_y*cos(0 - this->ego->yaw);
    }
    return {points_x, points_y};
}

vector<Trajectory> Behaviour::generate_trajectory(vector<double> points_x, vector<double> points_y){
    vector<State> next_states = available_states();
    next_states = {State("KL", this->ego->lane)};
    vector<double> ptsx, ptsy;
    vector<vector<double>> points;
    vector<Trajectory> trajectories;
    Trajectory trajectory;

    for (auto& state: next_states){
        points = forecast_points(&state, points_x, points_y);
        ptsx = points[0];
        ptsy = points[1];
        tk::spline s;
        s.set_points(ptsx, ptsy);
        // Calculate how to break up spline points
        double target_x = 30.0;
        double target_y = s(target_x);
        double target_dist = sqrt(pow(target_x, 2)+pow(target_y, 2));
        double time_ahead = KEEP_LANE_CHANGE_TIME;
        if (state.id != "KL" && state.id != "RDY"){
            time_ahead = MAX_LANE_CHANGE_TIME;
        }
        double time_to_complete = time_ahead - this->ego->prev_x.size()*UPDATE_STEP_TIME;
        trajectory = Trajectory(this->ego, s, {target_x, target_y}, target_dist, this->ref_vel, time_to_complete);
        return {trajectory};
    }

}

/*
Trajectory Behaviour::get_best_trajectory(vector<Trajectory> trajectories, vector<Vehicle> const surrounding_vehicles){
    // Get sensor fusions and classify them into cars ahead, behind, left or right
    vector<vector<Vehicle>> surroundings;
    float cost;
    vector<float> traj_cost;
    for (Trajectory& traj: trajectories){
        surroundings = prepare_cost(this->ego, surrounding_vehicles, traj.time_to_complete);
        cost = traj.cost(surroundings);
        cout << "[ COST TRAJ ] - ";
        cout << "ID: " << traj.state->id;
        cout << ", Velocity: " << traj.target[1] << ", Acceleration: " << traj.target[2] << ", Cost: " << cost << endl;
        cout << "- - -" << endl;
        traj_cost.push_back(cost);
    }
    int best = min_element(traj_cost.begin(), traj_cost.end()) - traj_cost.begin();
    cout << "[..BEST TRAJ..] - ";
    cout << "Index: " << best << ", ID: " << trajectories[best].state->id << ", Position: " << trajectories[best].target[0];
    cout << ", Velocity: " << trajectories[best].target[1] << ", Acceleration: " << trajectories[best].target[2] << endl;
    return trajectories[best];
}
*/

Behaviour::~Behaviour(){}