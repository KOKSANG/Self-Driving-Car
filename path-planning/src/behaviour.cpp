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
        this->time_ahead = 1;
    }
    else if ((this->id != "PLCL") || (this->id != "PLCR")){
        this->time_ahead = 1;
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
    this->current_timestep = 0;
    vector<State> states;
    vector<string> states_id;
    states.push_back(State("RDY", this->ego->lane));
    states.push_back(State("KL", this->ego->lane));
    states.push_back(State("PLCL", this->ego->lane));
    states.push_back(State("PLCR", this->ego->lane));
    states.push_back(State("LCL", this->ego->lane));
    states.push_back(State("LCR", this->ego->lane));
    this->states = states;
}

// states machine
vector<State> Behaviour::available_states(){
    vector<State> states;
    State current_state = *this->ego->state;

    if (current_state.id == "RDY"){
        states = {this->states[0], this->states[1]};
    }
    else if ((current_state.id == "KL")){
        states = {this->states[1], this->states[2], this->states[3]};
    }
    else if (current_state.id == "PLCL"){
        if (this->ego->lane > 0){ states = {this->states[1], this->states[2], this->states[4]}; }
        else if (this->ego->lane == 0){ states = {this->states[1], this->states[3]}; }
    }
    else if (current_state.id == "PLCR"){
        if (this->ego->lane < 2){ states = {this->states[1], this->states[3], this->states[5]}; }
        else if (this->ego->lane == 2){ states = {this->states[1], states[2]}; }
    }
    else if ((current_state.id == "LCL") || (current_state.id == "LCR")){
        states = {this->states[1], this->states[2], this->states[3]};
    }
    return states;
}

vector<Trajectory> Behaviour::generate_trajectory(vector<double> x, vector<double> y){
    // To get available states of current state
    vector<State> states = available_states();
    vector<Trajectory> trajectories;
    Trajectory trajectory;
    // start conditions
    vector<double> k = this->ego->kinematics;
    vector<double> start = {k[0], k[1], k[2], k[3], k[4], k[5]};
    // To generate max number of trajectories up to 3 using different acceleration
    vector<double> different_acc = {MAX_ACCELERATION, 0, -MAX_ACCELERATION};
    // Making kinematics of ego
    vector<double> ego_kinematics = {start[0], start[1], start[2]};
    vector<double> target;
    vector<vector<double>> kinematics_s, kinematics_d;
    double target_s, target_sd, target_sdd;

    cout << "[    STATE    ] - Current state: " << this->ego->state->id << " | Lane: " << this->ego->state->current_lane;
    cout << " | Next state: ";
    for (int i=0; i<states.size(); i++){ cout << states[i].id << " "; }
    cout << endl;
    
    for (auto& next_state: states){
        // Intended lane of the state
        double final_d = lane2d(next_state.final_lane);
        // Get the remaining time to complete for the trajectory
        double time_to_complete = (next_state.time_ahead*MAX_POINTS_PER_SEC - x.size())*PATH_TIMESTEP;
        for (auto& acc: different_acc){
            // To estimate future/ final kinematics of ego if it travels with this certain acceleration
            //target_s = calculate_final_kinematics(ego_kinematics, acc, next_state.time_ahead*MAX_POINTS_PER_SEC*PATH_TIMESTEP);

            target_sd = ego_kinematics[1] + acc*next_state.time_ahead;
            target_sdd = 0;
            target_s = ego_kinematics[0] + ((ego_kinematics[1] + target_sd)/2)*next_state.time_ahead;
            // Target conditions
            target = {target_s, target_sd, target_sdd, final_d, 0, 0};
            // Create new trajectory object
            trajectory = Trajectory(&next_state, this->ego, start, target, x, y, time_to_complete);
            trajectory.generate();
            trajectories.push_back(trajectory);
        }
    }
    return trajectories;
}

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


Behaviour::~Behaviour(){}