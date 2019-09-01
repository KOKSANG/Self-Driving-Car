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
#include "constants.h"

// for convenience
using tk::spline;
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
    string RDY("RDY"), KL("KL"), PLCL("PLCL"), PLCR("PLCR"), LCL("LCL"), LCR("LCR");
    this->current_lane = lane;

    if (this->id.compare(KL)==0 || this->id.compare(RDY)==0){
        this->intended_lane = lane;
        this->final_lane = lane;
        this->time_ahead = KEEP_LANE_CHANGE_TIME;
    }
    else if (this->id.compare(PLCL)==0){
        this->intended_lane = lane - 1;
        this->final_lane = lane;
        this->time_ahead = KEEP_LANE_CHANGE_TIME;
    }
    else if (this->id.compare(PLCR)==0){
        this->intended_lane = lane + 1;
        this->final_lane = lane;
        this->time_ahead = KEEP_LANE_CHANGE_TIME;
    }
    else if (this->id.compare(LCL)==0){
        this->intended_lane = lane - 1;
        this->final_lane = lane - 1;
        this->time_ahead = MAX_LANE_CHANGE_TIME;
    }
    else if (this->id.compare(LCR)==0){
        this->intended_lane = lane + 1;
        this->final_lane = lane + 1;
        this->time_ahead = MAX_LANE_CHANGE_TIME;
    }
}

State::~State(){}

Behaviour::Behaviour(){}

Behaviour::Behaviour(Vehicle* ego, double ref_vel){
    this->ego = ego;
    this->map = ego->map;
    this->ref_vel = ref_vel;
    this->state = ego->state;
    this->current_timestep = 0;
}
// states machine
vector<State> Behaviour::available_states(){
    vector<State> states;
    State* current_state = this->ego->state;

    if (current_state->id == "RDY") states = {State("RDY", this->ego->lane), State("KL", this->ego->lane)};
    else if ((current_state->id == "KL")) states = {State("KL", this->ego->lane), State("PLCL", this->ego->lane), State("PLCR", this->ego->lane)};
    else if (current_state->id == "PLCL"){
        if (this->ego->lane > 0) states = {State("KL", this->ego->lane), State("PLCL", this->ego->lane), State("LCL", this->ego->lane)};
        else if (this->ego->lane == 0) states = {State("KL", this->ego->lane), State("PLCR", this->ego->lane)};
    }
    else if (current_state->id == "PLCR"){
        if (this->ego->lane < 2) states = {State("KL", this->ego->lane), State("PLCR", this->ego->lane), State("LCR", this->ego->lane)};
        else if (this->ego->lane == 2) states = {State("KL", this->ego->lane), State("PLCL", this->ego->lane)};
    }
    else if (current_state->id == "LCL") states = {State("KL", this->ego->lane)};
    else if (current_state->id == "LCR") states = {State("KL", this->ego->lane)};
    
    return states;
}

vector<vector<double>> Behaviour::forecast_points(State* state, vector<double> points_x, vector<double> points_y){
    int lane = state->intended_lane;
    double buffer_1 = BUFFER_RANGE;
    double buffer_2 = 2.0*BUFFER_RANGE;
    double buffer_3 = 3.0*BUFFER_RANGE;
    double final_d = MIN_D+this->state->final_lane*LANE_WIDTH;
    // Add Frenet of evenly spaced 30m
    vector<double> next_wp0 = this->map->getXY(this->ego->s + buffer_1, final_d);
    vector<double> next_wp1 = this->map->getXY(this->ego->s + buffer_2, final_d);
    vector<double> next_wp2 = this->map->getXY(this->ego->s + buffer_3, final_d);
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

Trajectory Behaviour::get_best_trajectory(vector<double> points_x, vector<double> points_y){
    vector<State> next_states = available_states();
    //vector<State> next_states = {State("KL", 0)};
    vector<double> next_acc = {1, 0, -1};
    vector<double> ptsx, ptsy;
    vector<vector<double>> points;
    vector<Trajectory> trajectories;
    Trajectory trajectory;
    Trajectory best_trajectory;
    float cost = numeric_limits<float>::max();

    for (auto& state: next_states){
        cout << "[   STATE   ] >>>>>>> id: " << state.id << ", current lane: " << state.current_lane << ", intended: " << state.intended_lane << ", final: " << state.final_lane << endl; 
        points = forecast_points(&state, points_x, points_y);
        ptsx = points[0];
        ptsy = points[1];
        tk::spline spline;
        spline.set_points(ptsx, ptsy);
        // Calculate how to break up spline points
        double target_x = 30.0;
        double target_y = spline(target_x);
        double target_dist = sqrt(pow(target_x, 2)+pow(target_y, 2));
        double time_ahead = state.time_ahead;
        double time_to_complete = time_ahead - this->ego->prev_x.size()*UPDATE_STEP_TIME;

        for (double& target_acc: next_acc){
            double prev_vel = this->ref_vel;
            trajectory = Trajectory(this->ego, &state, spline, {target_x, target_y, target_dist, target_acc}, this->ref_vel, time_to_complete);
            trajectory.generate(this->map);
            double final_vel = trajectory.ref_vel;
            float traj_cost = trajectory.cost(prev_vel, final_vel);
            if (traj_cost < cost){
                best_trajectory = trajectory;
                cost = traj_cost;
            }
        }
    }
    
    return best_trajectory;
}

Behaviour::~Behaviour(){}