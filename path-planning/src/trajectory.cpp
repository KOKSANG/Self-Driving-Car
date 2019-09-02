#include <iostream>
#include <math.h>
#include <string>
#include <vector>
#include "behaviour.h"
#include "trajectory.h"
#include "vehicle.h"
#include "mapping.h"
#include "cost_functions.h"
#include "constants.h"

// for convenience
using std::string;
using std::vector;
using std::cout;
using std::sort;

tk::spline traj_spline;
spline Trajectory::spline = traj_spline;

Trajectory::Trajectory(){}

Trajectory::Trajectory(Vehicle* ego, State* state, tk::spline s, vector<double> target, double ref_vel, double time_to_complete){
    this->ego = ego;
    this->state = state;
    this->spline = s;
    this->target_x = target[0];
    this->target_y = target[1];
    this->target_distance = target[2];
    this->target_acc = target[3];
    this->ref_vel = ref_vel;
    this->time_to_complete = time_to_complete;
    this->step_to_complete = this->time_to_complete/ UPDATE_STEP_TIME;
}

void Trajectory::generate(Mapping* map){
    double ref_x = this->ego->x;
    double ref_y = this->ego->y;
    double ref_yaw = this->ego->yaw;
    this->points_x = this->ego->prev_x;
    this->points_y = this->ego->prev_y;
    this->ref_vel += this->target_acc*SPEED_INCREMENT;

    double x_add_on = 0;
    for (int i=0; i <= this->step_to_complete; i++){
        double N = this->target_distance/ (0.02*this->ref_vel*0.44704);
        double x_point = x_add_on + this->target_x/ N;
        double y_point = this->spline(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
        y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);

        x_point += ref_x;
        y_point += ref_y;

        this->points_x.push_back(x_point);
        this->points_y.push_back(y_point);
    }
}

float Trajectory::cost(double prev_vel, double final_vel){
    float cost;
    double weight = RULES_COST + EFFICIENCY_COST + SAFETY_COST;
    Vehicle next_ego = this->ego->next_ego(this);
    char mode;
    
    vector<vector<Vehicle>> surroundings = {this->ego->vehicles_ahead, this->ego->vehicles_behind, this->ego->vehicles_left, this->ego->vehicles_right};
    if (this->state->id != "RDY"){
        float rules_cost = costfunc_Rules(this, prev_vel, final_vel, this->time_to_complete);
        float efficiency_cost = costfunc_Efficiency(this, surroundings);
        float safety_cost = costfunc_Safety(this, &next_ego, surroundings, this->time_to_complete);
        cost = (RULES_COST*rules_cost + EFFICIENCY_COST*efficiency_cost + SAFETY_COST*safety_cost)/ weight;
        //cout << "*COST BRKDWN* --> Acc: " << this->target_acc << " | Rules: " << rules_cost << " | Efficiency: " << efficiency_cost << " | Safety: " << safety_cost << " | Sum: " << cost << endl;
    }
    else {
        cost = MAX_COST;
    }
    return cost;
}

Trajectory::~Trajectory(){}