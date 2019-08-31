#include <iostream>
#include <math.h>
#include <string>
#include <vector>
#include "trajectory.h"
#include "vehicle.h"
#include "behaviour.h"
#include "jmt.h"
#include "helpers.h"
#include "constants.h"
#include "spline.h"

// for convenience
using std::string;
using std::vector;
using std::cout;
using std::sort;

Trajectory::Trajectory(){}

Trajectory::Trajectory(Vehicle* ego, tk::spline s, vector<double> target_xy, double target_distance, double ref_vel, double time_to_complete){
    this->ego = ego;
    this->spline = s;
    this->target_x = target_xy[0];
    this->target_y = target_xy[1];
    this->target_distance = target_distance;
    this->ref_vel = ref_vel;
    this->time_to_complete = time_to_complete;
    this->step_to_complete = this->time_to_complete/ UPDATE_STEP_TIME;
}

void Trajectory::generate(vector<double> prev_x, vector<double> prev_y){
    double ref_x = this->ego->x;
    double ref_y = this->ego->y;
    double ref_yaw = this->ego->yaw;
    this->points_x = prev_x;
    this->points_y = prev_y;

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

/*
float Trajectory::cost(vector<vector<Vehicle>> surroundings){
    float cost;
    if (this->state->id != "RDY"){
        float legality_cost = costfunc_Legality(this, surroundings, this->time_to_complete, LEGALITY_COST);
        float efficiency_cost = costfunc_Efficiency(this, surroundings, this->time_to_complete, EFFICIENCY_COST);
        float safety_cost = costfunc_Safety(this, surroundings, this->time_to_complete, SAFETY_COST);
        float comfort_cost = costfunc_Comfort(this, 1.0);
        cost = (legality_cost + efficiency_cost + safety_cost)/ 3;
        //cout << "[COST BRKDWN] - " << "Legality: " << legality_cost << ", Efficiency: " << efficiency_cost << ", Safety: " << safety_cost << ", Sum: " << cost << endl;
    }
    else {
        cost = PUNISHMENT_MAX_COST;
    }
    return cost;
}
*/

Trajectory::~Trajectory(){}