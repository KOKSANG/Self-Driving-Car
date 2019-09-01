#ifndef TRAJECTORY_H
#define TRAJECTORY_H
#include <iostream>
#include <vector>
#include "constants.h"
#include "spline.h"

class State;
class Vehicle;

class Trajectory {
    public:
        State* state;
        Vehicle* ego;
        static tk::spline spline;
        double ref_vel;
        double target_x;
        double target_y;
        double target_distance;
        double target_acc;
        double time_to_complete;
        double step_to_complete;
        vector<double> points_x;
        vector<double> points_y;
        vector<double> points_s;
        vector<double> points_d;

        Trajectory();
        Trajectory(Vehicle* ego, State* state, tk::spline s, vector<double> target, double ref_vel, double time_to_complete);

        void generate(Mapping* map);
        float cost(double prev_vel, double final_vel);

        virtual ~Trajectory();

};

#endif
