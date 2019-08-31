#ifndef TRAJECTORY_H
#define TRAJECTORY_H
#include <iostream>
#include <vector>
#include "constants.h"

class State;
class Vehicle;

class Trajectory {
    public:
        vector<double> x;
        vector<double> y;
        /*
        vector<double> s;
        vector<double> s_d;
        vector<double> s_dd;
        vector<double> d;
        vector<double> d_d;
        vector<double> d_dd;
        vector<double> yaw;
        vector<double> target;
         */
        State* state;
        Vehicle* ego;
        tk::spline spline;
        double ref_vel;
        double target_x;
        double target_y;
        double target_distance;
        double time_to_complete;
        double step_to_complete;
        vector<double> points_x;
        vector<double> points_y;

        Trajectory();
        Trajectory(Vehicle* ego, tk::spline s, vector<double> target_xy, double target_distance, double ref_vel, double time_to_complete);

        void generate(vector<double> prev_x, vector<double> prev_y);
        float cost(vector<vector<Vehicle>> surroundings);

        virtual ~Trajectory();

};

#endif
