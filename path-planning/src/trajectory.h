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
        double time_to_complete;
        vector<double> start;
        vector<double> target;
        vector<vector<double>> jmt;
        vector<vector<double>> kinematics_s;
        vector<vector<double>> kinematics_d;


        Trajectory();
        //Trajectory(vector<double> x, vector<double> y, vector<double> s, vector<double> s_d, vector<double> s_dd,
        //            vector<double> d, vector<double> d_d, vector<double> d_dd, vector<double> yaw, vector<double> target);
        Trajectory(State* state, Vehicle* ego, vector<double> start, vector<double> target, vector<double> x, vector<double> y, double time_to_complete);

        void generate();
        float cost(vector<vector<Vehicle>> surroundings);

        virtual ~Trajectory();

};

#endif
