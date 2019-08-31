#ifndef BEHAVIOUR_H
#define BEHAVIOUR_H

#include <iostream>
#include <map>
#include <math.h>
#include <string>
#include <vector>
#include "helpers.h"
#include "constants.h"
#include "spline.h"

// for convenience
using std::string;
using std::vector;
using std::cout;
using std::sort;
using std::map;

class Trajectory;
class Vehicle;
class Mapping;

class State {
    public:
        string id;
        int current_lane;
        int intended_lane;
        int final_lane;
        int time_ahead;

        State();
        State(string id, int lane);

        virtual ~State();
};

class Behaviour {
    public:
        Vehicle* ego;
        double ref_vel;
        int current_timestep;
        vector<State> states;

        Behaviour();
        Behaviour(Vehicle* ego, double ref_vel);

        virtual ~Behaviour();

        vector<State> available_states();
        vector<Trajectory> generate_trajectory(vector<double> x, vector<double> y);
        Trajectory get_best_trajectory(vector<Trajectory> trajectories, vector<Vehicle> const surrounding_vehicles);
};

#endif
