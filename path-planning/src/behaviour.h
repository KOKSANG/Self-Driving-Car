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
        State* state;
        Mapping* map;
        double ref_vel;
        int current_timestep;
        vector<State> next_states;
        vector<State> states;

        Behaviour();
        Behaviour(Vehicle* ego);
        Behaviour(Vehicle* ego, double ref_vel);

        virtual ~Behaviour();

        vector<State> available_states();
        vector<vector<double>> forecast_points(State* state, vector<double> points_x, vector<double> points_y);
        vector<Trajectory> generate_trajectory(vector<double> points_x, vector<double> points_y);
        Trajectory get_best_trajectory(vector<Trajectory> trajectories, vector<Vehicle> const surrounding_vehicles);
};

#endif
