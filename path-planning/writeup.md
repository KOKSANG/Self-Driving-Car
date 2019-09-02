# **Path Planning** 
---

**Path Planning Project**

The goals / steps of this project are the followings:
* Build a path planner to enable the car drive around the track
* Finetune the parameters that enables the car to drive
* The car has to drive according to a few rules
* Make sure the car can at least drive one lap
* Document and summarize the results

[//]: # (Image References)

[flowchart]: ./flowchart.png "General flow of planner"
[FSM]: ./FSM.png "FSM"

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/1971/view) individually and describe how I addressed each point in my implementation.  

---

### 1. Code should compile

My project has a run.sh script. Just run it to compile and run the code. It should run with no problem.


### 2. Valid trajectories

#### Here are a few criteria to be evaluated:
* The car is able to drive at least **4.32 miles** without incident
* The car drives according to the speed limit, `(50 miles/s)`.
* Max **Acceleration** `(10 m/s^2)` and **Jerk** `(10 m/s^3)` are not Exceeded.
* Car does not have collisions.
* The car stays in its lane, except for the time between changing lanes and lane changing does not take >3 seconds.
* The car is able to change lanes

All criteria have been successfully met. The demo video can be found at [DEMO](https://youtu.be/eMWW5sSe0bg)


### 4. Reflection

The general flow of the planner is as shown below,

![General flow of planner][flowchart]

The FSM that I use:

![FSM][FSM]

#### Here are some clarifications to be made:
1. The parameters:
- Max speed is 50miles/s^1, speed increment per planning occasion made to car velocity is 0.224miles/s^1.
- Max acceleration and jerk allowed are both set to be 10.
- Max lane change and keep lane time are 1s, to always plan out for the next maximum 1s consistently.
- Planner is expected to update at consistent rate of 50Hz.
- Buffer range to the car ahead is 20m, it is also made use for lane changing check buffer range.
- Three main category of costs, rules, efficiency and safety are set to have weight of 1 only.

2. How to generate trajectory:
- Spline library is used.
- Waypoints of 1.5, 3 and 4.5 times of buffer range are used to generate the trajectory.
- The last two previous waypoints are also used.
- Three variations of accelerations, increase, zero and decrease, are made to each identical trajecory.

3. Initial conditions:
- Always have "RDY" which is READY as initial state.
- Starts at the middle lane
- Starts at velocity of 3miles/s^1 to prevent dead stop

4. Defining cost:
- The max cost is 1 and min cost is 0
- Cost either has binary or continuous value
- Some trajectory costs are set to be at 0.9 for maximum, this is to promote certain trajectory

---

To approach this path planner, firstly, the **ego** details are collected. The ego **previous waypoints x and y** are both retained for next trajectory x and y since we know that not all previous waypoints are **consumed**. If the car initialize, the current x, y and previous x, y calculated using car current yaw are used.

Next, data of **sensor fusion** are collected. A new ego is created and all these surrounding vehicles captured by sensor fusion are **sorted** according to their position to the ego, ahead, behind, left or right. Then, a new planner for the ego is created with the current reference velocity (starts at 3 initially).

The planner will get all the next available states, and compare them. Variations of positive, zero and negative increment to the speed are made for each state. So now, every state will get you three new trajectories. Planner then generates and calculate all their costs, and it returns the best one.

When the planner calls for trajectory generation function, the trajectory generates all its waypoints using spline. And when cost calculation function is called, the trajectory calculates the total cost based on the cost returned by all cost functions.


### 5. Documentation

#### Here are the documetations for the scripts

##### mapping.cpp
1. Class object: **`Mapping`**
2. Functions:
- `getXY`, to convert frenet to xy
- `getFrenet`, to convert xy to frenet
- `interpolate_points`, to interpolate between original points using spline

##### behavior.cpp
1. Class object: **`Behaviour`**
2. Functions:
- `available_states`, to get list of next available state
- `forecast_points`, to forecast the waypoints needed to generate a trajectory (using spline)
- `get_best_trajectory`, to generate all possible trajectory and return the best one
3. Class object: **`State`**
4. Has no function

##### vehicle.cpp
1. Class object: **`Vehicle`**
2. Functions:
- `sorting`, a lambda function to sort vehicles based on their distance to ego in s
- `sort_vehicles`, to sort surrounding vehicles captured by sensor fusion
- `predict_position`, to predict next position of surrrounding vehicles
- `next_ego`, to predict next position of ego

##### trajectory.cpp
1. Class object: **`Trajectory`**
2. Functions:
- `generate`, to generate trajectory based on the object parameters
- `cost`, to calculate trajectory cost

##### cost_functions.h
1. Has no class object
2. Functions:
- `lane_speed`, to get speed of desired lane
- `subcost_Speed`:, cost function for speed limit violation
- `subcost_Acceleration`, cost function for acceleration limit violation
- `costfunc_Rules`, subcost_Speed + subcost_Acceleration
- `subcost_LaneChange`, cost function for switching lane
- `subcost_SpeedChange`, cost function for the next highest possible speed
- `costfunc_Efficiency`, subcost_LaneChange + subcost_SpeedChange
- `subcost_Buffer`, cost function for buffering with car ahead
- `subcost_LatitudinalCollision`, cost function for lane switching car checking
- `costfunc_Safety`, subcost_Buffer + subcost_LatitudinalCollision

