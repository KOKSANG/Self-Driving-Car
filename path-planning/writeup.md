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

[p0]: ./P0.gif "Car Behaviour when P=0"
[d0]: ./D0.gif "Car Behaviour when D=0"
[i0]: ./I0.gif "Car Behaviour when I=0"

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

The general flow of the planner 



* P tends to make the car swings. Higher the P, harder the swinging is. With P = 0, the car is not able to react to high gradient change of CTE.

![Car Behaviour when P=0][p0]


* D tends to make the car drives calmly. Higher the D, the car converges to a point faster. With D = 0, the car is not able to stay on a lateral point for too long.

![Car Behaviour when D=0][d0]


* I tends to make the car converges to central of road. Higher I, makes the car easily to get to central. With I = 0, the car is not able to converge to central.

![Car Behaviour when I=0][i0]


#### 4. Describe how hyperparameters were chosen

* For inital hyperparameters, they are all optimized manually until the car is able to drive nicely at the start and also based on the twiddle values.
* As for final hyperparameters, they are done using twiddle which computes a descent of previous values based on the total CTE and current CTE.
* The twiddle function can be found in PID.cpp (line 69)


#### 5. The car must drive at least a lap succcessfully

The car has been tested and validated to drive around the track for multiple laps.
