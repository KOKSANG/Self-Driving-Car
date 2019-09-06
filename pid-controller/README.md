# **PID Controller** 

---

**PID Controller Project**

The goals / steps of this project are the following:
* Build a PID controller to calculate car's steering in simulator
* Finetune the tau and descent value of the controller
* Experiment effects of P, I and D on steering values.
* Test it out so that the car drives around track successfully
* Summarize the results with a written report


[//]: # (Image References)

[p0]: ./P0.gif "Car Behaviour when P=0"
[d0]: ./D0.gif "Car Behaviour when D=0"
[i0]: ./I0.gif "Car Behaviour when I=0"

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/1972/view) individually and describe how I addressed each point in my implementation.  

---

#### 1. Code should compile

My project has a run.sh script. Just run it to compile and run the code. It should run with problem.


#### 2. PID procedure follows what was taught in lessons

P, I and D controller are implemented according to what was taught in lessons. Hyperparameter is also optimized with twiddle.


#### 3. Describe effects of P, I and D

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
