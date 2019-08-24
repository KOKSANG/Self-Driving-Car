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

[image1]: ./images/dist_ori.png "Original Steering Distribution"
[image2]: ./images/tail_ori.png "Original Data"
[image3]: ./images/tail_adjusted.png "Adjusted Data"
[image4]: ./images/dist_adjusted.png "Adjusted Steering Distribution"
[image5]: ./images/model_summary.png "Model Summary"
[image6]: ./images/training.png "Training Process"
[image7]: ./images/loss_visualization.png "Training and Validation Loss Visualization"
[image8]: ./images/nvidia.png "Model Architecture Visualization"
[image9]: ./images/center.jpg "Edge (Center Camera Image)"
[image10]: ./images/left.jpg "Edge (Left Camera Image)"
[image11]: ./images/right.jpg "Edge (Right Camera Image)"
[link1]: https://www.stat.umn.edu/arc/yjpower.pdf "Yeo-Johnson Transformation"

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/1972/view) individually and describe how I addressed each point in my implementation.  

---

#### 1. Code should compile

My project has a run.sh script. Just run it to compile and run the code. It should run with problem.


#### 2. PID procedure follows what was taught in lessons

P, I and D controller are implemented according to what was taught in lessons. Hyperparameter is also optimized with twiddle.


#### 3. Describe effects of P, I and D

* P tends to make the car swings. Higher the P, harder the swinging is. With P = 0, the car is not able to react to high gradient change of CTE.

![alt text]


* D tends to make the car drives calmly. Higher the D, the car converges to a point faster. With D = 0, the car is not able to stay on a lateral point for too long.

![alt text]


* I tends to make the car converges to central of road. Higher I, makes the car easily to get to central. With I = 0, the car is not able to converge to central.

![alt text]


#### 4. Describe how hyperparameters were chosen

* For inital hyperparameters, they are all optimized manually until the car is able to drive nicely at the start and also based on the twiddle values.
* As for final hyperparameters, they are done using twiddle which computes a descent of previous values based on the total CTE and current CTE.
* The twiddle function can be found in PID.cpp (line 69)

![alt text][image5]


#### 5. The car must drive at least a lap succcessfully

The car has been tested and validated to drive around the track for multiple laps.
