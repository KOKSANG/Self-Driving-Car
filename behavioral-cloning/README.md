# **Behavioral Cloning** 

## Writeup Template

### You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---

**Behavioral Cloning Project**

The goals / steps of this project are the following:
* Use the simulator to collect data of good driving behavior
* Build, a convolution neural network in Keras that predicts steering angles from images
* Train and validate the model with a training and validation set
* Test that the model successfully drives around track one without leaving the road
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
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/432/view) individually and describe how I addressed each point in my implementation.  

---
### Files Submitted & Code Quality

#### 1. Submission includes all required files and can be used to run the simulator in autonomous mode

My project includes the following files:
* model.ipynb containing the script run the training was then converted to .py
* model.py containing the script to create and train the model
* drive.py for driving the car in autonomous mode
* model.h5 containing a trained convolution neural network 
* writeup_report.md or summarizing the results

#### 2. Submission includes functional code
Using the Udacity provided simulator and my drive.py file, the car can be driven autonomously around the track by executing 
```sh
python drive.py model/model.h5 run1
```

#### 3. Submission code is usable and readable

The model.py file contains the code for training and saving the convolution neural network. The file shows the pipeline I used for training and validating the model, and it contains comments to explain how the code works.

### Model Architecture and Training Strategy

#### 1. An appropriate model architecture has been employed

My model consists of multiple stacks of cropping layer, convolution neural network and dense/ feedforward layers (model.py lines 188-212). The overall architecture is shown below:

![alt text][image5]


The model includes RELU activation function to introduce nonlinearity (you can find it in almost every layers), and the data is normalized before feeding into the model using a self created numpy function (code line 75). 

#### 2. Attempts to reduce overfitting in the model

The model contains dropout layers in order to reduce overfitting (model.py lines 203-207).

The model was trained and validated on different data sets to ensure that the model was not overfitting by using train_test_split in sklearn (code line 141). The model was tested by running it through the simulator and ensuring that the vehicle could stay on the track.

#### 3. Model parameter tuning

The model used an adam optimizer, and the learning rate was set to 0.0001 (model.py line 210).

#### 4. Appropriate training data

Training data was chosen to keep the vehicle driving on the road. I used a combination of center lane driving, recovering from the left and right sides of the road, also left and right camera images of my recordings. 

The left and right camera images were given bias of random float number between 0.2 to 0.5. Left camera images was added with positive bias whereas right camera images were added with negative bias. This is to increase the car of steering to right when it is driving off to the left and vice versa.

For details about how I created the training data, refer to the next section. 

### Model Architecture and Training Strategy

#### 1. Solution Design Approach

The overall strategy for deriving a model architecture was to create a strong regression that can give us accurate steering values.

My first step was to start with data cleaning. As per discussed above, I added bias to left and right camera images. 

Without bias (Original)

![alt text][image2]

![alt text][image1]

With bias (Adjusted)

![alt text][image3]

![alt text][image4]

Other than that, I also attempted to add a Yeo-Johnson Transformation as bias to any of my left and right camera images with steering exceeding 0.8. You can read about Yeo-Johnson here [link1]

After that, I moved on to designing my model. I was inpirted to use a convolution neural network model similar to the Nvidia self driving car model introduced in the course. I thought this model might be appropriate because of its comprehensive design with strong proof of works backed by Nvidia.

In order to gauge how well the model was working, I split my image and steering angle data into a training and validation set of ratio 80:20. I shuffle my data before feeding into the model. 

I found that my first model had long training time per epoch. This implied that I might be passing too much data between layers. To overcome this, I revise my model and realize that I should also insert activation function layers in between my dense layers. So I did that.

Next, when I finished training my revised first model, I proceeded with feeding it into the simulator. I found that it had a high mean squared error on the training set. So I decided to make some tweakings to improve it. The results was also way off at few spots. I did realize that these spots are usually near the edges of the track. So, I went on to collect more data driving at the edges and also removed the Yeo-Johnson bias since it might be implemeted wrongly that it contributes to oversteering at the edges.

Besides, I also modified the model so that it has a lower learning rate, and a few more epochs just to make my validation and training loss both converging to minimal.

Then, I also added dropout layers in between dense layers to combat overfitting of the model and I run the simulator with my new model again. Below shows the training process.

![alt text][image6]

At the end of the process, the car was able to drive around autonomously without going off-track. So I decided to take it as my final results.

#### 2. Final Model Architecture

The final model architecture (model.py lines 188-212) consisted of a convolution neural network with the following layers and layer sizes.

![alt text][image5]

Here is a visualization of the architecture.

![alt text][image8]


#### 3. Creation of the Training Set & Training Process

I used mainly the image data found in project resources. In addition to that, I recorded one lap focusing on track center, and two with each of them focusing on edges. These images show a series of center, left and right camera images on edge of the track.

![alt text][image9]

![alt text][image10]

![alt text][image11]

After the collection process, I had 34108 number of data points. I then preprocessed this data by normalizing them to zero.


I finally randomly shuffled the data set and put 20% of the data into a validation set. 

I used this training data for training the model. The validation set helped determine if the model was over or under fitting. The ideal number of epochs was 10 to 15 as evidenced by the training and validation loss visualization as show below:

![alt text][image7]

I used an adam optimizer with learning rate of 0.0001.
