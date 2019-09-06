# **Traffic Sign Recognition** 

---

**Build a Traffic Sign Recognition Project**

The goals / steps of this project are the following:
* Load the data set (see below for links to the project data set)
* Explore, summarize and visualize the data set
* Design, train and test a model architecture
* Use the model to make predictions on new images
* Analyze the softmax probabilities of the new images
* Summarize the results with a written report


[//]: # (Image References)

[image1]: ./images/hist_train.png "Training Data Distribution"
[image2]: ./images/hist_valid.png "Validation Data Distribution"
[image3]: ./images/hist_test.png "Testing Data Distribution"
[image4]: ./images/all_images.png "All 43 classes of Traffic Sign Images"
[image5]: ./images/hist_aug.png "Augmented Training Data Distribution"
[image6]: ./images/image_ori.png "Original Image"
[image7]: ./images/image_gray.png "Grayscale Image"
[image8]: ./images/image_norm.png "Normalized Grayscale Image"
[image9]: ./images/image_aug.png "Image with Gaussian Noise"
[image10]: ./images/test_images/all_test_images.png "Test Images Found Online"

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/481/view) individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one. You can submit your writeup as markdown or pdf. You can use this template as a guide for writing the report. The submission includes the project code.

You're reading it! and here is a link to my [project code](https://github.com/KOKSANG/Self-Driving-Car/blob/master/traffic-sign-classifier/Traffic_Sign_Classifier.ipynb)

### Data Set Summary & Exploration

#### 1. Provide a basic summary of the data set. In the code, the analysis should be done using python, numpy and/or pandas methods rather than hardcoding results manually.

I used the pandas library to calculate summary statistics of the traffic
signs data set:

* The size of training set is 34799
* The size of the validation set is 4410
* The size of test set is 12630
* The shape of a traffic sign image is (32, 32, 3)
* The number of unique classes/labels in the data set is 43

#### 2. Include an exploratory visualization of the dataset.

Here is an exploratory visualization of the data set. It is a bar chart showing how the data ...

![alt text][image1]
![alt text][image2]
![alt text][image3]

### Design and Test a Model Architecture

#### 1. Describe how you preprocessed the image data. What techniques were chosen and why did you choose these techniques? Consider including images showing the output of each preprocessing technique. Pre-processing refers to techniques such as converting to grayscale, normalization, etc. (OPTIONAL: As described in the "Stand Out Suggestions" part of the rubric, if you generated additional data for training, describe why you decided to generate additional data, how you generated the data, and provide example images of the additional data. Then describe the characteristics of the augmented training set like number of images in the set, number of images for each class, etc.)

As a first step, I decided to convert the images to grayscale because a 3 channels image like RGB requires more computing resources to train my ConvNet and also RGB image might cause overfitting issue due to its bigger size of data.

Here is an example of a traffic sign image before and after grayscaling.

![alt text][image6]
![alt text][image7]

As a last step, I normalized the image data because it would help the ConvNet to train faster.

I decided to generate additional data because I found that certain classes have insufficient data size which could lead to biases in the outcomes. 

To add more data to the the data set, I used the following techniques because they give a differentiable yet effective new data. Techniques that I used are:
1. Left right or up down flip
2. Gaussian noise
3. Rotation

Here is an example of an augmented image with Gaussian Noise:

![alt text][image8]

The difference between the original data set and the augmented data set is that the augmented data could be flipped, rotated or noisy. 


#### 2. Describe what your final model architecture looks like including model type, layers, layer sizes, connectivity, etc.) Consider including a diagram and/or table describing the final model.

My final model consisted of the following layers:

| Layer         		|     Description	        						| 
|:---------------------:|:-------------------------------------------------:| 
| Input         		| 32x32x1 Grayscale image   						| 
| Convolution 5x5     	| 1x1 stride, valid padding, outputs 28x28x6 		|
| RELU					|													|
| Max pooling 2x2		| 2x2 stride,  valid padding, outputs 14x14x6 		|
| Convolution 5x5     	| 1x1 stride, valid padding, outputs 10x10x16 		|
| RELU					|													|
| Max pooling 2x2		| 2x2 stride,  valid padding, outputs 5x5x16 		|
| Batch Normalization	|													|
| Dropout 0.5			|													|
| Convolution 3x3     	| 1x1 stride, valid padding, outputs 3x3x100 		|
| RELU					|													|
| Batch Normalization	|													|
| Dropout 0.5			|													|
| Fully connected 900	| outputs 400										|
| RELU					|													|
| Fully connected 400	| outputs 200										|
| RELU					|													|
| Fully connected 200	| outputs 43										| 


#### 3. Describe how you trained your model. The discussion can include the type of optimizer, the batch size, number of epochs and any hyperparameters such as learning rate.

To train the model, I fine tuned several parameters and they are:
1. mu = 0, sigma = 0.1
2. Epochs = 60
3. Batch size = 128
4. Learning rate = 0.005
5. Optimizer = Adam


#### 4. Describe the approach taken for finding a solution and getting the validation set accuracy to be at least 0.93. Include in the discussion the results on the training, validation and test sets and where in the code these were calculated. Your approach may have been an iterative process, in which case, outline the steps you took to get to the final solution and why you chose those steps. Perhaps your solution involved an already well known implementation or architecture. In this case, discuss why you think the architecture is suitable for the current problem.

My final model results were:
* training set accuracy of 
* validation set accuracy of 93.0% 
* test set accuracy of 93.9%

The first architecture tried was the one adopted from LeNet architecture, it was chosen because of its highly versatile performance on handwritings so I planned to design my ConvNet based on LeNet.

LeNet was able to achieve around 90% of accuracy. Therefore, in order to achieve at least a 93%,  I started to tune with LeNet. 

Firstly, I lower down my learning rate to 0.00001 because I thought that my gradient would explode with 0.001 initially. However, this new learning rate seems to be learning too slow in the sense that epochs of 60 is not enough to reach 93%. So, I increased the rate to 0.0001.

This time,my ConvNet was able to learn faster significantly, but there was a problem of its accuracy hovers around 80 to 90% which was still insufficient to reach our goal. Therefore, I started to look for more options.

I decided to augment some of my image classes due to their huge difference of data size. I put 1000 images to be the minimum training data size for each class. Then, I started to augment my images randomly with techniques Gaussian Noise, rotation, or flipping.

After that, I read that batch normalization could help in speeding up the training as well. So I decided to put some of them into my ConvNet layers. I also thought that with the extra augmented data, there was a possibility that overfitting could happen. Hence, I put in some dropout layers as well. The dropout rate was tested to be the best at 0.5 to 0.6.

Next, I continued to train this ConvNet design. The outcomes were so much better that it surpassed 95% of validation and testing accuracy. However, it was not performing as expected when moving on to testing new images.

To overcome it, I thought that I might just insert another layer of convolution and linearly connected layer into my design. This is to improve the capability of my ConvNet design in recognizing more complex traffic sign images. The new conv layer is 3x3x100 in shape, and the linear layer was inserted in the sense that to spread out the neurons distribution in all layers.

Then, epochs of 60 seemed to be too little for my design, so I again increased my learning rate to 0.0005.

The results was mediocre in validation accuracy but performing well in both test accuracy and testing new traffic sign images.


### Test a Model on New Images

#### 1. Choose five German traffic signs found on the web and provide them in the report. For each image, discuss what quality or qualities might be difficult to classify.

Here are five German traffic signs that I found on the web:

![alt text][image10]


#### 2. Discuss the model's predictions on these new traffic signs and compare the results to predicting on the test set. At a minimum, discuss what the predictions were, the accuracy on these new predictions, and compare the accuracy to the accuracy on the test set (OPTIONAL: Discuss the results in more detail as described in the "Stand Out Suggestions" part of the rubric).

Here are the results of the prediction:

| Image						        |     Prediction	        			| 
|:---------------------------------:|:-------------------------------------:| 
| Right-of-way at next intersection | Right-of-way at next intersection   	| 
| 20 km/h     						| 20 km/h 								|
| Priority road						| Priority road							|
| Keep right	      				| Keep right			 				|
| Turn left ahead					| Turn left ahead      					|
| General caution					| General caution      					|
| Road work							| Road work      						|
| 60 km/h							| 60 km/h      							|


The model was able to correctly guess all 8 traffic signs, which gives an accuracy of 100%. This compares favorably to the accuracy on the test set of 93.9%

#### 3. Describe how certain the model is when predicting on each of the five new images by looking at the softmax probabilities for each prediction. Provide the top 5 softmax probabilities for each image along with the sign type of each probability. (OPTIONAL: as described in the "Stand Out Suggestions" part of the rubric, visualizations can also be provided such as bar charts)

The code for making predictions on my final model is located in the 20th cell of the Ipython notebook.

Across all the images, the model is seemed to be not assertively sure that it has found the correct sign.
For the forth image, the model is relatively sure that this is a stop sign (probability of 0.6), and the image does contain a stop sign. The top five soft max probabilities were

| Probability         	|     Prediction	        					| 
|:---------------------:|:---------------------------------------------:| 
| .344      			| Keep right					 				|
| .290     				| Turn left ahead      							|
| .269      			| Right-of-way at next intersection   			|
| .260     				| General caution      							|
| .207     				| Priority road									|
| .203     				| 20 km/h 										|
| .136     				| 60 km/h      									|
| .127     				| Road work      								|

The top image is keep right that it is 34.4% sure that it is a keep right sign, then followed by turn left ahead and so on. It can be concluded that the model is only able to recognize our dataset well but it is not generalized to recognize unseen or new traffic signs.

Here are few of the points that can be worked on:
- Augment all data classes instead of those are low to balance out the training data.
- Use bigger variety of augmentation techniques.
- More hyperparameters tuning.

### (Optional) Visualizing the Neural Network (See Step 4 of the Ipython notebook for more details)
#### 1. Discuss the visual output of your trained network's feature maps. What characteristics did the neural network use to make classifications?


