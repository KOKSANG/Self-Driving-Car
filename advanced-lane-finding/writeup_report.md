## Project 2: Advanced Lane Finding

### Overview
In this project, the final goal is to write a software pipeline to identify lane boundaries in a video from a front-facing camera on a car. Various previosuly learnt techniques will be used in this project in order to identify the lane boundaries.

---

**Advanced Lane Finding Project**

The steps of this project are the following:
1)  Import the required library.
2)  Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
3)  Apply a distortion correction to raw images.
4)  Use color transforms, gradients, etc., to create a thresholded binary image.
5)  Apply a perspective transform to rectify binary image ("birds-eye view").
6)  Detect lane pixels and fit to find the lane boundary.
7)  Determine the curvature of the lane and vehicle position with respect to center.
8)  Warp the detected lane boundaries back onto the original image.
9)  Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.
10) Feed video into the pipeline.

[//]: # (Image References)

[image1]: ./output_images/chess.png "Undistorted"
[image2]: ./output_images/calibrate.png "Road Transformed"
[image3]: ./output_images/combined.png "Binary Example"
[image4]: ./output_images/warped.png "Warp Example"
[image5]: ./output_images/windows.png "Sliding Windows Example"
[image6]: ./output_images/lane.png "Fit Lane"
[image7]: ./output_images/blane.png "Boundary Lane"
[image8]: ./output_images/finallane.png "Lane with boundary and metric"
[video1]: ./project_video_solution.mp4 "Video"

---
### Step 1: Import required library

I import various popular libraries that have some useful function for this project.
* OpenCV for calibrate and undistort function
* Numpy for array transformation
* Matplotlib for 2D points plotting
* MoviePy for video editing
The complete code for this step can be found in the first code cell of this "./Project2.ipynb" Jupyter notebook.

### Step 2: Camera Calibration

The code for this step is contained in the second code cell of the mentioned Jupyter notebook.

I start by preparing "object points", which will be the (x, y, z) coordinates of the chessboard corners in the world. Here I am assuming the chessboard is fixed on the (x, y) plane at z=0, such that the object points are the same for each calibration image.  Thus, `objp` is just a replicated array of coordinates, and `objpoints` will be appended with a copy of it every time I successfully detect all chessboard corners in a test image.  `imgpoints` will be appended with the (x, y) pixel position of each of the corners in the image plane with each successful chessboard detection.  

I then used the output `objpoints` and `imgpoints` to compute the camera calibration and distortion coefficients using the `cv2.calibrateCamera()` function.  I applied this distortion correction to the test image using the `cv2.undistort()` function and obtained this result: 

![alt text][image1]

### Step 3: Apply distortion correction to raw image

The code can be found in the third code cell.

To demonstrate this step, I applied the same distortion correction to the raw image using the `cv2.undistort()` function with the same mtx and dist paramter i get from the step above. The result is like this one:
![alt text][image2]


### Step 4: Use color transforms, gradients, etc., to create a thresholded binary image

In this steps, I used a combination of color and multiple gradient thresholds to generate a binary image. There is total four different gradient calculation function here. 

The first gradient measurements is directional gradient `abs_sobel_thresh()`.

The second gradient measurements is gradient magnitude `mag_thresh()`. 

The third gradient measurements is gradient direction `dir_thresh()`..

The fourth gradient measurements is colour gradient `color_gradient()`.

Finally, those gradient measurement will be combined `combined_thresh()` and created binary image below: 

![alt text][image3]

### Step 5: Apply a perspective transform to rectify binary image ("birds-eye view")

The code for my perspective transform includes a function called `warper()`.  The `warper()` function takes as inputs an image (`img`) and transform the perspective of the image from source to destination points. The source points is actually the trapezoid in the image and the destination points is the rectagle or trapezoid in the "birds-eye view". 

The source and destination points is hardcoded as following:

| Source        | Destination   | 
|:-------------:|:-------------:| 
| 280, 700      | 250, 720      | 
| 595, 460      | 250, 0        |
| 725, 460      | 1065, 0       |
| 1125, 700     | 1065, 720     |

I verified that my perspective transform was working as expected by drawing the `src` and `dst` points onto a test image and its warped counterpart to verify that the lines appear parallel in the warped image.

![alt text][image4]

### Step 6: Detect lane pixels and fit to find the lane boundary

In this steps, sliding windows techniques is used to find the lane pixels. First, we will take the histogram of the bottom half of the image. The peak of the left and right halves of the histogram will be the starting point for the left and right lines. Then sliding windows is used to find coordinates of left line and right line. Finally, the coordinates will be fit into a second order polynomial is calculated for both the left and right lane line. 

The code for function above is located in 15th cell where the function is called `detect_lines()` and it will produce the image below: 

![alt text][image5]

There is another function called `detect_similar_lines()` that identify the lane based on the line detected from `detect_lines()` since once the lines is found, it is most likely to remain there. The output using `detect_similar_lines()` is as below:

![alt text][image6]

### Step 7: Determine the curvature of the lane and vehicle position with respect to center

The code for this step can be found in 19th and 21th cell. The function calculate the curvature of the lane, where is one the important metric to control the steering and the offset of the vehicle to the center.

### Step 8: Warp the detected lane boundaries back onto the original image

So, the lines had been found. The things need to do is just apply the inverse matrix of the wrap perspective. Through this we can wrap the lane boundaries from bird-eyes view back to camera views. Here is an example of my result on a test image:

![alt text][image7]

### Step 9: Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position

Remember that we still have the curvature and offset values, we can directly write the information back to the screen as below:

![alt text][image8]

---

### Step 10: Pipeline (video)

In this step, a pipeline is created by using all previous function. The code is available in 27th and 28th cell block.

Here's a [link to my video result](./project_video_solution.mp4)

---

### Discussion

I am glad to finish this project although it is a very basic implementation and it is still room of improvements. 

One of the future improvement is smoothing. When everything is working, the line detections will jump around from frame to frame a bit and it can be preferable to smooth over the last n frames of video to obtain a cleaner result. So, each time a new high-confidence measurement is detected, it can be appended to the list of recent measurements and then take an average over n past measurements to obtain the lane position that want to draw onto the image.

In a nutshell, it is quite a challenging project and I learnt a lot of things throughout the project. Last but not least, many thanks to Udacity for this course. 
