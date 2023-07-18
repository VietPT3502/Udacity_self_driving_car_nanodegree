# Writeup: Track 3D-Objects Over Time

Please use this starter template to answer the following questions:

### 1. Write a short recap of the four tracking steps and what you implemented there (filter, track management, association, camera fusion). Which results did you achieve? Which part of the project was most difficult for you to complete, and why?

step1, implement the kalman filter:
* The F matrix (State translation)
* The Q matrix (Noise covariance) by calculate the integral of the FqF.transpose() from 0 to dt
*  update
* gamma, S predict 
step2, implement track management 
* check visibility to decrease unassigned track score if in fov and delete old tracks. If track state are confirmed, consider track score and track P. Else only consider track P so don't delete initialize track
* increase track score and check track score if bigger than confirmed threshold, update track state
step3, Implement association class
* calculate association matrix base on Mahalanobis distance
* add gating to reduce computation of Mahalanobis distance if higher than defined threshold using chi2.ppf
step4, Implement camera sensor and measurement
* in_fov function to decided object x can be seen by this sensor
* implement nonlinear measurement hx for camera by project position estimate from vehicle to camera coordinates and project to image coordinates
* Initialize camera measurement with z and R with sigma_i and sigma_j from misc/params 

I achieved RMSE of track 2 initial object in sequence 1 is 0.17 and 0.11
I think the most difficult part is step 3 because im new to Mahalanobis distance and Gating. I have learn a lot from this 

### 2. Do you see any benefits in camera-lidar fusion compared to lidar-only tracking (in theory and in your concrete results)? 
I think camera-lidar fusion can be helpful instead of only lidar because lidar has some drawbacks by using reflected light. Furthermore, there will be less ghost tracking for lidar or camera false positives.


### 3. Which challenges will a sensor fusion system face in real-life scenarios? Did you see any of these challenges in the project?
 When some object is hard to get reflected light like black object, camera measurements will help to adjust the result. Or when detection model is not very good. Or when the object can be associate with other pairs.

### 4. Can you think of ways to improve your tracking results in the future?
I think to improve tracking results we can use different track score based on confidence of the detection model and num frame appear. And we can implement more advance data association instead of current SNN.

