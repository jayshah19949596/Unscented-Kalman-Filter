# Unscented Kalman Filter Project
----

This project utilizes a Unscented kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements.

# Overview of Kalman Filter
----

[image1]: ./data/kalman_filter_algo.PNG "Kalman Filter"
[image2]: ./data/ctrv_model.PNG "CTRV Model"

![Kalman Filter Overview][image1]

- Imagine you are in a car equipped with sensors on the outside. The car sensors can detect objects moving around: for example, the sensors might detect a pedestrian, or even a bicycle. 
- The Unscented Kalman filter demonstrated in this repo uses the CTRV motion model
- For variety, let's step through the Unscented Kalman Filter algorithm using the bicycle example.
- The Kalman Filter algorithm will go through the following steps:   
     **1] First Measurement** : the filter will receive initial measurements of the bicycle's position relative to the car. These measurements will come from a radar or lidar sensor.    
     **2] Initialize State and Covariance matrices** : the filter will initialize the bicycle's position based on the first measurement. Then the car will receive another sensor measurement after a time period Δt.   
     **3] Predict** : the algorithm will predict where the bicycle will be after time Δt.         
     **4] Update** : the filter compares the "predicted" location with what the sensor measurement says.
     
# State vector and model
----

For this project, a Constant Turn Rate and Velocity (CTRV) model is assumed while building the state vector. The state vector contains following components:

1. Position of the object in X axis (px)
2. Position of the object in Y axis (py)
3. Speed, or magnitude of velocity (v)
4. Yaw angle (psi)
5. Yaw rate, or rate of change of yaw angle (psidot)

where X and Y axis are relative to the direction in which the self driving car moves, shown below:  

![CTRV Model][image2]  


# Unscented Kalman Filter Implementation Algorithm:
----

Following goals were achieved as a part of implementation:

**1]** Build the state vector and the state transition matrix. Derive state transition equation. This represents the deterministic part of motion model. Stochastic part of motion is represented by νak (For magnitude of acceleration) and νpsik (For yaw acceleration or the change in yaw rate). 

**2]** LIDAR measures the distance between self driving car and an object in X and Y axis. Hence, the measurement function for LASER updates, given by H_laser, is a linear transform.

**3]** RADAR measures the radial distance, the bearing (or angle of orientation w.r.t car) and the radial velocity.

**4]** Now that the state transition and measurement functions are derived, Kalman filter is used to estimate the path of moving object. Upon receiving a measurement for timestamp k+1, following processes are triggered:    
    
    - Kalman filter Predict: To use the state vector at timestamp k (Xk) and **predict** the state vector at timestamp k (Xk+1). This is the updated belief after motion.   
    - Use the measurement and update the belief using Kalman filter **update** once measurement is received.   
  
**5]** Kalman filter predict step is same for LASER and RADAR measurements. This step involves use of Unscented Kalman Filter algorithm to predict the mean and covariance for the next step. Given the belief of state and covariance matrix at state k, Unscented Kalman Filter algorithm consists of following steps:  
    
    - Generate sigma points: In this step, 2n + 1 sigma points are generated, where n is the number of states in state vector. Here, one sigma point is the mean of state while rest all are chosen on the uncertainty ellipse of the Gaussian distribution.   
    - Predict sigma points: The sigma points generated in step a are then passed through the process model and to predict the sigma points for the state k+1.   
    - Predict the mean and covariance: Predicted sigma points from step b are then used to calculate the predicted mean and covariance for the state k+1.    

**6]** When a LASER measurement at step k+1 is received, use vanilla Kalman filter (since the measurement function is linear) equations to update the predicted belief in step 5.

**7]** When a RADAR measurement at step k+1 is received, again use Unscented Kalman filter (since the measurement function is non-linear) equations to update the predicted belief in step 5.

**8]** Take a note of Normalized Innovation Squared (NIS) at each step for both LASER and RADAR. Plot the NIS distribution and fine tune the process noise parameters (σa and σyaw) to attain consistency in the system. Equation for calculating NIS is given below:

**9]** Calculate the root mean squared error (RMSE) after Kalman filter update at each time step. This is given by:

   
   
# Precition Step
----

- Prediction step includes following steps:  
   1] Generate Sigma Points  
   2] Predict Sigma Points  
   3] Predict Mean and Co-variance  
   
# Updation Step
----

- Updation step includes following steps:    
  1] Predict Measurement  
  2] Update State  
  
# Unscented Kalman Filter RoadMap:
----

[image3]: ./data/Ukf_roadmap.PNG "Kalman Filter"

![Ukf_roadmap][image3]
  

   


# Program Architecture
----

- The src folder has the source code of the project  
- It has the following files:   
  1] [json.hpp](https://github.com/jayshah19949596/Unscented-Kalman-Filter/blob/master/src/json.hpp)      
  2] [main.cpp](https://github.com/jayshah19949596/Unscented-Kalman-Filter/blob/master/src/main.cpp)      
  3] [measurement_package.h](https://github.com/jayshah19949596/Unscented-Kalman-Filter/blob/master/src/measurement_package.h)         
  4] [tools.cpp](https://github.com/jayshah19949596/Unscented-Kalman-Filter/blob/master/src/tools.cpp)     
  5] [tools.h](https://github.com/jayshah19949596/Unscented-Kalman-Filter/blob/master/src/tools.h)      
  6] [ukf.cpp](https://github.com/jayshah19949596/Unscented-Kalman-Filter/blob/master/src/ukf.cpp)      
  7] [ukf.h](https://github.com/jayshah19949596/Unscented-Kalman-Filter/blob/master/src/ukf.h)      


# Source Code File Description
----

1] [main.cpp](https://github.com/jayshah19949596/Unscented-Kalman-Filter/blob/master/src/main.cpp):
   - This is where the code execution starts
   - This file Communicates with the Term 2 Simulator receiving data measurements
   - main.cpp reads in the sensor [data](https://github.com/jayshah19949596/Unscented-Kalman-Filter/blob/master/data/obj_pose-laser-radar-synthetic-input.txt) which is a txt file  
   - After reading the sensor data the code figures out whether the sensor data is LIDAR or RADAR and accordingly set the the state matrix and sends a sensor measurement to FusionEKF.cpp
   - Calls `ProcessMeasurement()` function of class `UKF` to process the data. The body `ProcessMeasurement()` is defined in [ukf.cpp](https://github.com/jayshah19949596/Unscented-Kalman-Filter/blob/master/src/ukf.cpp) file 
   - Calls `CalculateRMSE()` function of class `Tools` to calculate RMSE. The body `CalculateRMSE()` is defined in [tools.cpp](https://github.com/jayshah19949596/Unscented-Kalman-Filter/blob/master/src/tools.cpp) file 
   
2] [ukf.cpp](https://github.com/jayshah19949596/Unscented-Kalman-Filter/blob/master/src/ukf.cpp)	
   - Initializes the kalman filter
   - Calls `Predict()` function of `UKF` class. The body `Predict()` is defined in [ukf.cpp](https://github.com/jayshah19949596/Unscented-Kalman-Filter/blob/master/src/ukf.cpp) file
   - Calls `UpdateRadar()` function of `UKF` class if the sensor data is RADAR. The body `UpdateRadar()` is defined in [ukf.cpp](https://github.com/jayshah19949596/Unscented-Kalman-Filter/blob/master/src/ukf.cpp) file. 
   - Calls `UpdateLidar()` function of `UKF` class if the sensor data is LASER. The body `UpdateLidar()` is defined in [ukf.cpp](https://github.com/jayshah19949596/Unscented-Kalman-Filter/blob/master/src/ukf.cpp) file.

4] [tools.cpp](https://github.com/jayshah19949596/Unscented-Kalman-Filter/blob/master/src/ukf.cpp)
   - Defines the function `CalculateRMSE()` function of class `Tools` to calculate RMSE 


# Required Installation and Detials
----

- Download the Simulator [here](https://github.com/udacity/self-driving-car-sim/releases)
- [uWebSocketIO](https://github.com/uNetworking/uWebSockets) package is required so that main.cpp can communicate with the simulator
- [uWebSocketIO](https://github.com/uNetworking/uWebSockets) package facilitates the connection between the simulator and [main.cpp](https://github.com/jayshah19949596/Unscented-Kalman-Filter/blob/master/src/main.cpp)
- uWebSocketIO installtion on Linux: From the project repository directory run the script `install-ubuntu.sh` which will do the installtion for you
- uWebSocketIO installtion on Mac: From the project repository directory run the script `install-mac.sh` which will do the installtion for you
- uWebSocketIO installtion on Windows: Follow the instruction on the [link](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10) to install bash on the Windows and with the bash run the script `install-ubuntu.sh` which will do the installtion for you

# Important Dependencies
----

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
  
  
# Basic Build Instructions
----

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./UnscentedKF `


# Running the Simulator 
---

1] Open the Simulator after you run the `./UnscentedKF ` command   
2] Click on "SELECT" Option   
3] Click on "START" Option  
