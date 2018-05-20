# Model-Predictive-Control
Udacity Self-Driving NanoDegree Term 2 Project on Model Predictive Control

## Project: Model-Predictive-Control [![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Overview
---
This project implements Model Predictive Control (MPC) for Udacity's Self Driving Car Nanodegree. It uses an MPC optimizer to estimate the best trajectory of actuators (steering and throttle) to minimize a defined cost function which depends on Cross-track-error (CTE), steering angle error (epsi) and ensures a smooth change in control actuators over a predetermined time horizon.

Code changes
---
For this project I made the following changes to the starter code.

__*mpc.h*___

1. I defined useful macros for number of timesteps _N_, time-delta between acutations _dt_, reference speed (converted to m/s), and cost function multipliers for changing steering values as well as rate-of-change of steering value.
2. In selecting _N_ and _dt_ I considered the following factors: (a) Predicting states for _N * dt_ timesteps should span the entire set of waypoints which are available at each instant. This allows us to make better instantaneous control decisions because we are operating with knowledge of the longest future time-horizon. (b) _dt_ should be a divisor of the control latency (100ms) allowing us absorb the control latency into our model as a prediction over some finite number of _dt_ time intervals.
3. Picking a reasonable refrence speed of _V_ of 50 mph, the considerations in (2) yielded two possible options for (_N_ , _dt_) - (15 ,  0.1) and (30 , 0.05). Setting _N_ = 15 and _dt_ = 0.1 gave better results than the other option, so I settled on that.

__*main.cpp*__

1. I transformed the telemetry data by converting the speed _v_ into m/s from mph.
2. I converted the waypoints from map-coordinates to vehicle coordinates by applying a matrix transformation, and fit a 3rd order polynomial function to the waypoints. The coefficients of this polynomial _f_ will be provided to the MPC solver to predict the optimal control trajectory for minimizing a cost function.
3. To determine the state parameters _CTE_ and _psi_, we use _f(0)_ and _f'(0)_.
4. The state vector and best-fit polynomial coeffs are fed to the MPC solve routing.
5. The MPC solve routine returns the actuator settings for the next time step, as well as the predicted optimal trajectory which minimizes the cost function over _N_ timesteps.
6. The steering actuator value is normalized by _deg2rad(25)_ to convert the steering value to be between _[-1,1]_.
7. The MPC predicted trajectory is returned to the Unity simulator as a json message for visualization purposes.

__*MPC.cpp*__

1. There are two 

Tuning
---
I manually tuned the  PID parameters for steer value _Kp_, _Kd_ and _Ki_ assuming a fixed throttle position of 0.3. The procedure I followed for tuning _Kp_, _Kd_ and _Ki_.

1. Assume _Kd_ and _Ki_ are zero. Choose a value of _Kp_ which causes the car to remain on track for a reasonable length of time, say several timesteps. This results in a plot of CTE as shown below with a value of _Kp_ = -0.2. As the plot shows the car stays on the track for several timesteps, but the CTE oscillations increase without dampening until the car veers off track. This indicates that we need to tweak the _Kd_ parameter to reduce the magnitude of the oscillations.

![Vary_Kp_alone](https://github.com/calvinhobbes119/PID-Controller/blob/master/figures/Kp_-0.2_Kd_0.0_Ki_0.0.png)

3. Keeping _Kp_ at -0.2, and _Ki_ at 0.0, I modified the _Kd_ parameter until the car stayed on track for the entire course. This results in a plot of CTE as shown below with a value of _Kd_ = -3.0.

![Vary_Kd_alone](https://github.com/calvinhobbes119/PID-Controller/blob/master/figures/Kp_-0.2_Kd_-3.0_Ki_0.0.png)

After zooming into the above plot, I noticed that there were many oscillations in the steering value (and the resulting CTE) from one timestep to the next. To reduce these oscillations I smoothed the CTE by averaging it with the CTE from the previous timestep. This resulted in a smoother steer value and CTE from one timestep to the next.

![Using_raw_vs_smoothed_CTE](https://github.com/calvinhobbes119/PID-Controller/blob/master/figures/Using_raw_vs_smoothed_CTE.png)

4. Finally, by calculating the average and median CTE over the course of one lap, it was clear that the CTE had a non-zero bias. By setting the _Ki_ parameter to -0.001, I was able to bring both the average and median CTE to close to zero.

5. I did some fine tuning of the _Kp_, _Kd_ and _Ki_ to account for the throttle PID controller in arriving at the final values used in the code. The video below shows the performance of the PID controllers around the simulated racetrack.


[![PID Controller](https://github.com/calvinhobbes119/PID-Controller/blob/master/figures/Untitled.png)](https://youtu.be/PbgqzFjZbFI)
