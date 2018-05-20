# Model-Predictive-Control
Udacity Self-Driving NanoDegree Term 2 Project on Model Predictive Control

## Project: Model-Predictive-Control [![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Overview
---
This project implements Model Predictive Control (MPC) for Udacity's Self Driving Car Nanodegree. It uses an MPC optimizer to estimate the best trajectory of actuators (steering and throttle) to minimize a defined cost function which depends on Cross-track-error (CTE), steering angle error (epsi) and ensures a smooth change in control actuators over a predetermined time horizon.

Code changes
---
For this project I made the following changes to the starter code.

__*mpc.h*__

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

1. There are two main classes: MPC and FG_eval. MPC class implements the _Solve()_ method which returns the best actuator controls for the current time step, as well as the trajectory which minimizes the cost function. FG_eval class implements the _operator()_ method which sets up the _fg_ vector storing the cost function, states and constraints for the vehicle over _N_ time-steps assuming a simple kinematic model.
2. __operator()__ method: The first element of the _fg_ vector is the cost function. The cost function is defined by a weighted combination of the _CTE_, _epsi_, deviation of actual speed from the reference speed, penalties for using throttle and steering actuators, as well as penalties for changing the actuators too rapidly. The weight parameters for applying steering actuator or penalizing rapid changes in steering angle are tunable. After some experimentation, both weights are configured to 300.
3. __operator()__ method: The rest of the _fg_ vector holds states and constraints as described in the Udacity lectures. Since the steering actuator in the Unity simulator is opposite in sign to the examples in the lectures, I inverted the signs in the equations involving the steering actuator delta.
4. __Solve()__ method: The Solve method sets up the arguments for invoking the CppAD::ipopt library's _solve()_ routine. This involves setting the _vars[]_ array. The first few elements of this array are the state of the car. To model the control latency of 100ms, we initialize the states by using the current states, and predicting the state of the car one time step _dt_ into the future using the previous throttle and steering actuator controls. The remainder of the _vars[]_ array is initialized as described in the Udacity lectures.
5. __Solve()__ method: The solution returned by CppAD::ipopt library's _solve()_ routine holds the optimal trajectory which minimizes the cost function, as well as the associated actuator controls for achieving that trajectory. The controls for the current timestep is used to set the steering and throttle values inside the _main()_ routine. The optimal trajectory returned from _solve()_ are used to overlay the optimal planned trajectory over the stored map of waypoints for debug purposes.

__*Results*__

The results of the MPC project demonstrating the car moving in autonomous mode around the Unity simulator track are shown below.

[![PID Controller](https://github.com/calvinhobbes119/Model-Predictive-Control/blob/master/Untitled.png)](https://youtu.be/Xwy_awrvJbM)
