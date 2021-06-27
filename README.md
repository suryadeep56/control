# Controls
This repository contains the codes for various path tracking controllers I have implemented during my work in Controls team of AGV, IIT Kharagpur :
1. Pure Pursuit
2. Stanley
3. Linear Quadratic Regulator
4. Model Predictive Control

The first three controllers are designed based on this [paper](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf).
The Pure Pursuit and Stanley Controllers are based on the Bicycle model of the vehicle, while the LQR and MPC are based on kinematic model. I have implemented these on Python and used ROS platform.
To run these, you will need to download [basic-car-sim](https://github.com/sridhar-singhal/basic_car_sim) package and install ROS (Melodic or higher) on Ubuntu 18.04 or higher.
