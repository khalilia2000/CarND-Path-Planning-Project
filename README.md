# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
This repository is a project for term 3 of self-driving car nanodgree from Udacity.
   
## Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

## Reflection on Path Generation

A Finite State Machine (FSM) is defined with the following states:  
1. KL: Keep Lane - in this state, the car drives only in the current lane;
2. ECL: Explore Lane Change - in this state the car explores the possibility of changing lanes;
3. PCL: Perform Lane Change - in this state the car follow the previous trajectory for changing lanes until the change lane finishes;

both the speed and the current state is updated in main.cpp (lines X to X). The state change occures in planner.cpp (lines X to X).  

All pahts are generated in main.cpp (line X to X) and then are combined with the previous_path that was returned from simulator for easy transitioning. Once combined, a cost is assigned for each trajectory (main.cpp lines X to X, and planner.cpp lines X to X) and then the minimum cost is cacluated (main.cpp lines X to X). The minimum cost trajectory is selected and passed on to the simulator. The method for calculating the cost considers the penalizes possible collisions, and rewards the clear lanes (planner.cpp lines X to X). 

Only the adjacent lanes are explored when building trajectories and if the current state is ECL.

