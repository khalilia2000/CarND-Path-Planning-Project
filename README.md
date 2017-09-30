# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
This repository is a project for term 3 of self-driving car nanodegree from Udacity.
   
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

Both the speed and the current state is updated in `main.cpp` (lines 138 to 144). The state change occurs in `planner.cpp` in method update_state (lines 27 to 43). The speed change occurs in `planner.cpp` in update_target_speed method (lines 341 to 351).

All trajectories are generated in main.cpp (line 178 to 185) and then are combined with the previous_path that was returned from simulator for easy transitioning. Once combined, a cost is assigned for each trajectory (`main.cpp` lines 188 to 194, and `planner.cpp` lines 174 to 222) and then the minimum cost is calculated (`main.cpp` lines 196 to 207). The minimum cost trajectory is selected and passed on to the simulator. The method for calculating the cost considers and the penalizes possible collisions, and rewards the clear lanes (`planner.cpp` lines 174 to 222). 

Only the adjacent lanes are explored when building trajectories and if the current state is ECL.

The speed and the acceleration of the trajectories do not affect their cost, because the update_target_speed method (`planner.cpp` lines 341 to 351) inherently does not allow the target_speed to go beyond the maximum allowed speed. Furthermore, the method generate_trajectories (`planner.cpp` lines 78 to 171) inherently only generates trajectories that are smooth (using spline) and are not having high jerks or accelerations.

