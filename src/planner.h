#ifndef PLANNER_H_
#define PLANNER_H_

#include <math.h>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "spline.h"
#include "helper.h"

using namespace std;

class Planner {
public:

  /**
  * Constructor.
  */
  Planner();

  /**
  * Destructor.
  */
  virtual ~Planner();

  // current state of the finite state machine
  // KL: Keep Lane
  // ECL: Explore Change Lane
  string state = "KL"; 
  // number of lanes
  int number_of_lanes = 3;
  // width of each lane in m
  double lane_width = 4;
  // maximum allowed speeds on the road in miles per hour
  double max_speed = 50;
  // target speed
  double target_speed = 0;
  // number of points to send to simulator
  int num_points_in_trajectory = 50;
  // time interval between points
  double time_interval_between_points = 0.02;
  // safe distance to the car in front
  double safe_distance_from_other_cars = 30;
  // collision distance - limit beyond whitch cars will collide
  double collision_distance = 5;
  // meter per second to mile per house conversion factor - multiply mps by this factor to get mph
  double conversion_factor_mps_to_mph = 2.23694;
  // smoothing ratio at the end of tranjecotires to make sure car is parallel ot the road
  double smoothing_ratio = 0.30;
  // number of tranjectories to build in each lane
  int num_trajectories_for_each_lane = 50;
  // maximum allowable acceleration in m/s2
  double max_acceleration = 10.0;
  // maximum allowable jerk in m/s3
  double max_jerk = 50.0; 

  // return current lane based on d
  int get_current_lane_for_d(double d); 

  // return d value for the center of the lane - lane = 0 is the left most lane
  double get_d_for_current_lane(int lane);

  // generate all possible states from the current state
  vector<int> generate_possible_lanes_to_explore(int current_lane);

  // generate all possible tranjectories using only few points that are spaced far apart
  vector<vector<vector<double>>> generate_trajectory_coarse(int current_lane, double given_s, 
    vector<double> given_xy, vector<double> prev_xy, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);

  // generates fine trajectory from coarse trajectory of a few points
  vector<vector<double>> generate_fine_from_coarse_trajectory(vector<double> ptsx, vector<double> ptsy, vector<double> given_xyyaw);

  // calculate various metrics for a given trajectory
  double estimate_cost_for_trajectory(vector<double> car_xyyawspeed, vector<vector<double>> xy_traj, vector<double> maps_x, vector<double> maps_y, vector<vector<double>> sensor_fusion); 

  // determine if trajectory collides with another car
  bool will_collide(vector<vector<double>> sensor_fusion, vector<vector<double>> xy_trajectory, double delta_t, int car_lane, double car_s);

  // return sensor_fusion data for the car immediately ahead in the specified lane
  vector<double> sensor_fusion_data_for_car_ahead(vector<vector<double>> sensor_fusion, int ref_lane, double car_s);

  // return sensor_fusion data for the car immediately behind in the specified lane
  vector<double> sensor_fusion_data_for_car_behind(vector<vector<double>> sensor_fusion, int ref_lane, double car_s);

};

#endif /* PLANNERH_ */