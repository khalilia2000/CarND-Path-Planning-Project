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
  // PCL: Perform Change Lane
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
  int num_points_in_trajectory = 5;
  // time interval between points
  double time_interval_between_points = 0.02;
  // safe distance to the car in front
  double safe_distance_from_other_cars = 30;
  // collision distance - limit beyond whitch cars will collide
  double collision_distance = 3;
  // passing distance - minimum distance from car behind before changing lanes
  double passing_distance = 10;
  // meter per second to mile per house conversion factor - multiply mps by this factor to get mph
  double conversion_factor_mps_to_mph = 2.23694;
  // smoothing ratio at the end of tranjecotires to make sure car is parallel ot the road
  double smoothing_ratio = 0.30;
  // number of tranjectories to build in each lane
  int num_trajectories_for_each_lane = 35;
  // maximum allowable acceleration in m/s2
  double max_acceleration = 10.0;
  // maximum allowable jerk in m/s3
  double max_jerk = 10.0; 
  // target lane
  int target_lane = 1;


  // return current lane based on d
  int get_lane_for_d(double d); 

  // return d value for the center of the lane - lane = 0 is the left most lane
  double get_d_for_lane(int lane);

  // update state
  void update_state(bool too_close_ahead, double end_d);

  // generate all possible states from the current state
  vector<int> possible_lanes_to_explore(int ref_lane);

  // generate all possible tranjectories using only few points that are spaced far apart
  vector<vector<vector<double>>> generate_trajectories(vector<int> abs_possible_lanes, double car_s, vector<double> end_xyyaw, vector<double> prev_xyyaw, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);

  // calculate various metrics for a given trajectory
  double estimate_cost_for_trajectory(vector<double> car_xyyawspeed, vector<double> car_sd, vector<vector<double>> xy_traj, vector<double> maps_x, vector<double> maps_y, vector<vector<double>> sensor_fusion); 

  // determine if trajectory collides with another car
  bool will_collide(vector<vector<double>> sensor_fusion, vector<vector<double>> xy_trajectory, double car_s, int num_points_to_check);

  // return sensor_fusion data for the car immediately ahead in the specified lane
  vector<double> sensor_fusion_data_for_car_ahead(vector<vector<double>> sensor_fusion, int ref_lane, double car_s);

  // return sensor_fusion data for the car immediately behind in the specified lane
  vector<double> sensor_fusion_data_for_car_behind(vector<vector<double>> sensor_fusion, int ref_lane, double car_s);

  // determine if the vehicle is too close to the car ahead
  bool is_too_close_ahead(vector<vector<double>> sensor_fusion, int ref_lane, double ref_s, double time_shift);

  // update target speed
  void update_target_speed(bool is_too_close_ahead, double speed_of_car_ahead);

  // distance to car ahead
  double distance_to_nearby_car(double ref_s, int ref_lane, vector<double> ref_xy, vector<vector<double>> sensor_fusion, bool ahead);

  // speed of the car ahead
  double speed_of_nearby_car(double ref_s, int ref_lane, vector<vector<double>> sensor_fusion, bool ahead);

  // determines if there is any car nearby
  bool is_there_any_car_nearby(double ref_s, int ref_lane, vector<vector<double>> sensor_fusion, bool ahead);

};

#endif /* PLANNERH_ */