#include "planner.h"

Planner::Planner() {}

Planner::~Planner() {}

// return current lane based on d
int Planner::get_current_lane_for_d(double d) 
{
  if (d==number_of_lanes*lane_width) 
  {
    return number_of_lanes-1;
  } 
  else 
  {
    return (int)floor(d/lane_width);
  }
}

// return d value for the center of the lane - lane = 0 is the left most lane
double Planner::get_d_for_current_lane(int lane) 
{
	return (lane_width/2+lane_width*lane);
}


// generate all possible states from the current state
vector<int> Planner::generate_possible_lanes_to_explore(int current_lane)
{
  vector<int> possible_lanes;
  possible_lanes.push_back(0);
  if (state == "ECL")
  {
    if (current_lane < number_of_lanes-1) 
    {
      possible_lanes.push_back(1);
    } 
    else if (current_lane > 0) 
    {
      possible_lanes.push_back(-1);
    } 
  }
  //return possible_lanes;
  return possible_lanes;
}


// generate all possible tranjectories using only few points that are spaced far apart
// will return points in xy coordinates
vector<vector<vector<double>>> Planner::generate_trajectory_coarse(int current_lane, double given_s, 
  vector<double> given_xy, vector<double> prev_xy, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y) 
{

  vector<vector<vector<double>>> result_trajectories;  
  
  double delta_s = max_speed / conversion_factor_mps_to_mph;
  delta_s *= num_points_in_trajectory * time_interval_between_points;

  // define possible lanes for making trajectories
  vector<int> possible_lanes = generate_possible_lanes_to_explore(current_lane);

  // generate trajectories
  for (int j=0; j<possible_lanes.size(); j++)
  {
  	int ref_lane = current_lane + possible_lanes[j];
    for (int i=1; i<=num_trajectories_for_each_lane; i++)
    {
      // define vectors
      vector<vector<double>> trajectory; 
      vector<double> pts_x; 
      vector<double> pts_y;
      
      // define 2 end points
      double current_distance = delta_s / num_trajectories_for_each_lane * i;
  
      double target1_s = given_s + (1 - smoothing_ratio) * current_distance;
      double target2_s = target1_s + (smoothing_ratio * 0.5) * current_distance;
      double target3_s = target2_s + (smoothing_ratio * 0.5) * current_distance;
      double target1_d = get_d_for_current_lane(ref_lane);
      double target2_d = target1_d;
      double target3_d = target2_d;

      vector<double> target1_xy = Helper::getXY(target1_s, target1_d, maps_s, maps_x, maps_y);
      vector<double> target2_xy = Helper::getXY(target2_s, target2_d, maps_s, maps_x, maps_y);
      vector<double> target3_xy = Helper::getXY(target3_s, target3_d, maps_s, maps_x, maps_y);
      
      // create waypoints for s
      pts_x.push_back(prev_xy[0]);
      pts_x.push_back(given_xy[0]);
      pts_x.push_back(target1_xy[0]);
      pts_x.push_back(target2_xy[0]);
      pts_x.push_back(target3_xy[0]);
      
      // create wayponts for d
      pts_y.push_back(prev_xy[1]);
      pts_y.push_back(given_xy[1]);
      pts_y.push_back(target1_xy[1]);
      pts_y.push_back(target2_xy[1]);
      pts_y.push_back(target3_xy[1]);
      
      // add waypoints to the trajectory
      trajectory.push_back(pts_x);
      trajectory.push_back(pts_y);
      
      // add to all_trajectories
      result_trajectories.push_back(trajectory);
    }

  }

  return result_trajectories;
}


// generate fine trajectory from coarse trajectory of a few points
// x=0 y=0 is the vehicle position. All ptsx, and ptxy coordinates are in vehicle coordinates 
// given_xyyaw is the location and heading of the car in mapt coordinates
vector<vector<double>> Planner::generate_fine_from_coarse_trajectory(vector<double> ptsx, vector<double> ptsy, vector<double> given_xyyaw)
{

	// result arrays
	vector<double> x_results;
	vector<double> y_results;
	vector<vector<double>> result;

	// create and fit a spline object
    tk::spline sp1;
    sp1.set_points(ptsx, ptsy);

    // maximum delta x from x=0
    double delta_x = ptsx.back(); //-ptsx[0];

    for (int i=1; i<=num_points_in_trajectory; i++) 
    {
    	// calculate x and y along the x axis
    	double x_point = delta_x * (double)i / num_points_in_trajectory;
      double y_point = sp1(x_point);

      // shift to map coordinates
      vector<double> vehicle_coords = Helper::get_map_coords_from_vehicle_coords(x_point, y_point, given_xyyaw);
      x_results.push_back(vehicle_coords[0]);
      y_results.push_back(vehicle_coords[1]);

    }

    // set up results
    result.push_back(x_results);
    result.push_back(y_results);

    return result;
}

// calculate various metrics for a given trajectory
// return values will be delta_s, v_min, v_max, s_2dot_max, d_2dot_max, a_max, s_3dot_max, d_3dot, j_max 
double Planner::estimate_cost_for_trajectory(vector<double> car_xyyawspeed, 
  vector<vector<double>> xy_traj, vector<double> maps_x, vector<double> maps_y, vector<vector<double>> sensor_fusion) 
{

  // current car locations
  double ref_x = car_xyyawspeed[0];
  double ref_y = car_xyyawspeed[1];
  double ref_yaw = car_xyyawspeed[2];
  double ref_v_abs = car_xyyawspeed[3];
  vector<double> veh_coords = Helper::get_vehicle_coords_from_map_coords(ref_x, ref_y, {ref_x, ref_y, ref_yaw});
  double ref_t = veh_coords[0];
  double ref_n = veh_coords[1];
  vector<double> sd_coords = Helper::getFrenet(ref_x, ref_y, ref_yaw, maps_x, maps_y);
  double ref_s = sd_coords[0];
  double ref_d = sd_coords[1];


	// convert trajectory to both vehicle coordinates + Frenet coordinates
	vector<double> t_seq = {}; // vehicle coordinate tangent
	vector<double> n_seq = {}; // vehcile coordinate normal
	vector<vector<double>> tn_traj; // trajectory in vehicle coordinates
  vector<double> s_seq = {}; // s coordinate
  vector<double> d_seq = {}; // d coordinate
  vector<vector<double>> sd_traj; // trajectory in Frenet coordinates
	for (int i=0; i<xy_traj[0].size(); i++) 
	{
		// calcualte vehicle coordinates
    veh_coords = Helper::get_vehicle_coords_from_map_coords(xy_traj[0][i], xy_traj[1][i], {ref_x, ref_y, ref_yaw});
		t_seq.push_back({veh_coords[0]});
		n_seq.push_back({veh_coords[1]});

    // calculate Frenet coordinates
    double theta;
    if (i==0) 
    {
      theta = atan2(xy_traj[1][i]-ref_y,xy_traj[0][i]-ref_x);
    } 
    else 
    {
      theta = atan2(xy_traj[1][i]-xy_traj[1][i-1],xy_traj[0][i]-xy_traj[0][i-1]);
    }
    sd_coords = Helper::getFrenet(xy_traj[0][i], xy_traj[1][i], theta, maps_x, maps_y);
    s_seq.push_back({sd_coords[0]});
    d_seq.push_back({sd_coords[1]});
	}
	tn_traj.push_back(t_seq);
	tn_traj.push_back(n_seq);
  sd_traj.push_back(s_seq);
  sd_traj.push_back(d_seq);

	// calculate the speed vectors
	vector<double> speed_abs;
	for (int i=0; i<xy_traj[0].size(); i++) 
	{
		double v_abs = 0;
		if (i==0)
		{
			v_abs = Helper::distance(ref_x, ref_y, xy_traj[0][i], xy_traj[1][i])/time_interval_between_points;
		}
		else
		{
			v_abs = Helper::distance(xy_traj[0][i-1], xy_traj[1][i-1], xy_traj[0][i], xy_traj[1][i])/time_interval_between_points;
		}
		speed_abs.push_back(v_abs);
	}

  // calculate the acceleration vectors
  vector<double> acc_abs;
  for (int i=0; i<speed_abs.size(); i++) 
  {
    double a_abs = 0;
    if (i==0)
    {
      a_abs = abs(speed_abs[i]-ref_v_abs)/time_interval_between_points;
    }
    else
    {
      a_abs = abs(speed_abs[i]-speed_abs[i-1])/time_interval_between_points;
    }
    
    acc_abs.push_back(a_abs);
  }

  // calcualte the jerk vectors
  vector<double> jerk_abs;
  for (int i=1; i<acc_abs.size(); i++) 
  {
    double j_abs = 0;
    j_abs = abs(acc_abs[i]-acc_abs[i-1])/time_interval_between_points;
    jerk_abs.push_back(j_abs);
  }

  Helper::debug_print("speed_abs: ", speed_abs);
  Helper::debug_print("acc_abs: ", acc_abs);
  Helper::debug_print("jerk_abs: ", jerk_abs);

  // for going over the speed limit
  double cost1 = 0;
  for (int i=0; i<speed_abs.size(); i++)
  {
    if (speed_abs[i]>max_speed / conversion_factor_mps_to_mph)
    {
      cost1 += 1000;
    }
  }

  // for deviating from the target_speed
  double cost2 = 0;
  for (int i=0; i<speed_abs.size(); i++)
  {
    cost2 += 5*abs(speed_abs[i] - target_speed/conversion_factor_mps_to_mph);
  }
  

  // for high acceleration
  double cost3 = 0;
  for (int i=0; i<acc_abs.size(); i++)
  {
    if (acc_abs[i] >= max_acceleration)
    {
      cost3 += 100;
    }
  }

  // for high jerk
  double cost4 = 0;
  for (int i=0; i<jerk_abs.size(); i++)
  {
    if (jerk_abs[i] >= max_jerk)
    {
      cost4 += 100;
    }
  }

  // // for high normal acceleration
  double cost5 = 0;
  // for (int i=0; i<acc_n.size(); i++)
  // {
  //   if (acc_n[i] >= max_acceleration*0.3)
  //   {
  //     cost5 += 2000;
  //   }
  // }

  // // for high normal jerk
  double cost6 = 0;
  // for (int i=0; i<jerk_n.size(); i++)
  // {
  //   if (jerk_n[i] >= max_jerk*0.3)
  //   {
  //     cost6 += 2000;
  //   }
  // }

  // encourage faster speeds
  double cost7 = 0;
  cost7 = (ref_s-sd_traj[0].back())*100;

  // distance from the car ahead
  double cost8 = 0;
  for (int i=0; i<sd_traj[0].size(); i++)
  {
    double cur_point_s = sd_traj[0][i];
    double cur_point_d = sd_traj[1][i];
    int cur_point_lane = get_current_lane_for_d(cur_point_d);
    vector<double> car_ahead_sf_data = sensor_fusion_data_for_car_ahead(sensor_fusion, cur_point_lane, cur_point_s);
    double car_ahead_vx = car_ahead_sf_data[3];
    double car_ahead_vy = car_ahead_sf_data[4];
    double car_ahead_v = sqrt(car_ahead_vx*car_ahead_vx + car_ahead_vy*car_ahead_vy);
    double car_ahead_s = car_ahead_sf_data[5];
    double distance_ahead = car_ahead_s + car_ahead_v*(i+1)*time_interval_between_points - cur_point_s;
    if (distance_ahead < safe_distance_from_other_cars)
    {
      cost8 = (safe_distance_from_other_cars - distance_ahead) * 100;
    }
  }

  cost8 = 0;

  cout << cost1 << " ";
  cout << cost2 << " ";
  cout << cost3 << " ";
  cout << cost4 << " ";
  cout << cost5 << " ";
  cout << cost6 << " ";
  cout << cost7 << " ";
  cout << cost8 << " " << endl;
	return cost1+cost2+cost3+cost4+cost5+cost6+cost7+cost8;
}


// determine if trajectory collides with another car,
// xy_trajectory starts at delta t
// sensor_fusion is for time 0
bool Planner::will_collide(vector<vector<double>> sensor_fusion, vector<vector<double>> xy_trajectory, double delta_t, int car_lane, double car_s)
{
  
  bool result = false;
  vector<int> possible_lanes = generate_possible_lanes_to_explore(car_lane);

  for (int i=0; i<possible_lanes.size(); i++)
  {
    vector<double> car_ahead = sensor_fusion_data_for_car_ahead(sensor_fusion, car_lane+possible_lanes[i], car_s);
    vector<double> car_behind= sensor_fusion_data_for_car_behind(sensor_fusion, car_lane+possible_lanes[i], car_s);

    // define variables for car ahead
    int other_car_id = car_ahead[0];
    double other_car_x = car_ahead[1];
    double other_car_y = car_ahead[2];
    double other_car_vx = car_ahead[3];
    double other_car_vy = car_ahead[4];
    double other_car_s = car_ahead[5];

    // calculate the distance shift due to time lag of xy_trajectory
    double delta_x = other_car_vx * delta_t;
    double delta_y = other_car_vy * delta_t;

    // check for collision with the specified trajectory
    for (int j=0; j<xy_trajectory[0].size(); j++)
    {
      double now_x = other_car_x + other_car_vx * (j+1) * time_interval_between_points + delta_x;
      double now_y = other_car_y + other_car_vy * (j+1) * time_interval_between_points + delta_y;
      if (Helper::distance(now_x, now_y, xy_trajectory[0][j], xy_trajectory[1][j]) <= collision_distance) 
      {
          result = true;
      } 
    }

    if (possible_lanes[i] != 0)
    {
      // define variables for car behind
      other_car_id = car_behind[0];
      other_car_x = car_behind[1];
      other_car_y = car_behind[2];
      other_car_vx = car_behind[3];
      other_car_vy = car_behind[4];
      other_car_s = car_behind[5];

      // calculate the distance shift due to time lag of xy_trajectory
      delta_x = other_car_vx * delta_t;
      delta_y = other_car_vy * delta_t;

      // check for collision with the specified trajectory
      for (int j=0; j<xy_trajectory[0].size(); j++)
      {
        double now_x = other_car_x + other_car_vx * (j+1) * time_interval_between_points + delta_x;
        double now_y = other_car_y + other_car_vy * (j+1) * time_interval_between_points + delta_y;
        if (Helper::distance(now_x, now_y, xy_trajectory[0][j], xy_trajectory[1][j]) <= collision_distance) 
          {
            result = true;
          } 
      }
    }

  }

  return result;
}

// return sensor_fusion data for the car immediately ahead in the specified lane
vector<double> Planner::sensor_fusion_data_for_car_ahead(vector<vector<double>> sensor_fusion, int ref_lane, double car_s)
{

  double min_s = 10000.0;  
  int result_index = 0;

  for (int i=0; i<sensor_fusion.size(); i++)
    {
      double check_d = sensor_fusion[i][6];
      if (check_d < lane_width * (ref_lane + 1) && (check_d >  lane_width * ref_lane))
      {
        double check_s = sensor_fusion[i][5];
        if ((check_s>car_s) && (check_s<min_s))
        {
          min_s = check_s;
          result_index = i;
        }
      }
    }
  return (sensor_fusion[result_index]);
}


// return sensor_fusion data for the car immediately behind in the specified lane
vector<double> Planner::sensor_fusion_data_for_car_behind(vector<vector<double>> sensor_fusion, int ref_lane, double car_s)
{

  double max_s = -10000.0;  
  int result_index = 0;

  for (int i=0; i<sensor_fusion.size(); i++)
    {
      double check_d = sensor_fusion[i][6];
      if (check_d < lane_width * (ref_lane + 1) && (check_d >  lane_width * ref_lane))
      {
        double check_s = sensor_fusion[i][5];
        if ((check_s<car_s) && (check_s>max_s))
        {
          max_s = check_s;
          result_index = i;
        }
      }
    }
  return (sensor_fusion[result_index]);
}
