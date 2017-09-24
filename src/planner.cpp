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
  
  double delta_s = max_speed;
  delta_s *= conversion_factor_mph_to_mps;
  delta_s *= num_points_in_trajectory * time_interval_between_points;
  delta_s -= smoothing_distance;

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
      double target1_s = given_s + delta_s / num_trajectories_for_each_lane * i;
      double target2_s = target1_s + smoothing_distance/2;
      double target3_s = target2_s + smoothing_distance/2;
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
  vector<vector<double>> xy_traj, vector<double> maps_x, vector<double> maps_y) 
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
	vector<double> t_seq; // vehicle coordinate tangent
	vector<double> n_seq; // vehcile coordinate normal
	vector<vector<double>> tn_traj; // trajectory in vehicle coordinates
  vector<double> s_seq; // s coordinate
  vector<double> d_seq; // d coordinate
  vector<vector<double>> sd_traj; // trajectory in Frenet coordinates
	for (int i=0; i<xy_traj.size(); i++) 
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
  vector<double> speed_t;
  vector<double> speed_n;
  vector<double> speed_s;
  vector<double> speed_d;
	for (int i=0; i<xy_traj.size(); i++) 
	{
		double v_abs = 0;
    double v_t = 0;
    double v_n = 0;
    double v_s = 0;
    double v_d = 0;
		if (i==0)
		{
			v_abs = Helper::distance(ref_x, ref_y, xy_traj[0][i], xy_traj[1][i])/time_interval_between_points;
      v_t = Helper::distance(ref_t, 0, tn_traj[0][i], 0)/time_interval_between_points;
      v_n = Helper::distance(0, ref_n, 0, tn_traj[1][i])/time_interval_between_points;
      v_s = Helper::distance(ref_s, 0, sd_traj[0][i], 0)/time_interval_between_points;
      v_d = Helper::distance(0, ref_d, 0, sd_traj[1][i])/time_interval_between_points;
		}
		else
		{
			v_abs = Helper::distance(xy_traj[0][i-1], xy_traj[1][i-1], xy_traj[0][i], xy_traj[1][i])/time_interval_between_points;
      v_t = Helper::distance(tn_traj[0][i-1], 0, tn_traj[0][i], 0)/time_interval_between_points;
      v_n = Helper::distance(0, tn_traj[1][i-1], 0, tn_traj[1][i])/time_interval_between_points;
      v_s = Helper::distance(sd_traj[0][i-1], 0, sd_traj[0][i], 0)/time_interval_between_points;
      v_d = Helper::distance(0, sd_traj[1][i-1], 0, sd_traj[1][i])/time_interval_between_points;
		}
		speed_abs.push_back(v_abs);
    speed_t.push_back(v_t);
    speed_n.push_back(v_n);
    speed_s.push_back(v_s);
    speed_d.push_back(v_d);
	}

  // calculate the acceleration vectors
  vector<double> acc_abs;
  vector<double> acc_t;
  vector<double> acc_n;
  vector<double> acc_s;
  vector<double> acc_d;
  for (int i=1; i<speed_abs.size(); i++) 
  {
    double a_abs = 0;
    double a_t = 0;
    double a_n = 0;
    double a_s = 0;
    double a_d = 0;
    a_abs = (speed_abs[i]-speed_abs[i-1])/time_interval_between_points;
    a_t = (speed_t[i]-speed_t[i-1])/time_interval_between_points;
    a_n = (speed_n[i]-speed_n[i-1])/time_interval_between_points;
    a_s = (speed_s[i]-speed_s[i-1])/time_interval_between_points;
    a_d = (speed_d[i]-speed_d[i-1])/time_interval_between_points;
    acc_abs.push_back(a_abs);
    acc_t.push_back(a_t);
    acc_n.push_back(a_n);
    acc_s.push_back(a_s);
    acc_d.push_back(a_d);
  }

  // calcualte the jerk vectors
  vector<double> jerk_abs;
  vector<double> jerk_t;
  vector<double> jerk_n;
  vector<double> jerk_s;
  vector<double> jerk_d;
  for (int i=1; i<acc_abs.size(); i++) 
  {
    double j_abs = 0;
    double j_t = 0;
    double j_n = 0;
    double j_s = 0;
    double j_d = 0;
    j_abs = (acc_abs[i]-acc_abs[i-1])/time_interval_between_points;
    j_t = (acc_t[i]-acc_t[i-1])/time_interval_between_points;
    j_n = (acc_n[i]-acc_n[i-1])/time_interval_between_points;
    j_s = (acc_s[i]-acc_s[i-1])/time_interval_between_points;
    j_d = (acc_d[i]-acc_d[i-1])/time_interval_between_points;
    jerk_abs.push_back(j_abs);
    jerk_t.push_back(j_t);
    jerk_n.push_back(j_n);
    jerk_s.push_back(j_s);
    jerk_d.push_back(j_d);
  }

  // for going over the speed limit
  double cost1 = 0;
  for (int i=0; i<speed_abs.size(); i++)
  {
    if (speed_abs[i]>max_speed)
    {
      cost1 += 1000;
    }
  }

  // for deviating from the target_speed
  double cost2 = 0;
  for (int i=0; i<speed_abs.size(); i++)
  {
    cost2 += 1000*abs(speed_abs[i]-target_speed);
  }

  // for high acceleration
  double cost3 = 0;
  for (int i=0; i<acc_abs.size(); i++)
  {
    if (acc_abs[i] >= max_acceleration)
    {
      cost3 += 1000;
    }
  }

  // for high jerk
  double cost4 = 0;
  for (int i=0; i<jerk_abs.size(); i++)
  {
    if (jerk_abs[i] >= max_jerk)
    {
      cost4 += 1000;
    }
  }

  // for high normal acceleration
  double cost5 = 0;
  for (int i=0; i<acc_n.size(); i++)
  {
    if (acc_n[i] >= max_acceleration*0.3)
    {
      cost5 += 2000;
    }
  }

  // for high normal jerk
  double cost6 = 0;
  for (int i=0; i<jerk_n.size(); i++)
  {
    if (jerk_n[i] >= max_jerk*0.3)
    {
      cost6 += 2000;
    }
  }

	return cost1+cost2+cost3+cost4+cost5+cost6;
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
          cout << "collision predicted with car: " << other_car_id << "with s = " << other_car_s << endl;
      } 
    }

    if (possible_lanes[i] != 0)
    {
      // define variables for car behind
      other_car_x = car_behind[1];
      other_car_y = car_behind[2];
      other_car_vx = car_behind[3];
      other_car_vy = car_behind[4];

      // calculate the distance shift due to time lag of xy_trajectory
      delta_x = other_car_vx * delta_t;
      delta_y = other_car_vy * delta_t;

      // check for collision with the specified trajectory
      for (int j=0; j<xy_trajectory[0].size(); j++)
      {
        double now_x = other_car_x + other_car_vx * (j+1) * time_interval_between_points + delta_x;
        double now_y = other_car_y + other_car_vy * (j+1) * time_interval_between_points + delta_y;
        if (Helper::distance(now_x, now_y, xy_trajectory[0][j], xy_trajectory[1][j]) <= collision_distance) {result = true;} 
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
