#include "planner.h"

Planner::Planner() {}

Planner::~Planner() {}

// return current lane based on d
int Planner::get_lane_for_d(double d) 
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
double Planner::get_d_for_lane(int lane) 
{
	return (lane_width/2+lane_width*lane);
}


// update state
void Planner::update_state(bool too_close_ahead, double end_d)
{
  double target_d = get_d_for_lane(target_lane);
  if (state=="KL" && too_close_ahead)
  {
    state = "ECL";
  }
  else if (state=="PCL" && abs(target_d-end_d)<0.2)
  {
    state = "KL";
  }
  else if (state=="ECL" && abs(target_d-end_d)>0.2)
  {
    state = "PCL";
  }
}

// generate all possible states from the current state
vector<int> Planner::possible_lanes_to_explore(int ref_lane)
{

  vector<int> possible_lanes;
  
  if (state=="ECL")
  {
    possible_lanes.push_back(ref_lane);
    if (ref_lane < number_of_lanes-1) 
    {
      possible_lanes.push_back(ref_lane + 1);
    } 
    if (ref_lane > 0) 
    {
      possible_lanes.push_back(ref_lane - 1);
    }    
  }
  else if (state=="PCL")
  {
    possible_lanes.push_back(target_lane);  
  }
  else if (state=="KL")
  {
    possible_lanes.push_back(ref_lane);   
  }
 
  //return possible_lanes;
  return possible_lanes;
  //return {ref_lane};
}



// generate all possible tranjectories using only few points that are spaced far apart
// will return points in xy coordinates - i.e. map coordinates
vector<vector<vector<double>>> Planner::generate_trajectory_coarse(vector<int> lanes_to_explore, double ref_s, vector<double> end_xyyaw, vector<double> prev_xyyaw, 
  vector<double> maps_s, vector<double> maps_x, vector<double> maps_y) 
{

  // result
  vector<vector<vector<double>>> result_trajectories;  

  // generate trajectories
  for (int i=0; i<lanes_to_explore.size(); i++)
  {

    int ref_lane = lanes_to_explore[i];

    // define vectors
    vector<double> pts_x; 
    vector<double> pts_y;

    double target1_s = ref_s + 30;
    double target2_s = target1_s + 5;
    double target3_s = target2_s + 5;
    double target1_d = get_d_for_lane(ref_lane);
    double target2_d = target1_d;
    double target3_d = target2_d;

    // convert to map coordinate
    vector<double> target1_xy = Helper::getXY(target1_s, target1_d, maps_s, maps_x, maps_y);
    vector<double> target2_xy = Helper::getXY(target2_s, target2_d, maps_s, maps_x, maps_y);
    vector<double> target3_xy = Helper::getXY(target3_s, target3_d, maps_s, maps_x, maps_y);
    
    // create waypoints for s
    pts_x.push_back(prev_xyyaw[0]);
    pts_x.push_back(end_xyyaw[0]);
    pts_x.push_back(target1_xy[0]);
    pts_x.push_back(target2_xy[0]);
    pts_x.push_back(target3_xy[0]);
    
    // create wayponts for d
    pts_y.push_back(prev_xyyaw[1]);
    pts_y.push_back(end_xyyaw[1]);
    pts_y.push_back(target1_xy[1]);
    pts_y.push_back(target2_xy[1]);
    pts_y.push_back(target3_xy[1]);

    // convert to vehicle coordiantes
    vector<double> veh_x_points;
    vector<double> veh_y_points;
    for (int i=0; i<pts_x.size(); i++)
    {
      vector<double> vehicle_coords = Helper::get_vehicle_coords_from_map_coords(pts_x[i], pts_y[i], end_xyyaw);
      veh_x_points.push_back(vehicle_coords[0]);
      veh_y_points.push_back(vehicle_coords[1]);
    }

    // create and fit a spline object
    tk::spline sp1;
    sp1.set_points(veh_x_points, veh_y_points);

    // maximum distance without going over max_speed
    double target_x = veh_x_points.back();
    double target_y = veh_y_points.back();
    double target_dist = Helper::distance(0, 0, target_x, target_y);

    double increment_distance = time_interval_between_points * (target_speed / conversion_factor_mps_to_mph);
    double num_points = (target_dist / increment_distance);


    vector<double> x_results;
    vector<double> y_results;

    for (int j=0; j<(int)floor(num_points); j++) 
    {

      double x_point = target_x * (j+1) / num_points;
      double y_point = sp1(x_point);

      // convert to map coordinates
      vector<double> vehicle_coords = Helper::get_map_coords_from_vehicle_coords(x_point, y_point, end_xyyaw);

      x_results.push_back(vehicle_coords[0]);
      y_results.push_back(vehicle_coords[1]);

    }  

    
    // add to all_trajectories
    result_trajectories.push_back({x_results, y_results});

  }

  return result_trajectories;

}


// generate fine trajectory from coarse trajectory of a few points
// x=0 y=0 is the vehicle position in ptsx and ptsy. i.e. All ptsx, and ptxy coordinates are in vehicle coordinates 
// given_xyyaw is the location and heading of the car in mapt coordinates
vector<vector<double>> Planner::generate_fine_trajectory_at_target_speed(vector<double> ptsx, vector<double> ptsy, vector<double> given_xyyaw, bool verbose)
{

  // convert to vehicle coordiantes
  vector<double> veh_x_points;
  vector<double> veh_y_points;
  for (int i=0; i<ptsx.size(); i++)
  {
    vector<double> vehicle_coords = Helper::get_vehicle_coords_from_map_coords(ptsx[i], ptsy[i], given_xyyaw);
    veh_x_points.push_back(vehicle_coords[0]);
    veh_y_points.push_back(vehicle_coords[1]);
  }

  // create and fit a spline object
  tk::spline sp1;
  sp1.set_points(veh_x_points, veh_y_points);

  // maximum distance without going over max_speed
  double target_x = target_speed / conversion_factor_mps_to_mph * num_points_in_trajectory * time_interval_between_points;
  double target_y = sp1(target_x);
  double target_dist = Helper::distance(0, 0, target_x, target_y);

  double increment_distance = time_interval_between_points * (target_speed / conversion_factor_mps_to_mph);
  double num_points = (target_dist / increment_distance);

  vector<double> x_results;
  vector<double> y_results;

  for (int j=0; j<num_points_in_trajectory; j++) 
  {

    double x_point = target_x * (j+1) / num_points;
    double y_point = sp1(x_point);

    vector<double> vehicle_coords = Helper::get_map_coords_from_vehicle_coords(x_point, y_point, given_xyyaw);

    x_results.push_back(vehicle_coords[0]);
    y_results.push_back(vehicle_coords[1]);

  }

  return {x_results, y_results};
}


// generate fine trajectory from coarse trajectory of a few points
// x=0 y=0 is the vehicle position in ptsx and ptsy. i.e. All ptsx, and ptxy coordinates are in vehicle coordinates 
// given_xyyaw is the location and heading of the car in mapt coordinates
vector<vector<vector<double>>> Planner::generate_fine_from_coarse_trajectory(vector<double> ptsx, vector<double> ptsy, vector<double> given_xyyaw, bool verbose)
{


  if (verbose)
  {
    Helper::debug_print("ptsx in generate_fine... ", ptsx);
    Helper::debug_print("ptsx in generate_fine... ", ptsy);
  }

	// result arrays
	vector<vector<vector<double>>> result;

	// create and fit a spline object
  tk::spline sp1;
  sp1.set_points(ptsx, ptsy);

  // maximum distance without going over max_speed
  double target_x = max_speed / conversion_factor_mps_to_mph * num_points_in_trajectory * time_interval_between_points;
  double target_y = sp1(target_x);
  double target_dist = Helper::distance(0, 0, target_x, target_y);

  if (verbose) 
  {
    Helper::debug_print("target_x, target_y, target_dist: ", {target_x, target_y, target_dist});
  }

  for (int i=0; i<num_trajectories_for_each_lane; i++)
  {

    double ref_speed = (double)(i+1) / num_trajectories_for_each_lane * target_speed * 1.2;
    double increment_distance = time_interval_between_points * (ref_speed / conversion_factor_mps_to_mph);
    double num_points = (target_dist / increment_distance);

    if (verbose) 
    {
      Helper::debug_print("ref_speed, increment_distance, num_points: ", {ref_speed, increment_distance, num_points});
    }


    vector<double> x_results;
    vector<double> y_results;

    for (int j=0; j<num_points_in_trajectory; j++) 
    {

      double x_point = target_x * (j+1) / num_points;
      double y_point = sp1(x_point);

      vector<double> vehicle_coords = Helper::get_map_coords_from_vehicle_coords(x_point, y_point, given_xyyaw);

      x_results.push_back(vehicle_coords[0]);
      y_results.push_back(vehicle_coords[1]);

    }

    // set up results
    result.push_back({x_results, y_results});

  }

  return result;
}

// calculate various metrics for a given trajectory
// return values will be delta_s, v_min, v_max, s_2dot_max, d_2dot_max, a_max, s_3dot_max, d_3dot, j_max 
double Planner::estimate_cost_for_trajectory(vector<double> car_xyyawspeed, vector<double> car_sd, 
  vector<vector<double>> xy_traj, vector<double> maps_x, vector<double> maps_y, vector<vector<double>> sensor_fusion, bool verbose) 
{

  bool collision = will_collide(sensor_fusion, xy_traj, car_sd[0], num_points_in_trajectory);
  if (collision)
  {
    return 1e10;
  }

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
  vector<double> s_seq = {}; // s coordinate
  vector<double> d_seq = {}; // d coordinate
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
	

  int start_lane = get_lane_for_d(car_sd[1]);
  int end_lane = get_lane_for_d(d_seq.back());

  double dist_to_car_ahead = distance_to_nearby_car(car_sd[0], end_lane, sensor_fusion, true);
  double dist_to_car_behind = distance_to_nearby_car(car_sd[0], end_lane, sensor_fusion, false);
  double speed_of_car_ahead = speed_of_nearby_car(car_sd[0], end_lane, sensor_fusion, true);
  bool is_there_car_ahead = is_there_any_car_nearby(car_sd[0], end_lane, sensor_fusion, true);
  bool is_there_car_behind = is_there_any_car_nearby(car_sd[0], end_lane, sensor_fusion, false);
  
  // encourage faster speeds
  double cost = 0;
  if (!is_there_car_ahead)
  {
    cost += -100;
  }
  else if (dist_to_car_ahead<collision_distance)
  {
    cost += 100;
  }
  else 
  {
    cost += 100-(dist_to_car_ahead-collision_distance);
  }

  if (is_there_car_behind && abs(dist_to_car_behind)<passing_distance)
  {
    cost += 250;
  }
  

	return cost;
}


// determine if trajectory collides with another car,
// sensor_fusion is for time 0
bool Planner::will_collide(vector<vector<double>> sensor_fusion, vector<vector<double>> xy_trajectory, double car_s, int num_points_to_check)
{
  
  vector<vector<double>> cars_to_check_sf;

  for (int i=0; i<number_of_lanes; i++)
  {
    vector<double> car_ahead = sensor_fusion_data_for_car_ahead(sensor_fusion, i, car_s);
    vector<double> car_behind = sensor_fusion_data_for_car_behind(sensor_fusion, i, car_s);
    if (car_ahead.size()!=0) {cars_to_check_sf.push_back(car_ahead);}
    if (car_behind.size()!=0) {cars_to_check_sf.push_back(car_behind);}
  }


  for (int i=0; i<cars_to_check_sf.size(); i++)
  {
    vector<double> check_car = cars_to_check_sf[i];
    double check_x = check_car[1];
    double check_y = check_car[2];
    double check_vx = check_car[3];
    double check_vy = check_car[4];
    double check_v = sqrt(check_vx*check_vx + check_vy*check_vy);

    double cur_x = check_x;
    double cur_y = check_y;

    // check trajectory for collision
    for (int j=0; j<num_points_to_check; j++)
    {
      // calculate distance with other car
      double cur_dist = Helper::distance(cur_x, cur_y, xy_trajectory[0][j], xy_trajectory[1][j]);
      if (cur_dist<collision_distance) {return true;}

      // update location
      cur_x += check_vx * time_interval_between_points;
      cur_y += check_vy * time_interval_between_points;
    }
  }

  return false;
}

// return sensor_fusion data for the car immediately ahead in the specified lane
vector<double> Planner::sensor_fusion_data_for_car_ahead(vector<vector<double>> sensor_fusion, int ref_lane, double car_s)
{

  double min_s = 10000.0;  
  int result_index = -1;
  vector<double> result_sf = {};

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
          result_sf = sensor_fusion[result_index];
        }
      }
    }
  return result_sf;
}


// return sensor_fusion data for the car immediately behind in the specified lane
vector<double> Planner::sensor_fusion_data_for_car_behind(vector<vector<double>> sensor_fusion, int ref_lane, double car_s)
{

  double max_s = -10000.0;  
  int result_index = -1;
  vector<double> result_sf = {};

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
          result_sf = sensor_fusion[result_index];
        }
      }
    }
  return result_sf;
}


// return optimized trajectory from a given trajectory. will spread the points along the trajectory to minimize acceleration change
  vector<vector<double>> Planner::optimize_trajectory(vector<double> car_xyyaw, vector<vector<double>> xy_traj)
  {

    // convert to vehicle coordinates
    int N = xy_traj[0].size();
    vector<double> t_traj;
    vector<double> n_traj;
    for (int i=0; i<N; i++) 
    {
      vector<double> veh_coords = Helper::get_vehicle_coords_from_map_coords(xy_traj[0][i], xy_traj[1][i], car_xyyaw);
      t_traj.push_back(veh_coords[0]);
      n_traj.push_back(veh_coords[1]);
    }


    // calculate total length of trejectory, initial speed, final speed, delta speed
    double distance = 0;
    distance += Helper::distance(0, 0, t_traj[0], n_traj[0]);
    for (int i=0; i<N; i++)
    {
      distance += Helper::distance(t_traj[i], n_traj[i], t_traj[i+1], n_traj[i+1]);
    }

    double initial_velocity = Helper::distance(0, 0, t_traj[0], n_traj[0])/time_interval_between_points;
    double final_velocity = Helper::distance(t_traj[N-2], n_traj[N-2], t_traj[N-1], n_traj[N-1])/time_interval_between_points;
    double delta_velocity = (final_velocity - initial_velocity)/N;

    // create and fit a spline object
    tk::spline sp1;
    sp1.set_points(t_traj, n_traj); 


    // build new trajectory
    vector<double> new_t_traj;
    vector<double> new_n_traj;

    // convert to map coordinate
    double current_velocity = initial_velocity;
    double prev_t = 0;
    double prev_n = 0;
    double current_t = t_traj[0];
    double current_n = n_traj[0];
    new_t_traj.push_back(current_t);
    new_n_traj.push_back(current_n);


    for (int i=1; i<N; i++)
    {
      double current_heading = atan2(current_n-prev_n, current_t-prev_t);
      current_velocity += delta_velocity;
      double delta_s = current_velocity * time_interval_between_points;
      double detla_t = delta_s * cos(current_heading);
      prev_t = current_t;
      prev_n = current_n;
      current_t += detla_t;
      current_n = sp1(current_t);
      //
      new_t_traj.push_back(current_t);
      new_n_traj.push_back(current_n);  
    }
    

    // convert back to map coordinates
    vector<double> new_x_traj;
    vector<double> new_y_traj;
    for (int i=0; i<N; i++) 
    {
      vector<double> new_map_coords = Helper::get_map_coords_from_vehicle_coords(new_t_traj[i], new_n_traj[i], car_xyyaw);
      new_x_traj.push_back(new_map_coords[0]);
      new_y_traj.push_back(new_map_coords[1]);
    }    


    // returen the reuslt
    return {new_x_traj, new_y_traj};

  }


  // determine if the vehicle is too close to the car ahead
  bool Planner::is_too_close_ahead(vector<vector<double>> sensor_fusion, int ref_lane, double ref_s, double time_shift)
  {

    // get sensor fusion data
    vector<double> car_ahead = sensor_fusion_data_for_car_ahead(sensor_fusion, ref_lane, ref_s);
    if (car_ahead.size()!=0)
    {
      double check_vx = car_ahead[3];
      double check_vy = car_ahead[4];
      double check_v = sqrt(check_vx*check_vx + check_vy*check_vy);
      double check_s = car_ahead[5] + time_shift * check_v;

      if (check_s-ref_s < safe_distance_from_other_cars) {return true;}
    }

    return false;
  }


  // update target speed
  void Planner::update_target_speed(bool is_too_close_ahead, double speed_of_car_ahead) 
  {
    if ((is_too_close_ahead && (target_speed > speed_of_car_ahead)) || target_speed > max_speed - 0.5)
    {
      target_speed -= 0.2;
    }
    else if (target_speed < max_speed - 0.5)
    {
      target_speed += 0.30; 
    }
  }


  // determines if there is any car nearby
  bool Planner::is_there_any_car_nearby(double ref_s, int ref_lane, vector<vector<double>> sensor_fusion, bool ahead)
  {

    // get sensor fusion data
    vector<double> nearby_car;
    if (ahead)
    {
      nearby_car = sensor_fusion_data_for_car_ahead(sensor_fusion, ref_lane, ref_s);
    }
    else
    {
      nearby_car = sensor_fusion_data_for_car_behind(sensor_fusion, ref_lane, ref_s); 
    }

    // return result
    if (nearby_car.size()==0)
    {
      return false;
    }
    else
    {
      return true;
    }
  }


  // distance to car ahead
  double Planner::distance_to_nearby_car(double ref_s, int ref_lane, vector<vector<double>> sensor_fusion, bool ahead)
  {

    // get sensor fusion data
    vector<double> nearby_car;
    if (ahead)
    {
      nearby_car = sensor_fusion_data_for_car_ahead(sensor_fusion, ref_lane, ref_s);
    }
    else
    {
      nearby_car = sensor_fusion_data_for_car_behind(sensor_fusion, ref_lane, ref_s); 
    }

    // calculate s
    double nearby_car_s;
    if (nearby_car.size()==0)
    {
      nearby_car_s = ref_s;
    }
    else
    {
      nearby_car_s = nearby_car[5]; 
    }
    return (nearby_car_s - ref_s);
  }


  // speed of the car ahead
  double Planner::speed_of_nearby_car(double ref_s, int ref_lane, vector<vector<double>> sensor_fusion, bool ahead)
  {

    // get sensor fusion data
    vector<double> nearby_car;
    if (ahead)
    {
      nearby_car = sensor_fusion_data_for_car_ahead(sensor_fusion, ref_lane, ref_s);
    }
    else
    {
      nearby_car = sensor_fusion_data_for_car_behind(sensor_fusion, ref_lane, ref_s); 
    }

    // calculate v
    double nearby_car_speed;
    if (nearby_car.size()==0)
    {
      nearby_car_speed = -1;
    }
    else
    {
      double nearby_car_vx = nearby_car[3];
      double nearby_car_vy = nearby_car[4];
      nearby_car_speed = sqrt(nearby_car_vx*nearby_car_vx + nearby_car_vy*nearby_car_vy) * conversion_factor_mps_to_mph;
    }
    return (nearby_car_speed);

  }

