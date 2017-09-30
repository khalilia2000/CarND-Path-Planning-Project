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
    state = "ECL"; // Explore Change Lane
  }
  else if (state=="PCL" && abs(target_d-end_d)<0.2)
  {
    state = "KL";  // Keep Lane
  }
  else if (state=="ECL" && abs(target_d-end_d)>0.2)
  {
    state = "PCL"; // Perform Change Lane
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
}



// generate all possible tranjectories using only few points that are spaced far apart
// will return points in xy coordinates - i.e. map coordinates
vector<vector<vector<double>>> Planner::generate_trajectories(vector<int> lanes_to_explore, double ref_s, vector<double> end_xyyaw, vector<double> prev_xyyaw, 
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

    double target1_s = ref_s + 40;
    double target2_s = target1_s + 10;
    double target3_s = target2_s + 10;
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


// calculate cost for a given trajectory
double Planner::estimate_cost_for_trajectory(vector<double> car_xyyawspeed, vector<double> car_sd, 
  vector<vector<double>> xy_traj, vector<double> maps_x, vector<double> maps_y, vector<vector<double>> sensor_fusion) 
{

  bool collision = will_collide(sensor_fusion, xy_traj, car_sd[0], num_points_in_trajectory);
  if (collision)
  {
    return 1e10;
  }

	// obtain Frenet coordinates for the last point of the trajectory
  int traj_size = xy_traj[0].size();
  double theta = atan2(xy_traj[1][traj_size-1]-xy_traj[1][traj_size-2],xy_traj[0][traj_size-1]-xy_traj[0][traj_size-2]);
  vector<double> sd_coords = Helper::getFrenet(xy_traj[0].back(), xy_traj[1].back(), theta, maps_x, maps_y);

  // lane at the end of the trajectory
  int end_lane = get_lane_for_d(sd_coords[1]);

  // cars ahead and behind the end of trajectory
  double dist_to_car_ahead = distance_to_nearby_car(car_sd[0], end_lane, car_xyyawspeed, sensor_fusion, true);
  double dist_to_car_behind = distance_to_nearby_car(car_sd[0], end_lane, car_xyyawspeed, sensor_fusion, false);
  double speed_of_car_ahead = speed_of_nearby_car(car_sd[0], end_lane, sensor_fusion, true);
  bool is_there_car_ahead = is_there_any_car_nearby(car_sd[0], end_lane, sensor_fusion, true);
  bool is_there_car_behind = is_there_any_car_nearby(car_sd[0], end_lane, sensor_fusion, false);

  // calcualte cost
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
    cost += 450;
  }
  

	return cost;
}


// determine if trajectory collides with another car
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
    target_speed *= 0.99;
  }
  else if (target_speed < max_speed - 1.0)
  {
    target_speed *= 1.01;
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
double Planner::distance_to_nearby_car(double ref_s, int ref_lane, vector<double> ref_xy, vector<vector<double>> sensor_fusion, bool ahead)
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

  // calculate distance
  double distance = -1;
  if (nearby_car.size()!=0)
  {
    distance = Helper::distance(ref_xy[0], ref_xy[1], nearby_car[1], nearby_car[2]); 
  }
  return distance;
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

