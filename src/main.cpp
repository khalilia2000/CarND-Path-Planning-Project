#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "helper.h"
#include "planner.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// verbose state for debugging
int verbose_counter = 0;
bool verbose() { 
	return (verbose_counter>0); 
}

void turn_verbose_off() {
  verbose_counter = 0;
}

void turn_verbose_on() {
  verbose_counter = 1;
}


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int master_counter = 0;
int get_counter() 
{
	master_counter++;
	return(master_counter);
}


int main() {

  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }


  // AK defined variables
  Planner p;
  p.target_speed = 0;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&p](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

            //  vectors to be passed on to the simulator
          	vector<double> next_x_vals;
          	vector<double> next_y_vals;
            
            // AK Begin
              //Helper::debug_print("max_speed, target_speed, car_speed: ", {p.max_speed, p.target_speed, car_speed});
        	  if (p.target_speed < p.max_speed - 1.0)
        	  {
        	  	p.target_speed += 0.5;
        	  }

	          // Define all variables
	          vector<double> ptsx;
	          vector<double> ptsy;
	            
              double end_pos_x = 0; 
              double end_pos_y = 0;
              double end_pos_s = 0;
              double end_pos_d = 0;
              double ref_yaw = 0;
              double ref_speed = 0;
              double ref_distance = 0;
              double prev_pos_x = 0;
              double prev_pos_y = 0;
              int path_size = previous_path_x.size();

              int tmp_cntr = get_counter();
              if (verbose())
              {
                cout << endl;
                cout << endl;
                cout << "------------------------" << endl;
                cout << "counter: " << tmp_cntr << endl;
                cout << "------------------------" << endl;
              }

              // create initial 2 points with right heading from the current car location
              // or the end of previous path
              if(path_size < 2)
              {
                end_pos_x = car_x; 
                end_pos_y = car_y;
                ref_yaw = Helper::deg2rad(car_yaw);
                prev_pos_x = end_pos_x - cos(ref_yaw)*1.0;
                prev_pos_y = end_pos_y - sin(ref_yaw)*1.0;
                ref_distance = 1.0;
                ref_speed = car_speed;
                if (verbose())
                {
                  cout << "Starting with loop1 - path_size = " << path_size << endl;
                  Helper::debug_print("car_yaw, ref_yaw: ", {car_yaw, ref_yaw});
                }
              }
              else
              {
                // claculate the position of the car at the end of the previous path
                end_pos_x = previous_path_x[path_size-1];
                end_pos_y = previous_path_y[path_size-1];
                // calculate the angle of the car at the end of the previous path
                prev_pos_x = previous_path_x[path_size-2];
                prev_pos_y = previous_path_y[path_size-2];
                ref_yaw = atan2(end_pos_y-prev_pos_y,end_pos_x-prev_pos_x);
                ref_distance = Helper::distance(end_pos_x, end_pos_y, prev_pos_x, prev_pos_y);
                ref_speed = ref_distance / p.time_interval_between_points;
                if (verbose())
                {
                  cout << "Starting with loop2 - path_size = " << path_size << endl;
                  Helper::debug_print("car_yaw, ref_yaw: ", {car_yaw, ref_yaw});
                }
              }

              end_pos_s = end_path_s;
              end_pos_d = end_path_d;

              // generate trajectories
              int car_lane = p.get_lane_for_d(car_d);
              int end_pos_lane = p.get_lane_for_d(end_pos_d);
              vector<int> abs_possible_lanes = {1};
              auto all_coarse_trajectories = p.generate_trajectory_coarse(abs_possible_lanes, car_s, {end_pos_x, end_pos_y}, {prev_pos_x, prev_pos_y}, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    

              if (verbose())
  	          {
  	          	  Helper::debug_print("end_pos_lane: ", {(double)end_pos_lane});
  	              Helper::debug_print("car x,y,s,d,yaw, speed: ", {car_x, car_y, car_s, car_d, car_yaw, car_speed});
  	              Helper::debug_print("end_pos_x,y,s,d: ", {end_pos_x, end_pos_y, end_pos_s, end_pos_d});
  	              Helper::debug_print("prev_pos_x,y: ", {prev_pos_x, prev_pos_y});
  	              Helper::debug_print("ref_yaw, ref_speed: ", {ref_yaw, ref_speed});
                  Helper::debug_print("previous_path_x: ", previous_path_x);
                  Helper::debug_print("previous_path_y: ", previous_path_y);

  	              cout << "number of generated coarse trajectories: " << all_coarse_trajectories.size() << endl;
  	              for (int i=0; i<all_coarse_trajectories.size(); i++)
  	              {
  	                cout << "trajectory " << i << endl;
  	                Helper::debug_print("map x_points: ", all_coarse_trajectories[i][0]);
  	                Helper::debug_print("map y_points: ", all_coarse_trajectories[i][1]);
  	              }
  	              cout << "----------------" << endl;
  	          }

              // generate costs
              vector<double> all_costs;
              vector<vector<vector<double>>> all_fine_trajectories;
              for (int i=0; i<all_coarse_trajectories.size(); i++) 
              {

                // select trajectory
                auto current_coarse_trajectory = all_coarse_trajectories[i];

                // convert from Frenet to car reference coordinates
                for (int j = 0; j<current_coarse_trajectory[0].size(); j++)
                {
                  vector<double> vehicle_coords = Helper::get_vehicle_coords_from_map_coords(current_coarse_trajectory[0][j], current_coarse_trajectory[1][j], {end_pos_x, end_pos_y, ref_yaw});
                  current_coarse_trajectory[0][j] = vehicle_coords[0];
                  current_coarse_trajectory[1][j] = vehicle_coords[1];
                }

                // convert to fine trajectory
                auto cur_fine_traj = p.generate_fine_from_coarse_trajectory(current_coarse_trajectory[0], current_coarse_trajectory[1], {end_pos_x, end_pos_y, ref_yaw}, verbose());
                

                // copy unused points from previous path received from the simulator
                for (int j=0; j<cur_fine_traj.size(); j++)
                {

                  auto combined_fine_trajectory = Helper::combine_trajectories({previous_path_x, previous_path_y}, cur_fine_traj[j], p.num_points_in_trajectory);  
                  all_fine_trajectories.push_back(combined_fine_trajectory);


                  if (verbose()) 
                  {
                    cout << "------------" << endl;
                    cout << "fine trajectory " << j << " from coarse trajectory " << i << endl;
                    Helper::debug_print("fine_traj x: ", combined_fine_trajectory[0]);
                    Helper::debug_print("fine_traj y: ", combined_fine_trajectory[1]);
                  }


                  // calculate and save cost for trajectory
                  bool will_collide = p.will_collide(sensor_fusion, combined_fine_trajectory, 0, p.get_lane_for_d(car_d), car_s);
                  double cost = 0;
                  cost = p.estimate_cost_for_trajectory({car_x, car_y, Helper::deg2rad(car_yaw), car_speed}, {car_s, car_d},
                    combined_fine_trajectory, map_waypoints_x, map_waypoints_y, sensor_fusion, verbose()); 
                  all_costs.push_back(cost);

                }
                
              }

              // select trajectory with minimum cost
              double min_cost = 1e20;
              double index = -1;
              bool changed = false;
              for (int i=0; i<all_costs.size(); i++) 
              {
                if (all_costs[i] < min_cost)
                {
                  min_cost = all_costs[i];
                  index = i;
                  changed = true;
                }
              }
              if (!changed)
              {
              	cout << "all bad trajectories - car_s: " << car_s << "cost = " << min_cost << endl;
              }
              vector<vector<double>> selected_fine_trajectory = all_fine_trajectories[index];

              if (verbose())
              {
              	  cout << "---------------" << endl;
	              cout << "index = " << index << endl;
	              Helper::debug_print("all_costs: ", all_costs);
	              Helper::debug_print("selected_fine_trajectory.size(): ", {(double)selected_fine_trajectory.size()});
	              //Helper::debug_print("selected coarse trajectory x_points: ", all_coarse_trajectories[index][0]);
	              //Helper::debug_print("selected coarse trajectory y_points: ", all_coarse_trajectories[index][1]);
	              Helper::debug_print("selected fine trajectory x_points: ", selected_fine_trajectory[0]);
	              Helper::debug_print("selected fine trajectory y_points: ", selected_fine_trajectory[1]);
              }

              if (verbose())
              {
                cout << "counter: " << tmp_cntr << " - car speed: " << car_speed ;
                cout << " - target speed: " << p.target_speed << " - min_cost: ";
                cout << min_cost << " - index: " << index << endl;
              }
              if (tmp_cntr > 2 && tmp_cntr < 2) 
              {
                turn_verbose_on();
              } 
              else
              {
                turn_verbose_off();
              }
              

              // // copy unused points from previous path received from the simulator
              // if (get_counter()==1)
              // {
                  next_x_vals = selected_fine_trajectory[0];
                  next_y_vals = selected_fine_trajectory[1];
              // }
              // else
              // {
              // 	for(int i=0; i<path_size; i++)
	             //  {
	             //    next_x_vals.push_back(previous_path_x[i]);
	             //    next_y_vals.push_back(previous_path_y[i]);
	             //  }
              // }
              // if (next_x_vals.size()==0)
              // {
              // 	return;
              // }
              

            // AK End

            json msgJson;

          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
