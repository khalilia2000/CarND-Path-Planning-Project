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
  p.target_speed = 5;


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

            // AK Begin

          	// automatically adjust number of points in trajectory so that the car is more responsive at lower speeds.
            p.num_points_in_trajectory = (int)(floor(p.target_speed*0.7));

            // Define all required variables
            double end_pos_x = 0; 
            double end_pos_y = 0;
            double ref_yaw = 0;
            double ref_speed = 0;
            double ref_distance = 0;
            double ref_end_s;
            double prev_pos_x = 0;
            double prev_pos_y = 0;
            int path_size = previous_path_x.size();


            // update state and speed
            int end_pos_lane = p.get_lane_for_d(end_path_d);
            bool too_close_ahead = p.is_too_close_ahead(sensor_fusion, end_pos_lane, end_path_s, path_size * p.time_interval_between_points);
            p.update_state(too_close_ahead, end_path_d);
            vector<int> lanes_to_explore = p.possible_lanes_to_explore(end_pos_lane);
            double speed_of_car_ahead = p.speed_of_nearby_car(car_s, end_pos_lane, sensor_fusion, true);
            p.update_target_speed(too_close_ahead, speed_of_car_ahead);

            
            // create  2 points with right heading from the current car location or the end of previous path
            if(path_size < 2)
            {
              end_pos_x = car_x; 
              end_pos_y = car_y;
              ref_yaw = Helper::deg2rad(car_yaw);
              prev_pos_x = end_pos_x - cos(ref_yaw)*1.0;
              prev_pos_y = end_pos_y - sin(ref_yaw)*1.0;
              ref_distance = 1.0;
              ref_speed = car_speed;
              ref_end_s = car_s;
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
              ref_speed = ref_distance / p.time_interval_between_points * p.conversion_factor_mps_to_mph;
              ref_end_s = end_path_s;

            }
            vector<double> end_xyyawspeed = {end_pos_x, end_pos_y, ref_yaw, ref_speed};
            vector<double> prev_xyyawspeed = {prev_pos_x, prev_pos_y, ref_yaw, ref_speed};
            vector<double> car_xyyawspeed = {car_x, car_y, Helper::deg2rad(car_yaw), car_speed};


            // generate trajectories
            auto end_trajectories = p.generate_trajectories(lanes_to_explore, ref_end_s, end_xyyawspeed, prev_xyyawspeed, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<vector<vector<double>>> combined_trajectories;
            for (int i=0; i<end_trajectories.size(); i++)
            {
              auto combined_trajectory = Helper::combine_trajectories({previous_path_x, previous_path_y}, end_trajectories[i], path_size + end_trajectories[i][0].size());  
              combined_trajectories.push_back(combined_trajectory);
            }


            // calcualte costs
            vector<double> all_costs;
            for (int i=0; i<combined_trajectories.size(); i++)
            {
              double cost = p.estimate_cost_for_trajectory(car_xyyawspeed, {car_s, car_d}, combined_trajectories[i], map_waypoints_x, map_waypoints_y, sensor_fusion);
              all_costs.push_back(cost);
            }


            // find the minimum cost
            double min_cost = 1e20;
            int index = -1;
            for (int i=0; i<all_costs.size(); i++)
            {
              if (all_costs[i]<min_cost)
              {
                min_cost = all_costs[i];
                index = i;
              }
            }
            p.target_lane = lanes_to_explore[index];


            // generate output trajectories to be passed on to simulator with correct number of points
            vector<double> next_x_points;
            vector<double> next_y_points;
            for (int i=0; i<p.num_points_in_trajectory; i++)
            {
              next_x_points.push_back(combined_trajectories[index][0][i]);
              next_y_points.push_back(combined_trajectories[index][1][i]);
            }

              
            // AK End

            json msgJson;

          	msgJson["next_x"] = next_x_points;
          	msgJson["next_y"] = next_y_points;

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
