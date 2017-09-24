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
double verbose() { return true; }

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

  // Ali K defining target_speed
  double target_speed = 0.0;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&target_speed](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    
    // Ali Khalili defining target_lane and target_speed
    int target_lane = 1;
    bool too_close_ahead = false;
    Planner p;

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

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;


          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
            
            // AK Begin

	          // Define all variables
	            vector<double> ptsx;
	            vector<double> ptsy;
	            double end_pos_x; 
              double end_pos_y;
              double ref_yaw;
              double prev_pos_x;
              double prev_pos_y;
              int path_size = previous_path_x.size();


              // check other vehicles
              too_close_ahead = false;
              double max_safe_speed = p.max_speed;
              for (int i=0; i<sensor_fusion.size(); i++)
              {
                double check_d = sensor_fusion[i][6];
                if ((check_d>4*target_lane) && (check_d<4*(target_lane+1)))
                {
                  double check_vx = sensor_fusion[i][3];
                  double check_vy = sensor_fusion[i][4];
                  double check_v = sqrt(check_vx*check_vx + check_vy*check_vy);
                  double check_s = sensor_fusion[i][5];

                  check_s += ((double)path_size * p.time_interval_between_points * check_v);

                  if ((check_s>car_s) && (check_s-car_s<p.safe_distance_from_other_cars))
                  {
                    if (verbose())
                    {
                      cout << "Car ID: " << sensor_fusion[i][0] << " - Distance: " << check_s-car_s;
                      cout << " - front_car_speed: " << check_v;
                      cout << " - target_speed: " << target_speed << endl;
                    }
                    too_close_ahead = true;
                    if (check_v < max_safe_speed) {max_safe_speed = check_v;}
                  }
                }
              }


              // adjust speed if close to vehicle
              if (target_speed > max_safe_speed)
              {
                target_speed -= 0.224;
              } 
              else if (target_speed < max_safe_speed)
              {
                target_speed += 0.224;
              }


              // create initial 2 points with right heading from the current car location
              // or the end of previous path
              if(path_size < 2)
              {
                end_pos_x = car_x; 
                end_pos_y = car_y;
                ref_yaw = Helper::deg2rad(car_yaw);
                prev_pos_x = end_pos_x - cos(car_yaw)*1.0;
                prev_pos_y = end_pos_y - sin(car_yaw)*1.0;
                if (verbose())
                {
                  cout << endl << "Starting with loop1" << endl;
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
                if (verbose())
                {
                  cout << endl << "Starting with loop2" << endl;
                  Helper::debug_print("car_yaw, ref_yaw: ", {car_yaw, ref_yaw});
                }
              }

              vector<double> end_frenet = Helper::getFrenet(end_pos_x, end_pos_y, ref_yaw, map_waypoints_x, map_waypoints_y);
              double end_pos_s = end_frenet[0];
              double end_pos_d = end_frenet[1];

              // add 2 points to the x array
              ptsx.push_back(prev_pos_x);
              ptsx.push_back(end_pos_x);
              // add 2 points to the y array
              ptsy.push_back(prev_pos_y);
              ptsy.push_back(end_pos_y); 

              // Add evently 30 m spaced points to the end in Frenet coordinates
              vector<double> wp0 = Helper::getXY(end_pos_s+2, 2+4*target_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              vector<double> wp1 = Helper::getXY(end_pos_s+4, 2+4*target_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              vector<double> wp2 = Helper::getXY(end_pos_s+8, 2+4*target_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              // add to ptsx
              ptsx.push_back(wp0[0]);
              ptsx.push_back(wp1[0]);
              ptsx.push_back(wp2[0]);
              // add to ptsy
              ptsy.push_back(wp0[1]);
              ptsy.push_back(wp1[1]);
              ptsy.push_back(wp2[1]);

              // debug
              if (verbose()) 
              {
	              Helper::debug_print("ptsx in map coordinates: ", ptsx);
	              Helper::debug_print("ptsy in map coordinates: ", ptsy);
          	  }


              // shift to car reference coordinates
              for (int i = 0; i<ptsx.size(); i++)
              {
              	vector<double> vehicle_coords = Helper::get_vehicle_coords_from_map_coords(ptsx[i], ptsy[i], {end_pos_x, end_pos_y, ref_yaw});
                ptsx[i] = vehicle_coords[0];
                ptsy[i] = vehicle_coords[1];
              }

              // *****
              // Debug
              // *****
              if (verbose())
              {
              	Helper::debug_print("car_x,y,s: ", {car_x, car_y, car_s});
                Helper::debug_print("previous path size: ", {(double)path_size});
                Helper::debug_print("previous_path_x: ", previous_path_x);
                Helper::debug_print("previous_path_y: ", previous_path_y);
                Helper::debug_print("prev_pos: ", {prev_pos_x, prev_pos_y});
                Helper::debug_print("end_pos: ", {end_pos_x, end_pos_y});
                Helper::debug_print("ptsx: ", ptsx);
                Helper::debug_print("ptsy: ", ptsy);
              }
              // end Debug


              vector<vector<double>> fine_trajectory = p.generate_fine_from_coarse_trajectory(ptsx, ptsy, {end_pos_x, end_pos_y, ref_yaw});

              // debug
              if (verbose()) 
              {
	              Helper::debug_print("fine_trajectory_x: ", fine_trajectory[0]);
	              Helper::debug_print("fine_trajectory_y: ", fine_trajectory[1]);
          	  }

              // copy unused points from previous path received from the simulator
              for(int i=0; i<path_size; i++)
              {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
              }

              // add new points at the end to bring the number to 50
              for (int i=0; i<p.num_points_in_trajectory-path_size; i++)
              {
              	next_x_vals.push_back(fine_trajectory[0][i]);
              	next_y_vals.push_back(fine_trajectory[1][i]);
              }


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
















































































