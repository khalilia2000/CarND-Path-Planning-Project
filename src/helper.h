#ifndef HELER_H_
#define HELER_H_

#include <math.h>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include <iostream>

using namespace std;

class Helper {
public:

  /**
  * Constructor.
  */
  Helper();

  /**
  * Destructor.
  */
  virtual ~Helper();

  // For converting back and forth between radians and degrees.
  static double pi(); 
  static double deg2rad(double x); 
  static double rad2deg(double x); 

  // Calculate euclidian distance between two points
  static double distance(double x1, double y1, double x2, double y2);

  // Returns the closes waypoint to point x, y
  static int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y);

  // Return the next waypoint in the path of the car
  static int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);

  // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
  static vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);

  // Transform from Frenet s,d coordinates to Cartesian x,y
  static vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);

  // transfer from map to vehicle coordinates
  static vector<double> get_vehicle_coords_from_map_coords(double global_x, double global_y, vector<double> car_xyyaw);

  // transfer from vehcile to map coordinates
  static vector<double> get_map_coords_from_vehicle_coords(double local_x, double local_y, vector<double> car_xyyaw);

  // transfer from sd coordinates to vehicle coordinates
  static vector<double> get_vehicle_coords_from_frenet(double s_coord, double d_coord, vector<double> car_xyyaw, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);

  // print to console for debugging purposes
  static void debug_print(string title, vector<double> anything);

};

#endif /* HELER_H_ */