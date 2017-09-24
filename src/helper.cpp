#include "helper.h"

Helper::Helper() {}

Helper::~Helper() {}

double Helper::pi() 
{
	return M_PI;
}

double Helper::deg2rad(double x) 
{ 
	return x * pi() / 180; 
}


double Helper::rad2deg(double x) 
{ 
	return x * 180 / pi(); 
}

// Calculate euclidian distance between two points
// x1, y1 are coordinates of the first pointa and x2, y2 are coordinates of the second point
double Helper::distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}


// Returns the closes waypoint to point x, y
// maps_s, and maps_y are waypoints coordinates of all map points.
int Helper::ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = Helper::distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

// Return the next waypoint in the path of the car
// x, y coordinates of the car, theta is the anble of the car, and maps_x, and maps_y are waypoint coordinates
int Helper::NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = Helper::ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > Helper::pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
// x, y are car coordinates, theta is the car angle, maps_x, and maps_y are map points
vector<double> Helper::getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	int next_wp = Helper::NextWaypoint(x,y, theta, maps_x, maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	
	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = Helper::distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = Helper::distance(center_x,center_y,x_x,x_y);
	double centerToRef = Helper::distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += Helper::distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += Helper::distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}


// Transform from Frenet s,d coordinates to Cartesian x,y
// s, d are car coordinates, maps_s, maps_x, maps_y define the map
vector<double> Helper::getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-Helper::pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

// transfer from map to vehicle coordinates
vector<double> Helper::get_vehicle_coords_from_map_coords(double global_x, double global_y, vector<double> car_xyyaw) 
{
  double shift_x = global_x-car_xyyaw[0];
  double shift_y = global_y-car_xyyaw[1];
  double result_x = (shift_x*cos(0-car_xyyaw[2])-shift_y*sin(0-car_xyyaw[2]));
  double result_y = (shift_x*sin(0-car_xyyaw[2])+shift_y*cos(0-car_xyyaw[2]));	

  return {result_x, result_y};
}

// transfer from vehcile to map coordinates
vector<double> Helper::get_map_coords_from_vehicle_coords(double local_x, double local_y, vector<double> car_xyyaw) 
{
  double map_x = (local_x*cos(car_xyyaw[2])-local_y*sin(car_xyyaw[2]))+car_xyyaw[0];
  double map_y = (local_x*sin(car_xyyaw[2])+local_y*cos(car_xyyaw[2]))+car_xyyaw[1];

  return {map_x, map_y};
}


// transfer from sd coordinates to vehicle coordinates
vector<double> Helper::get_vehicle_coords_from_frenet(double s_coord, double d_coord, vector<double> car_xyyaw, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
  vector<double> xy_coords = getXY(s_coord, d_coord, maps_s, maps_x, maps_y);
  vector<double> veh_coords = get_vehicle_coords_from_map_coords(xy_coords[0], xy_coords[1], car_xyyaw);

  return veh_coords;
}

// print to console for debugging purposes
void Helper::debug_print(string title, vector<double> anything)
{
	cout << title;
	for (int i=0; i<anything.size(); i++)
	{
		cout << anything[i] << " ";
	}
	cout << endl;
}

// combine trajectories
// only return size num_points number of points
vector<vector<double>> Helper::combine_trajectories(vector<vector<double>> trajectory_1, vector<vector<double>> trajectory_2, int num_points)
{
	vector<double> result_x;
	vector<double> result_y;

	if (trajectory_1[0].size() >= num_points)
	{
		for (int i=0; i<num_points; i++)
		{
			result_x.push_back(trajectory_1[0][i]);
			result_y.push_back(trajectory_1[1][i]);
		}
	}
	else
	{
		for (int i=0; i<trajectory_1[0].size(); i++)
		{
			result_x.push_back(trajectory_1[0][i]);
			result_y.push_back(trajectory_1[1][i]);
		}

		for (int i=0; i<num_points-trajectory_1[0].size(); i++)
		{
			result_x.push_back(trajectory_2[0][i]);
			result_y.push_back(trajectory_2[1][i]);
		}

	}

	return {result_x, result_y};
}


