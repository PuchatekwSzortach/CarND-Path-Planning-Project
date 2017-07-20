//
// Created by Kolodziejczyk Jakub on 2017/07/20.
//

#ifndef PATH_PLANNING_PROCESSING_H
#define PATH_PLANNING_PROCESSING_H

#include <vector>
#include <cmath>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"

using namespace std ;


// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }


double deg2rad(double x) { return x * pi() / 180; }


double rad2deg(double x) { return x * 180 / pi(); }


double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}


int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y) {

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); i++) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x, y, map_x, map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }

  }

  return closestWaypoint;

}


int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y) {

  int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y - y), (map_x - x));

  double angle = abs(theta - heading);

  if (angle > pi() / 4) {
    closestWaypoint++;
  }

  return closestWaypoint;

}


// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y) {
  int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0) {
    prev_wp = maps_x.size() - 1;
  }

  double n_x = maps_x[next_wp] - maps_x[prev_wp];
  double n_y = maps_y[next_wp] - maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - maps_x[prev_wp];
  double center_y = 2000 - maps_y[prev_wp];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; i++) {
    frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};

}


// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp + 1] && (prev_wp < (int) (maps_s.size() - 1))) {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % maps_x.size();

  double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - pi() / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x, y};

}


vector<vector<double>> get_trajectory(
  double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed,
  vector<double> maps_x, vector<double> maps_y)
{
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  // Get index of closest point
  int waypoint_index = NextWaypoint(car_x, car_y, car_yaw, maps_x, maps_y) ;

  // Interpolate from current position to waypoint in 100 steps
  double interpolation_steps_count = 100 ;

  for(double index = 0 ; index < interpolation_steps_count ; ++index)
  {
    double x = car_x + (index * (maps_x[waypoint_index] - car_x) / interpolation_steps_count) ;
    double y = car_y + (index * (maps_y[waypoint_index] - car_y) / interpolation_steps_count) ;

    next_x_vals.push_back(x) ;
    next_y_vals.push_back(y) ;
  }

  vector<vector<double>> trajectory {next_x_vals, next_y_vals} ;
  return trajectory ;
}

vector<vector<double>> convert_frenet_trajectory_to_cartesian_trajectory(
  vector<double> s_trajectory, vector<double> d_trajectory,
  vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
  vector<double> x_trajectory ;
  vector<double> y_trajectory ;

  for(int index = 0 ; index < s_trajectory.size() ; ++index)
  {
    auto xy = getXY(s_trajectory[index], d_trajectory[index], maps_s, maps_x, maps_y) ;

    x_trajectory.push_back(xy[0]) ;
    y_trajectory.push_back(xy[1]) ;
  }

  vector<vector<double>> xy_trajectory {x_trajectory, y_trajectory} ;
  return xy_trajectory ;

};

vector<vector<double>> get_lane_keeping_trajectory(
    double car_s, double car_d, double car_speed,
    vector<double> maps_s, vector<double> maps_x, vector<double> maps_y) {

    // For now just try to travel at 20 metres per second
    double target_s = car_s + 20;

    // We want to be in the middle of the second lane
    double target_d = 6;

    vector<double> s_trajectory;
    vector<double> d_trajectory;

    double s_delta = target_s - car_s ;
    double d_delta = target_d - car_d ;

    double steps = 50 ;

    for (double index = 0; index < steps ; ++index)
    {
    double s = car_s + (index * s_delta / steps) ;
    double d = car_d + (index * d_delta / steps) ;

    s_trajectory.push_back(s) ;
    d_trajectory.push_back(d) ;
    }

    auto xy_trajectory = convert_frenet_trajectory_to_cartesian_trajectory(
        s_trajectory, d_trajectory, maps_s, maps_x, maps_y) ;

    return xy_trajectory ;

}


vector<double> get_jerk_minimizing_trajectory_coefficients(
    vector<double> initial_state, vector<double> final_state, double time)
{

    Eigen::MatrixXd time_matrix = Eigen::MatrixXd(3, 3);

    double time_square = time * time ;
    double time_cubic = time_square * time ;

	time_matrix <<
	    time_cubic, time_cubic * time, time_cubic * time_square,
        3.0 * time_square, 4.0 * time_cubic, 5.0 * time_cubic * time,
        6.0 * time, 12.0 * time_square, 20.0 * time_cubic ;

	Eigen::MatrixXd boundary_conditions = Eigen::MatrixXd(3,1);

	boundary_conditions <<
	    final_state[0] - (initial_state[0] + (initial_state[1] * time) + (0.5 * initial_state[2] * time_square)),
        final_state[1] - (initial_state[1] + (initial_state[2] * time)),
        final_state[2] - initial_state[2];

    Eigen::MatrixXd coefficients_matrix = time_matrix.inverse() * boundary_conditions;

    vector<double> coefficients = {initial_state[0], initial_state[1], initial_state[2]} ;

    for(int index = 0; index < coefficients_matrix.size(); index++)
	{
	    coefficients.push_back(coefficients_matrix.data()[index]);
	}

    return coefficients ;
}

#endif //PATH_PLANNING_PROCESSING_H
