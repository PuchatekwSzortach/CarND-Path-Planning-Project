//
// Created by Kolodziejczyk Jakub on 2017/07/20.
//

#ifndef PATH_PLANNING_PROCESSING_H
#define PATH_PLANNING_PROCESSING_H

#include <vector>
#include <cmath>
#include <algorithm>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "spline.h"

using namespace std ;


// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }


double deg2rad(double x) { return x * pi() / 180.0; }


double rad2deg(double x) { return x * 180.0 / pi(); }


double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}


int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y) {

  double closestLen = 100000.0; //large number
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

  double center_x = 1000.0 - maps_x[prev_wp];
  double center_y = 2000.0 - maps_y[prev_wp];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0.0;
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


vector<vector<double>> convert_frenet_trajectory_to_cartesian_trajectory(
  vector<double> &s_trajectory, vector<double> &d_trajectory,
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


vector<double> get_jerk_minimizing_trajectory_coefficients(
    vector<double> &initial_state, vector<double> &final_state, double time)
{

    Eigen::MatrixXd time_matrix = Eigen::MatrixXd(3, 3);

	time_matrix <<
	    std::pow(time, 3.0), std::pow(time, 4.0), std::pow(time, 5.0),
        3.0 * std::pow(time, 2.0), 4.0 * std::pow(time, 3.0), 5.0 * std::pow(time, 4.0),
        6.0 * time, 12.0 * std::pow(time, 2.0), 20.0 * std::pow(time, 3.0) ;

	Eigen::MatrixXd boundary_conditions = Eigen::MatrixXd(3,1);

	boundary_conditions <<
	    final_state[0] - (initial_state[0] + (initial_state[1] * time) + (0.5 * initial_state[2] * std::pow(time, 2.0))),
        final_state[1] - (initial_state[1] + (initial_state[2] * time)),
        final_state[2] - initial_state[2];

    Eigen::MatrixXd coefficients_matrix = time_matrix.inverse() * boundary_conditions;

    vector<double> coefficients = {initial_state[0], initial_state[1], 0.5 * initial_state[2]} ;

    for(int index = 0; index < coefficients_matrix.size(); index++)
	{
	    coefficients.push_back(coefficients_matrix.data()[index]);
	}

    return coefficients ;
}


double evaluate_polynomial(vector<double> &coefficients, double x)
{
    double y = coefficients[0] ;

    for(int index = 1 ; index < coefficients.size() ; ++index)
    {

        y += coefficients[index] * std::pow(x, double(index)) ;
    }

    return y ;
}


vector<double> evaluate_polynomial_over_vector(vector<double> &coefficients, vector<double> &x_values)
{
    vector<double> y_values ;

    for(int index = 0 ; index < x_values.size() ; ++index)
    {
        double y = evaluate_polynomial(coefficients, x_values[index]) ;
        y_values.push_back(y) ;
    }

    return y_values ;
}


double get_last_s_speed(vector<double> &s_trajectory, double time_interval)
{
    int size = s_trajectory.size() ;
    return (s_trajectory[size - 1] - s_trajectory[size - 2]) / time_interval ;
}


double get_last_s_acceleration(vector<double> &s_trajectory, double time_interval)
{
    int size = s_trajectory.size() ;

    double last_speed = (s_trajectory[size - 1] - s_trajectory[size - 2]) / time_interval ;
    double second_last_speed = (s_trajectory[size - 2] - s_trajectory[size - 3]) / time_interval ;

    return (last_speed - second_last_speed) / time_interval ;
}


double get_cartesian_trajectory_distance(vector<double> &trajectory_x, vector<double> &trajectory_y)
{
    double cumulative_distance = 0 ;

    if(trajectory_x.size() > 1)
    {
        for(int index = 1 ; index < trajectory_x.size() ; ++index)
        {
            cumulative_distance += distance(
                trajectory_x[index], trajectory_y[index], trajectory_x[index - 1], trajectory_y[index - 1]) ;
        }
    }

    return cumulative_distance ;

}


vector<double> get_smoothed_trajectory(vector<double> &time_steps, vector<double> &trajectory)
{
    double size = double(trajectory.size()) ;

    if(size < 5)
    {
        return trajectory ;
    }

    // Select a few indices to use for spline interpolation
    vector<int> indices = {0, int(0.25 * size), int(0.5 * size), int(0.75 * size), int(size) - 1} ;

    vector<double> time_args ;
    vector<double> trajectory_args ;

    for(int index: indices)
    {
        time_args.push_back(time_steps[index]) ;
        trajectory_args.push_back(trajectory[index]) ;
    }

    tk::spline spline;
    spline.set_points(time_args, trajectory_args);

    vector<double> smooth_trajectory ;

    for(int index = 0 ; index < time_steps.size() ; ++index)
    {
        double smooth_value = spline(time_steps[index]) ;
        smooth_trajectory.push_back(smooth_value) ;
    }

    return smooth_trajectory ;
}


vector<double> get_initial_s_state(vector<double> &s_trajectory, double time_between_steps)
{
    double initial_s = s_trajectory.back() ;
    double initial_speed = get_last_s_speed(s_trajectory, time_between_steps) ;
    double initial_acceleration = get_last_s_acceleration(s_trajectory, time_between_steps) ;

    vector<double> initial_s_state {initial_s, initial_speed, initial_acceleration} ;
    return initial_s_state ;
}

vector<double> get_initial_d_state(vector<double> &previous_d_trajectory, double time_between_steps)
{
    int size = previous_d_trajectory.size() ;

    double last_d = previous_d_trajectory[size - 1] ;
    double second_last_d = previous_d_trajectory[size - 2] ;
    double third_last_d = previous_d_trajectory[size - 3] ;

    double last_d_speed = (last_d - second_last_d) / time_between_steps ;
    double second_last_d_speed = (second_last_d - third_last_d) / time_between_steps ;

    double last_d_acceleration = (last_d_speed - second_last_d_speed) / time_between_steps ;

    vector<double> initial_d_state {last_d, last_d_speed, last_d_acceleration} ;
    return initial_d_state ;
}


vector<double> get_final_s_state(vector<double> &s_trajectory, double time_horizon, double time_between_steps)
{
    auto initial_s_state = get_initial_s_state(s_trajectory, time_between_steps) ;

    double initial_s = initial_s_state[0] ;
    double initial_speed = initial_s_state[1] ;
    double initial_acceleration = initial_s_state[2] ;

    double ideal_target_speed = 20 ;
    double target_acceleration = (ideal_target_speed - initial_speed) / time_horizon ;

//    std::cout << "Previous trajectory acceleration was " << initial_acceleration << std::endl ;
//    std::cout << "Initial target acceleration: " << target_acceleration << std::endl ;

    double max_acceleration = 1.0 ;
    // If acceleration is too large, limit it
    while (std::abs(target_acceleration) > max_acceleration)
    {
        target_acceleration *= 0.9 ;
    }

//    std::cout << "After max acceleration check: " << target_acceleration << std::endl ;

    double max_jerk = 2.0 ;
    // If jerk would be too large, limit it
    while(std::abs(target_acceleration - initial_acceleration) / time_horizon > max_jerk)
    {
        target_acceleration = (0.8 * target_acceleration) + (0.2 * initial_acceleration) ;
    }

//    std::cout << "After max jerk check: " << target_acceleration << std::endl ;

    // Now compute position and velocity of final state
    double target_position =
        initial_s + (initial_speed * time_horizon) +
        (0.25 * (initial_acceleration + target_acceleration) * time_horizon * time_horizon) ;

    double target_speed = initial_speed + (0.5 * (initial_acceleration + target_acceleration) * time_horizon) ;

    vector<double> final_s_state {target_position, target_speed, target_acceleration} ;

    return final_s_state ;
}

vector<double> get_final_d_state(
    vector<double> &previous_d_trajectory, double ideal_position, double time_horizon, double time_between_steps)
{
    auto initial_d_state = get_initial_d_state(previous_d_trajectory, time_between_steps) ;

    double initial_position = initial_d_state[0] ;
    double initial_speed = initial_d_state[1] ;
    double initial_acceleration = initial_d_state[2] ;

    // Compute final state assuming input from initial state only
    double final_position_based_on_initial_state =
        initial_position + (initial_speed * time_horizon) *
        (0.5 * initial_acceleration * time_horizon * time_horizon) ;

    double final_speed_based_on_initial_state = initial_speed + (initial_acceleration * time_horizon) ;

    double position_difference = ideal_position - final_position_based_on_initial_state ;

    double final_acceleration = 0 ;

    // We should increase d
    if(position_difference > 0)
    {
        // We are already going in right direction
        if(final_speed_based_on_initial_state > 0)
        {
            final_acceleration = 0 ;
        }
        else
        {
            final_acceleration = -1 ;
        }

    }
    else // We should decrease d
    {
        // We need to go in opposite direction
        if(final_speed_based_on_initial_state > 0)
        {
            final_acceleration = -1 ;
        }
        else // continue
        {
            final_acceleration = 0 ;
        }
    }

    double max_acceleration = 0.5 ;
    // If acceleration is too large, limit it
    while (std::abs(final_acceleration) > max_acceleration)
    {
        final_acceleration *= 0.9 ;
    }

    double max_jerk = 1.0 ;
    // If jerk would be too large, limit it
    while(std::abs(final_acceleration - initial_acceleration) / time_horizon > max_jerk)
    {
        final_acceleration = (0.8 * final_acceleration) + (0.2 * initial_acceleration) ;
    }

    // Compute actual position and speed we can reach
    double final_position =
        initial_position + (initial_speed * time_horizon) +
        (0.25 * (initial_acceleration + final_acceleration) * time_horizon * time_horizon) ;

    double final_speed = initial_speed + (0.5 * (initial_acceleration + final_acceleration) * time_horizon) ;

    vector<double> final_state {final_position, final_speed, final_acceleration} ;
    return final_state ;
}


void print_trajectory(vector<double> &trajectory)
{
    std::cout << "\n\n[" ;

    for(auto value: trajectory)
    {
        std::cout << value << ", " ;
    }

    std::cout << "]," << std::endl ;
}

#endif //PATH_PLANNING_PROCESSING_H
