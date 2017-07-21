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


vector<double> get_jerk_minimizing_trajectory_coefficients(
    vector<double> initial_state, vector<double> final_state, double time)
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


double evaluate_polynomial(vector<double> coefficients, double x)
{
    double y = coefficients[0] ;

    for(int index = 1 ; index < coefficients.size() ; ++index)
    {

        y += coefficients[index] * std::pow(x, double(index)) ;
    }

    return y ;
}


vector<double> evaluate_polynomial_over_vector(vector<double> coefficients, vector<double> x_values)
{
    vector<double> y_values ;

    for(int index = 0 ; index < x_values.size() ; ++index)
    {
        double y = evaluate_polynomial(coefficients, x_values[index]) ;
        y_values.push_back(y) ;
    }

    return y_values ;
}


double get_previous_trajectory_final_speed(
    vector<double> previous_trajectory_x, vector<double> previous_trajectory_y, double time_between_steps)
{
    int size = previous_trajectory_x.size() ;

    double last_x = previous_trajectory_x[size - 1] ;
    double last_y = previous_trajectory_y[size - 1] ;

    double second_last_x = previous_trajectory_x[size - 2] ;
    double second_last_y = previous_trajectory_y[size - 2] ;

    double covered_distance = distance(last_x, last_y, second_last_x, second_last_y) ;
    return covered_distance / time_between_steps ;
}


double get_previous_trajectory_final_acceleration(
    vector<double> previous_trajectory_x, vector<double> previous_trajectory_y, double time_between_steps)
{
    int size = previous_trajectory_x.size() ;

    double last_x = previous_trajectory_x[size - 1] ;
    double last_y = previous_trajectory_y[size - 1] ;

    double second_last_x = previous_trajectory_x[size - 2] ;
    double second_last_y = previous_trajectory_y[size - 2] ;

    double third_last_x = previous_trajectory_x[size - 3] ;
    double third_last_y = previous_trajectory_y[size - 3] ;

    double last_to_second_last_distance = distance(last_x, last_y, second_last_x, second_last_y) ;
    double second_last_to_third_last_distance = distance(second_last_x, second_last_y, third_last_x, third_last_y) ;

    double last_velocity = last_to_second_last_distance / time_between_steps ;
    double second_last_velocity = second_last_to_third_last_distance / time_between_steps ;

    double acceleration = (last_velocity - second_last_velocity) / time_between_steps ;
    return acceleration ;
}


double get_cartesian_trajectory_distance(vector<double> trajectory_x, vector<double> trajectory_y)
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


vector<vector<double>> get_jerk_minimizing_trajectory(
    double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed, double car_acceleration,
    vector<double> maps_s, vector<double> maps_x, vector<double> maps_y,
    vector<double> previous_trajectory_x, vector<double> previous_trajectory_y)
{
    // Push current state to previous trajectory if it is empty
    if(previous_trajectory_x.size() < 3)
    {

        // Push back our current position 3 times - this way we can always compute finish velocity and acceleration
        // even if they'll be zeros
        for(int index = 0 ; index < 3 ; ++index)
        {
            previous_trajectory_x.push_back(car_x) ;
            previous_trajectory_y.push_back(car_y) ;
        }
    }

    vector<double> x_trajectory ;
    vector<double> y_trajectory ;

    // Initialize trajectory with previous steps
    for(int index = 0 ; index < previous_trajectory_x.size() ; ++index)
    {
        x_trajectory.push_back(previous_trajectory_x[index]) ;
        y_trajectory.push_back(previous_trajectory_y[index]) ;
    }

    double steps_per_second = 50 ;
    double time_between_steps = 1.0 / steps_per_second ;

    double car_yaw_in_rad = deg2rad(car_yaw) ;

    // Get initial state as last state of previous path
    // car_yaw might not be correct, but should be close to true value
    auto frenet_sd = getFrenet(previous_trajectory_x.back(), previous_trajectory_y.back(), car_yaw_in_rad, maps_x, maps_y) ;

    double initial_s = frenet_sd[0] ;

    double initial_car_speed = get_previous_trajectory_final_speed(
        previous_trajectory_x, previous_trajectory_y, time_between_steps) ;

    double initial_car_acceleration = get_previous_trajectory_final_acceleration(
        previous_trajectory_x, previous_trajectory_y, time_between_steps) ;

    vector<double> initial_s_state {initial_s, initial_car_speed, initial_car_acceleration} ;


    // Time span
    double time_delta = 4.0 ;
    double ideal_target_speed = 20 ;

    double target_acceleration = (ideal_target_speed - initial_car_speed) / time_delta ;

    // If acceleration is over 1m/s, limit it
    while (std::abs(target_acceleration) > 1.0)
    {
        target_acceleration *= 0.9 ;
    }


    double max_jerk = 5.0 ;
    // If jerk would be too large, limit it
    while(std::abs(target_acceleration - initial_car_acceleration) / time_delta > max_jerk)
    {
        target_acceleration = 0.5 * (target_acceleration + initial_car_acceleration) ;
    }

    // Now compute position and velocity of final state
    double target_position =
        initial_s + (initial_car_speed * time_delta) +
        (0.25 * (initial_car_acceleration + target_acceleration) * time_delta * time_delta) ;

    double target_speed = initial_car_speed + (0.5 * (initial_car_acceleration + target_acceleration) * time_delta) ;

    vector<double> final_s_state {target_position, target_speed, target_acceleration} ;

    // Initial d speed and acceleration are probably not quite correct
    vector<double> initial_d_state {frenet_sd[1], 0.0, 0.0} ;

    double target_d = 6.0 ;
    vector<double> final_d_state = {target_d, 0.0, 0.0} ;

    auto s_coefficients = get_jerk_minimizing_trajectory_coefficients(
        initial_s_state, final_s_state, time_delta) ;

    auto d_coefficients = get_jerk_minimizing_trajectory_coefficients(
        initial_d_state, final_d_state, time_delta) ;

    vector<double> time_steps ;
    double time_instant = 0 ;

    while(time_instant <= time_delta)
    {
        time_steps.push_back(time_instant) ;
        time_instant += 1.0 / steps_per_second ;
    }

    auto added_s_trajectory = evaluate_polynomial_over_vector(s_coefficients, time_steps) ;
    auto added_d_trajectory = evaluate_polynomial_over_vector(d_coefficients, time_steps) ;

    auto added_xy_trajectory = convert_frenet_trajectory_to_cartesian_trajectory(
        added_s_trajectory, added_d_trajectory, maps_s, maps_x, maps_y) ;


    // Add new generated steps
    for(int index = 0 ; index < added_s_trajectory.size() ; ++index)
    {
        x_trajectory.push_back(added_xy_trajectory[0][index]) ;
        y_trajectory.push_back(added_xy_trajectory[1][index]) ;
    }

    vector<vector<double>> xy_trajectory {x_trajectory, y_trajectory} ;


    return xy_trajectory ;

}




#endif //PATH_PLANNING_PROCESSING_H
