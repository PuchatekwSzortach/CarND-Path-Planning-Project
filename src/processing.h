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


int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y) {

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


int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y) {

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
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y) {
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
vector<double> getXY(
    double s, double d, const vector<double> &maps_s,
    const vector<double> &maps_x, const vector<double> &maps_y) {

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
  vector<double> &maps_s, vector<double> &maps_x, vector<double> &maps_y)
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


double get_first_s_speed(vector<double> &s_trajectory, double time_interval)
{
    return (s_trajectory[1] - s_trajectory[0]) / time_interval ;
}


double get_last_s_speed(vector<double> &s_trajectory, double time_interval)
{
    int size = s_trajectory.size() ;
    return (s_trajectory[size - 1] - s_trajectory[size - 2]) / time_interval ;
}


double get_first_s_acceleration(vector<double> &s_trajectory, double time_interval)
{

    double first_speed = (s_trajectory[1] - s_trajectory[0]) / time_interval ;
    double second_speed = (s_trajectory[2] - s_trajectory[1]) / time_interval ;

    return (second_speed - first_speed) / time_interval ;
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

    int min_size = 5 ;
    if(size < int(min_size))
    {
        return trajectory ;
    }

    // Select a few indices to use for spline interpolation
//    vector<int> indices = {0, int(0.25 * size), int(0.5 * size), int(0.75 * size), int(size) - 1} ;
    vector<int> indices = {0, int(0.25 * size), int(0.75 * size), int(size) - 1} ;

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


vector<double> get_s_state_at_trajectory_start(vector<double> &s_trajectory, double time_between_steps)
{
    double s = s_trajectory.front() ;
    double speed = get_first_s_speed(s_trajectory, time_between_steps) ;
    double acceleration = get_first_s_acceleration(s_trajectory, time_between_steps) ;

    return vector<double> {s, speed, acceleration} ;
}



vector<double> get_s_state_at_trajectory_end(vector<double> &s_trajectory, double time_between_steps)
{
    double s = s_trajectory.back() ;
    double speed = get_last_s_speed(s_trajectory, time_between_steps) ;
    double acceleration = get_last_s_acceleration(s_trajectory, time_between_steps) ;

    return vector<double> {s, speed, acceleration} ;
}


vector<double> get_d_state_at_trajectory_start(vector<double> &d_trajectory, double time_between_steps)
{

    double d = d_trajectory[0] ;
    double second_d = d_trajectory[1] ;
    double third_d = d_trajectory[2] ;

    double speed = (second_d - d) / time_between_steps ;
    double second_speed = (third_d - second_d) / time_between_steps ;

    double acceleration = (second_speed - speed) / time_between_steps ;

    return vector<double> {d, speed, acceleration} ;
}


vector<double> get_d_state_at_trajectory_end(vector<double> &previous_d_trajectory, double time_between_steps)
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


vector<double> get_final_s_state(
    vector<double> &initial_s_state, double target_speed, double time_horizon, double time_between_steps)
{
    double initial_s = initial_s_state[0] ;
    double initial_speed = initial_s_state[1] ;
    double initial_acceleration = initial_s_state[2] ;

    double acceleration = (target_speed - initial_speed) / time_horizon ;

    double max_acceleration = 20.0 ;
    // If acceleration is too large, limit it
    while (std::abs(acceleration) > max_acceleration)
    {
        acceleration *= 0.9 ;
    }

    double max_jerk = 20.0 ;
    // If jerk would be too large, limit it
    while(std::abs(acceleration - initial_acceleration) / time_horizon > max_jerk)
    {
        acceleration = (0.9 * acceleration) + (0.1 * initial_acceleration) ;
    }

    // Now compute position and velocity of final state
    double position =
        initial_s + (initial_speed * time_horizon) +
        (0.25 * (initial_acceleration + acceleration) * time_horizon * time_horizon) ;

    double final_speed = initial_speed + (0.5 * (initial_acceleration + acceleration) * time_horizon) ;

    vector<double> final_state {position, final_speed, acceleration} ;
    return final_state ;
}


vector<double> get_final_d_state(
    vector<double> &initial_d_state, double ideal_position, double time_horizon, double time_between_steps)
{
//    std::cout << "Getting from d " << initial_d_state[0] << " to " << ideal_position << std::endl ;

    double initial_position = initial_d_state[0] ;
    double initial_speed = initial_d_state[1] ;
    double initial_acceleration = initial_d_state[2] ;

    // Compute acceleration we need to achieve ideal position
    double final_acceleration = 4.0 / (time_horizon * time_horizon) *
        (ideal_position - initial_position - (initial_speed * time_horizon) -
            (0.25 * initial_acceleration * time_horizon * time_horizon)) ;

//    std::cout << "Initial final_acceleration " << final_acceleration << std::endl ;

    double max_acceleration = 5.0 ;
    // If acceleration is too large, limit it
    while (std::abs(final_acceleration) > max_acceleration)
    {
        final_acceleration *= 0.8 ;
    }

//    std::cout << "After max acceleration check: " << final_acceleration << std::endl ;

    double max_jerk = 5.0 ;
    // If jerk would be too large, limit it
    while(std::abs(final_acceleration - initial_acceleration) / time_horizon > max_jerk)
    {
        final_acceleration = (0.8 * final_acceleration) + (0.2 * initial_acceleration) ;
    }

//    std::cout << "After max_jerk check: " << final_acceleration << std::endl ;

    // Compute actual position and speed we can reach
    double final_position =
        initial_position + (initial_speed * time_horizon) +
        (0.25 * (initial_acceleration + final_acceleration) * time_horizon * time_horizon) ;

    double final_speed = initial_speed + (0.5 * (initial_acceleration + final_acceleration) * time_horizon) ;

    vector<double> final_state {final_position, final_speed, final_acceleration} ;
    return final_state ;
}


// Get dx dy (normal d vector) for a given xy - interpolates between known map points
vector<double> get_dx_dy(
    double x, double y, vector<double> &maps_x, vector<double> &maps_y,
    vector<double> &maps_dx, vector<double> &maps_dy)
{
    int closest_index = ClosestWaypoint(x, y, maps_x, maps_y) ;

    double closest_point_distance =  distance(x, y, maps_x[closest_index], maps_y[closest_index]) ;
    double previous_point_distance =  distance(x, y, maps_x[closest_index - 1], maps_y[closest_index - 1]) ;
    double next_point_distance =  distance(x, y, maps_x[closest_index + 1], maps_y[closest_index + 1]) ;

    int second_closest_index =
        (previous_point_distance < next_point_distance) ? closest_index - 1 : closest_index + 1 ;

    double second_closest_point_distance =
        (previous_point_distance < next_point_distance) ? previous_point_distance : next_point_distance ;

    double distances_sum = closest_point_distance + second_closest_point_distance ;

    double closest_dx = maps_dx[closest_index] ;
    double closest_dy = maps_dy[closest_index] ;

    double second_closest_dx = maps_dx[second_closest_index] ;
    double second_closest_dy = maps_dy[second_closest_index] ;

    // Do linear interpolation for dx and dy based on neighboring points
    double dx = ((closest_dx * second_closest_point_distance) + (second_closest_dx * closest_point_distance)) / distances_sum ;
    double dy = ((closest_dy * second_closest_point_distance) + (second_closest_dy * closest_point_distance)) / distances_sum ;

    return vector<double> {dx, dy} ;
}


vector<vector<double>> get_xy_states_from_sd_states(
    vector<double> &s_state, vector<double> &d_state,
    vector<double> &maps_s, vector<double> &maps_x, vector<double> &maps_y,
    vector<double> &maps_dx, vector<double> &maps_dy)
{
    auto xy = getXY(s_state[0], d_state[0], maps_s, maps_x, maps_y) ;

    auto dx_dy = get_dx_dy(xy[0], xy[1], maps_x, maps_y, maps_dx, maps_dy) ;
    double dx = dx_dy[0] ;
    double dy = dx_dy[1] ;

    double d_angle = std::atan2(dy, dx) ;
    double s_angle = d_angle + (pi() / 2.0) ;

    double x_speed = (s_state[1] * std::cos(s_angle)) + (d_state[1] * std::cos(d_angle)) ;
    double x_acceleration = (s_state[2] * std::cos(s_angle)) + (d_state[2] * std::cos(d_angle)) ;

    double y_speed = (s_state[1] * std::sin(s_angle)) + (d_state[1] * std::sin(d_angle)) ;
    double y_acceleration = (s_state[2] * std::sin(s_angle)) + (d_state[2] * std::sin(d_angle)) ;

    vector<double> x_state {xy[0], x_speed, x_acceleration} ;
    vector<double> y_state {xy[1], y_speed, y_acceleration} ;

    vector<vector<double>> xy_states {x_state, y_state} ;
    return xy_states ;
}


vector<double> get_sd_velocity_from_xy_velocity(
    double x, double y, double vx, double vy,
    vector<double> &maps_x, vector<double> &maps_y, vector<double> &maps_dx, vector<double> &maps_dy)
{
//    auto dx_dy = get_dx_dy(x, y, maps_x, maps_y, maps_dx, maps_dy) ;
//    double dx = dx_dy[0] ;
//    double dy = dx_dy[1] ;
//
//    double d_angle = std::atan2(dy, dx) ;
//    double s_angle = d_angle + (pi() / 2.0) ;
//
//    double s_velocity = vx * std::cos(s_angle) + (vy * std::sin(s_angle)) ;
//    double d_velocity = vx * std::cos(d_angle) + (vy * std::sin(d_angle)) ;

//    double a = std::cos(s_angle) ;
//    double b = std::cos(d_angle) ;
//    double c = std::sin(s_angle) ;
//    double d = std::sin(s_angle) ;
//
//    double scaling = a - (d * a / b) ;
//    double s_velocity = (vy - (d * vx / b)) / scaling ;
//    double d_velocity = (vx - (a * s_velocity)) / b ;

//    double s_velocity =

    double s_velocity = std::sqrt((vx*vx) + (vy*vy)) ;
    double d_velocity = 0 ;

    vector<double> sd_velocity {s_velocity, d_velocity} ;
    return sd_velocity ;
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


int get_index_of_closest_previous_x_trajectory_point(
    double car_x, double car_y, vector<double> &x_trajectory, vector<double> &y_trajectory)
{
    int best_index = 0 ;
    double best_distance = distance(car_x, car_y, x_trajectory[best_index], y_trajectory[best_index]) ;

    for(int index = 1 ; index < x_trajectory.size() ; ++index)
    {
        double current_distance = distance(
            car_x, car_y, x_trajectory[index], y_trajectory[index]) ;

        if(current_distance < best_distance)
        {
            best_distance = current_distance ;
            best_index = index ;
        }

    }

    return best_index ;
}


// Moves n elements from end of first vector to beginning of second vector
void move_n_elements(vector<double> &first, vector<double> &second, int n)
{
    for(int index = 0 ; index < n ; ++index)
    {
        auto element = first.back() ;
        first.pop_back() ;
        second.insert(second.begin(), element) ;
    }
}


void move_n_elements_from_end_of_first_to_beginning_of_second(vector<double> &first, vector<double> &second, int n)
{
    second.insert(second.begin(), first.end() - n, first.end()) ;
    first.erase(first.end() - n, first.end()) ;
}

void move_n_elements_from_beginning_of_first_to_end_of_second(vector<double> &first, vector<double> &second, int n)
{
    second.insert(second.end(), first.begin(), first.begin() + n) ;
    first.erase(first.begin(), first.begin() + n) ;
}


int get_arg_min(vector<double> &values)
{
    int best_index = 0 ;
    double best_value = values[best_index] ;

    for(int index = 1 ; index < values.size() ; ++index)
    {
        if(values[index] < best_value)
        {
            best_value = values[index] ;
            best_index = index ;
        }
    }

    return best_index ;
}


bool are_ego_and_vehicle_in_same_lane(double ego_d, double vehicle_d)
{
    double half_ego_width = 1.5 ;

    double ego_left = ego_d - half_ego_width ;
    double ego_right = ego_d + half_ego_width ;

    // Both in left lane
    if(vehicle_d < 4.0 && ego_left < 4.0)
    {
        return true ;
    }

    // Both in middle lane
    if(4.0 < vehicle_d && vehicle_d < 8.0 && ego_right > 4.0 && ego_left < 8.0)
    {
        return true ;
    }

    // Both in right lane
    if(8.0 < vehicle_d && ego_right > 8.0)
    {
        return true ;
    }

    // Not in the same lane
    return false ;
}


bool will_ego_collide_with_vehicle(
    vector<double> &ego_s_trajectory, vector<double> &ego_d_trajectory,
    double vehicle_s, double vehicle_d, double vehicle_vs, double vehicle_vd, double time_per_step,
    double safety_s_distance, double safety_d_distance)
{
    for(int index = 0 ; index < ego_s_trajectory.size() ; index++)
    {
        double ego_s = ego_s_trajectory[index] ;
        double ego_d = ego_d_trajectory[index] ;

        double time = time_per_step * double(index) ;

        double current_vehicle_s = vehicle_s + (vehicle_vs * time) ;
        double current_vehicle_d = vehicle_d + (vehicle_vd * time) ;

        double s_distance = ego_s - current_vehicle_s ;
        double d_distance = ego_d - current_vehicle_d ;

        if(std::abs(s_distance) < safety_s_distance && are_ego_and_vehicle_in_same_lane(ego_d, current_vehicle_d))
        {
            return true ;
        }

    }

    // No collision detected
    return false ;
}


double get_arc_angle(
    double first_x, double first_y, double second_x, double second_y, double third_x, double third_y)
{
    double a = distance(first_x, first_y, second_x, second_y) ;
    double b = distance(second_x, second_y, third_x, third_y) ;
    double c = distance(first_x, first_y, third_x, third_y) ;

    double cos_c = ((a * a) + (b * b) - (c * c)) / (2.0 * a * b) ;
    return std::acos(cos_c) ;
}


void print_vector(vector<double> &data)
{
    for(auto element: data)
    {
        std::cout << element << std::endl ;
    }
}



#endif //PATH_PLANNING_PROCESSING_H
