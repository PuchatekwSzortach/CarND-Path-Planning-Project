#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H


#include <vector>
#include "processing.h"

using namespace std ;

class TrajectoryPlanner
{

    public:

    vector<double> saved_x_trajectory ;
    vector<double> saved_y_trajectory ;
    vector<double> saved_s_trajectory ;
    vector<double> saved_d_trajectory ;

    vector<double> maps_x ;
    vector<double> maps_y ;
    vector<double> maps_s ;

    TrajectoryPlanner(vector<double> maps_x, vector<double> maps_y, vector<double> maps_s)
    {
        this->maps_x = maps_x ;
        this->maps_y = maps_y ;
        this->maps_s = maps_s ;
    }

    void save_trajectories(
        vector<double> x_trajectory, vector<double> y_trajectory,
        vector<double> s_trajectory, vector<double> d_trajectory)

    {
        this->saved_x_trajectory = x_trajectory ;
        this->saved_y_trajectory = y_trajectory ;

        this->saved_s_trajectory = s_trajectory ;
        this->saved_d_trajectory = d_trajectory ;
    }

    void set_saved_trajectories_from_current_state(double car_x, double car_y, double car_s, double car_d)
    {
        for(int index = 0 ; index < 3 ; ++index)
        {
            this->saved_x_trajectory.push_back(car_x) ;
            this->saved_y_trajectory.push_back(car_y) ;

            this->saved_s_trajectory.push_back(car_s) ;
            this->saved_d_trajectory.push_back(car_d) ;
        }

    }

    int get_index_of_closest_saved_trajectory_point(double car_x, double car_y)
    {
        int best_index = 0 ;
        double best_distance = distance(
            car_x, car_y, this->saved_x_trajectory[best_index], this->saved_y_trajectory[best_index]) ;

        for(int index = 1 ; index < this->saved_x_trajectory.size() ; ++index)
        {
            double current_distance = distance(
            car_x, car_y, this->saved_x_trajectory[index], this->saved_y_trajectory[index]) ;

            if(current_distance < best_distance)
            {
                best_distance = current_distance ;
                best_index = index ;
            }

        }

        return best_index ;
    }

    vector<vector<double>> get_smoothed_out_xy_trajectory_from_sd_trajectory(
        vector<double> &s_trajectory, vector<double> &d_trajectory, double time_per_step)
    {
        auto xy_trajectory = convert_frenet_trajectory_to_cartesian_trajectory(
            s_trajectory, d_trajectory, this->maps_s, this->maps_x, this->maps_y) ;

        vector<double> complete_time_steps ;
        double time_instant = 0 ;

        for(int index = 0 ; index < s_trajectory.size() ; ++index)
        {
            complete_time_steps.push_back(time_instant) ;
            time_instant += time_per_step ;
        }

        auto smooth_x_trajectory = get_smoothed_trajectory(complete_time_steps, xy_trajectory[0]) ;
        auto smooth_y_trajectory = get_smoothed_trajectory(complete_time_steps, xy_trajectory[1]) ;

        vector<vector<double>> smooth_xy_trajectory {smooth_x_trajectory, smooth_y_trajectory} ;
        return smooth_xy_trajectory ;
    }

    // Return 4D vector with x, y, s and d trajectories
    vector<vector<double>> get_trajectory_based_on_previous_trajectory(
        double car_x, double car_y, double car_s, double car_d,
        vector<double> previous_trajectory_x, vector<double> previous_trajectory_y)
    {
        // Get index of closest point in saved trajectory to current car position
        double current_position_index = this->get_index_of_closest_saved_trajectory_point(car_x, car_y) ;

        vector<double> s_trajectory ;
        vector<double> d_trajectory ;

        // Copy old trajectories from found index till end
        for(int index = current_position_index ; index < this->saved_s_trajectory.size() ; ++index)
        {
            s_trajectory.push_back(this->saved_s_trajectory[index]) ;
            d_trajectory.push_back(this->saved_d_trajectory[index]) ;
        }

        double time_horizon = 4.0 ;
        double steps_per_second = 50.0 ;
        double time_per_step = 1.0 / steps_per_second ;

        vector<double> initial_s_state = get_initial_s_state(s_trajectory, time_per_step) ;
        vector<double> final_s_state = get_final_s_state(s_trajectory, time_horizon, time_per_step) ;

        vector<double> initial_d_state = get_initial_d_state(d_trajectory, time_per_step) ;
        double target_d = 6.0 ;
        vector<double> final_d_state = get_final_d_state(d_trajectory, target_d, time_horizon, time_per_step) ;

        auto s_coefficients = get_jerk_minimizing_trajectory_coefficients(
            initial_s_state, final_s_state, time_horizon) ;

        auto d_coefficients = get_jerk_minimizing_trajectory_coefficients(
            initial_d_state, final_d_state, time_horizon) ;

        vector<double> added_time_steps ;

        // End of previous path already includes a point at our initial state, so add new trajectories
        // from one instant after that
        double time_instant = time_per_step ;

        while(time_instant <= time_horizon)
        {
            added_time_steps.push_back(time_instant) ;
            time_instant += time_per_step ;
        }

        auto added_s_trajectory = evaluate_polynomial_over_vector(s_coefficients, added_time_steps) ;
        auto added_d_trajectory = evaluate_polynomial_over_vector(d_coefficients, added_time_steps) ;

        for(int index = 0 ; index < added_s_trajectory.size() ; ++index)
        {
            s_trajectory.push_back(added_s_trajectory[index]) ;
            d_trajectory.push_back(added_d_trajectory[index]) ;
        }

        auto xy_trajectory = this->get_smoothed_out_xy_trajectory_from_sd_trajectory(
            s_trajectory, d_trajectory, time_per_step) ;

//        print_trajectory(xy_trajectory[1]) ;

        vector<vector<double>> xysd_trajectory {xy_trajectory[0], xy_trajectory[1], s_trajectory, d_trajectory} ;
        return xysd_trajectory ;
    }

} ;


#endif //PATH_PLANNING_TRAJECTORY_H