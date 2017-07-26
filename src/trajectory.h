#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H


#include <vector>
#include <random>
#include <algorithm>
#include <cmath>

#include "processing.h"
#include "config.h"

using namespace std ;

// Stores information about a single trajectory, including its initial and final states
class Trajectory
{
    public:

    vector<double> initial_s_state ;
    vector<double> final_s_state ;

    vector<double> initial_d_state ;
    vector<double> final_d_state ;

    vector<double> s_trajectory ;
    vector<double> d_trajectory ;
    vector<double> x_trajectory ;
    vector<double> y_trajectory ;

    Trajectory(
        vector<double> x_trajectory, vector<double> y_trajectory,
        vector<double> s_trajectory, vector<double> d_trajectory,
        vector<double> initial_s_state, vector<double> final_s_state,
        vector<double> initial_d_state, vector<double> final_d_state)
    {
        this->x_trajectory = x_trajectory ;
        this->y_trajectory = y_trajectory ;
        this->s_trajectory = s_trajectory ;
        this->d_trajectory = d_trajectory ;

        this->initial_s_state = initial_s_state ;
        this->final_s_state = final_s_state ;

        this->initial_d_state = initial_d_state ;
        this->final_d_state = final_d_state ;
    }

    void print()
    {
        std::cout << "Trajectory:\n" ;

        std::cout << "Initial s: " << this->initial_s_state[0] << ", " << this->initial_s_state[1]
            << ", " << this->initial_s_state[2] << std::endl ;

        std::cout << "Final s: " << this->final_s_state[0] << ", " << this->final_s_state[1]
            << ", " << this->final_s_state[2] << std::endl ;

        std::cout << "Final d: " << this->final_d_state[0] << ", " << this->final_d_state[1]
            << ", " << this->final_d_state[2] << std::endl ;
    }
} ;


// Generates multiple sd trajectories
class TrajectoriesGenerator
{
    public:

    vector<double> previous_x_trajectory ;
    vector<double> previous_y_trajectory ;
    vector<double> previous_s_trajectory ;
    vector<double> previous_d_trajectory ;

    vector<double> maps_x ;
    vector<double> maps_y ;
    vector<double> maps_s ;
    vector<double> maps_dx ;
    vector<double> maps_dy ;

    Configuration configuration ;

    TrajectoriesGenerator(
        vector<double> maps_x, vector<double> maps_y, vector<double> maps_s,
        vector<double> maps_dx, vector<double> maps_dy, Configuration configuration)
    {
        this->maps_x = maps_x ;
        this->maps_y = maps_y ;
        this->maps_s = maps_s ;

        this->maps_dx = maps_dx ;
        this->maps_dy = maps_dy ;

        this->configuration = configuration ;
    }

    void set_previous_trajectories(
        vector<double> x_trajectory, vector<double> y_trajectory,
        vector<double> s_trajectory, vector<double> d_trajectory)
    {
        this->previous_x_trajectory = x_trajectory ;
        this->previous_y_trajectory = y_trajectory ;

        this->previous_s_trajectory = s_trajectory ;
        this->previous_d_trajectory = d_trajectory ;
    }

    void set_previous_trajectories_from_current_state(double car_s, double car_d)
    {
        double acceleration = 1.0 ;
        double time_instant = 0 ;

        while(time_instant < this->configuration.trajectory_time)
        {
            double s = car_s + (0.5 * acceleration * time_instant * time_instant) ;
            time_instant += this->configuration.time_per_step ;

            this->previous_s_trajectory.push_back(s) ;
            this->previous_d_trajectory.push_back(car_d) ;
        }

        auto xy_trajectory = convert_frenet_trajectory_to_cartesian_trajectory(
            this->previous_s_trajectory, this->previous_d_trajectory, this->maps_s, this->maps_x, this->maps_y) ;

        this->previous_x_trajectory = xy_trajectory[0] ;
        this->previous_y_trajectory = xy_trajectory[1] ;
    }

    // Generates multiple candidate final s states
    vector<vector<double>> generate_final_s_states(
        vector<double> &initial_s_state, double time_horizon, double time_per_step)
    {
        double initial_s = initial_s_state[0] ;
        double initial_speed = initial_s_state[1] ;
        double initial_acceleration = initial_s_state[2] ;

//        vector<double> fractions {1.0, 0.75, 0.5} ;
        vector<double> fractions {1.0, 0.9, 0.8, 0.7, 0.6, 0.5} ;
//        vector<double> fractions {1.0, 0.25} ;

        vector<double> speed_values ;
        for(double fraction: fractions)
        {
            speed_values.push_back(fraction * this->configuration.target_speed) ;
        }

        vector<vector<double>> final_s_states ;

        for(auto speed: speed_values)
        {
            auto final_state = get_final_s_state(initial_s_state, speed, time_horizon, time_per_step) ;
            final_s_states.push_back(final_state) ;
        }

        return final_s_states ;
    }

    vector<vector<double>> generate_final_d_states(
        vector<double> &initial_d_state, double time_horizon, double time_per_step)
    {
        double initial_position = initial_d_state[0] ;
        double initial_speed = initial_d_state[1] ;
        double initial_acceleration = initial_d_state[2] ;

        double ideal_position = 6 ;

        vector<double> ideal_positions {2.0, 6.0, 10.0} ;
//        vector<double> ideal_positions {6.0} ;

        vector<vector<double>> final_d_states ;

        for(auto ideal_position: ideal_positions)
        {
            auto final_state = get_final_d_state(initial_d_state, ideal_position, time_horizon, time_per_step) ;
            final_d_states.push_back(final_state) ;
        }

        return final_d_states ;
    }

    Trajectory generate_trajectory(
        vector<double> initial_x_trajectory, vector<double> initial_y_trajectory,
        vector<double> initial_s_trajectory, vector<double> initial_d_trajectory,
        vector<double> next_segment_initial_s_state, vector<double> final_s_state,
        vector<double> next_segment_initial_d_state, vector<double> final_d_state,
        double time_horizon, double time_per_step)
    {
        auto s_coefficients = get_jerk_minimizing_trajectory_coefficients(
            next_segment_initial_s_state, final_s_state, time_horizon) ;

        auto d_coefficients = get_jerk_minimizing_trajectory_coefficients(
            next_segment_initial_d_state, final_d_state, time_horizon) ;

        // End of previous path already includes a point at our initial state, so add new trajectories
        // from one instant after that
        vector<double> new_segment_time_steps ;
        double time_instant = time_per_step ;

        while(time_instant <= time_horizon)
        {
            new_segment_time_steps.push_back(time_instant) ;
            time_instant += time_per_step ;
        }

        auto new_segment_s_trajectory = evaluate_polynomial_over_vector(s_coefficients, new_segment_time_steps) ;
        auto new_segment_d_trajectory = evaluate_polynomial_over_vector(d_coefficients, new_segment_time_steps) ;

        auto new_segment_xy_trajectory = convert_frenet_trajectory_to_cartesian_trajectory(
            new_segment_s_trajectory, new_segment_d_trajectory, this->maps_s, this->maps_x, this->maps_y) ;

        auto new_segment_x_trajectory = new_segment_xy_trajectory[0] ;
        auto new_segment_y_trajectory = new_segment_xy_trajectory[1] ;

        // Smooth out new segments
        auto new_segment_x_trajectory_smooth = get_smoothed_trajectory(new_segment_time_steps, new_segment_x_trajectory) ;
        auto new_segment_y_trajectory_smooth = get_smoothed_trajectory(new_segment_time_steps, new_segment_y_trajectory) ;
        auto new_segment_s_trajectory_smooth = get_smoothed_trajectory(new_segment_time_steps, new_segment_s_trajectory) ;
        auto new_segment_d_trajectory_smooth = get_smoothed_trajectory(new_segment_time_steps, new_segment_d_trajectory) ;


        vector<double> blending_s_segment, blending_d_segment, blending_x_segment, blending_y_segment ;

        // Move last n elements from end of initial trajectory and beginning of new trajectory into
        // a blending segment
        int elements_count = 40 ;
        if(initial_x_trajectory.size() > elements_count)
        {
            move_n_elements_from_end_of_first_to_beginning_of_second(initial_x_trajectory, blending_x_segment, elements_count) ;
            move_n_elements_from_end_of_first_to_beginning_of_second(initial_y_trajectory, blending_y_segment, elements_count) ;
            move_n_elements_from_end_of_first_to_beginning_of_second(initial_s_trajectory, blending_s_segment, elements_count) ;
            move_n_elements_from_end_of_first_to_beginning_of_second(initial_d_trajectory, blending_d_segment, elements_count) ;

            move_n_elements_from_beginning_of_first_to_end_of_second(new_segment_x_trajectory_smooth, blending_x_segment, elements_count) ;
            move_n_elements_from_beginning_of_first_to_end_of_second(new_segment_y_trajectory_smooth, blending_y_segment, elements_count) ;
            move_n_elements_from_beginning_of_first_to_end_of_second(new_segment_s_trajectory_smooth, blending_s_segment, elements_count) ;
            move_n_elements_from_beginning_of_first_to_end_of_second(new_segment_d_trajectory_smooth, blending_d_segment, elements_count) ;

            vector<double> blending_time ;
            double time_instant = 0 ;
            for(int index = 0 ; index < blending_x_segment.size() ; ++index)
            {
                time_instant += time_per_step ;
                blending_time.push_back(time_instant) ;
            }

            auto smooth_blending_x_segment = get_smoothed_trajectory(blending_time, blending_x_segment) ;
            auto smooth_blending_y_segment = get_smoothed_trajectory(blending_time, blending_y_segment) ;
            auto smooth_blending_s_segment = get_smoothed_trajectory(blending_time, blending_s_segment) ;
            auto smooth_blending_d_segment = get_smoothed_trajectory(blending_time, blending_d_segment) ;

            for(int index = 0 ; index < smooth_blending_x_segment.size() ; ++index)
            {
                initial_x_trajectory.push_back(smooth_blending_x_segment[index]) ;
                initial_y_trajectory.push_back(smooth_blending_y_segment[index]) ;
                initial_s_trajectory.push_back(smooth_blending_s_segment[index]) ;
                initial_d_trajectory.push_back(smooth_blending_d_segment[index]) ;
            }
        }

        for(int index = 0 ; index < new_segment_x_trajectory_smooth.size() ; ++index)
        {
            initial_x_trajectory.push_back(new_segment_x_trajectory_smooth[index]) ;
            initial_y_trajectory.push_back(new_segment_y_trajectory_smooth[index]) ;

            initial_s_trajectory.push_back(new_segment_s_trajectory_smooth[index]) ;
            initial_d_trajectory.push_back(new_segment_d_trajectory_smooth[index]) ;
        }

        auto initial_s_state = get_s_state_at_trajectory_start(initial_s_trajectory, time_per_step) ;
        auto initial_d_state = get_d_state_at_trajectory_start(initial_d_trajectory, time_per_step) ;

        Trajectory trajectory(
            initial_x_trajectory, initial_y_trajectory, initial_s_trajectory, initial_d_trajectory,
            initial_s_state, final_s_state, initial_d_state, final_d_state) ;

        if(trajectory.s_trajectory.back() > this->configuration.s_end)
        {
            trajectory = get_trajectory_adjusted_for_going_over_lap_end(trajectory) ;
        }

        return trajectory ;

    }


    Trajectory get_trajectory_adjusted_for_going_over_lap_end(Trajectory trajectory)
    {
        vector<double> s_trajectory ;

        for(int index = 0 ; index < trajectory.s_trajectory.size() ; ++index)
        {
            double s = std::fmod(trajectory.s_trajectory[index], this->configuration.s_end) ;
            s_trajectory.push_back(s) ;
        }

        auto xy_trajectories = convert_frenet_trajectory_to_cartesian_trajectory(
            s_trajectory, trajectory.d_trajectory, this->maps_s, this->maps_x, this->maps_y) ;

        vector<double> trajectory_time ;
        double time_instant = 0 ;
        for(int index = 0 ; index < s_trajectory.size() ; ++index)
        {
            time_instant += this->configuration.time_per_step ;
            trajectory_time.push_back(time_instant) ;
        }

        auto smooth_x_trajectory = get_smoothed_trajectory(trajectory_time, xy_trajectories[0]) ;
        auto smooth_y_trajectory = get_smoothed_trajectory(trajectory_time, xy_trajectories[1]) ;

        Trajectory adjusted_trajectory(
            smooth_x_trajectory, smooth_y_trajectory, s_trajectory, trajectory.d_trajectory,
            trajectory.initial_s_state, trajectory.final_s_state,
            trajectory.initial_d_state, trajectory.final_d_state) ;

        return adjusted_trajectory ;
    }



    // Generates candidate trajectories
    vector<Trajectory> generate_trajectories(
        double car_x, double car_y, double car_s, double car_d,
        vector<double> current_trajectory_x, vector<double> current_trajectory_y)
    {
        // Get index of closest point in saved trajectory to current car position
        double current_position_index = get_index_of_closest_previous_x_trajectory_point(
            car_x, car_y, this->previous_x_trajectory, this->previous_y_trajectory) ;

//        double current_position_index = get_index_of_closest_s_trajectory_point(car_s, this->previous_s_trajectory) ;

        vector<double> initial_x_trajectory ;
        vector<double> initial_y_trajectory ;
        vector<double> initial_s_trajectory ;
        vector<double> initial_d_trajectory ;

        // Copy old trajectories from found index till some time
        int end_index = current_position_index + std::min(
            int(this->configuration.trajectory_update_interval / this->configuration.time_per_step),
            int(this->previous_x_trajectory.size())) ;

        for(int index = current_position_index ; index < end_index ; ++index)
        {
            initial_x_trajectory.push_back(this->previous_x_trajectory[index]) ;
            initial_y_trajectory.push_back(this->previous_y_trajectory[index]) ;

            initial_s_trajectory.push_back(this->previous_s_trajectory[index]) ;
            initial_d_trajectory.push_back(this->previous_d_trajectory[index]) ;
        }

        double time_horizon = configuration.trajectory_time - configuration.trajectory_update_interval ;
        double time_per_step = this->configuration.time_per_step ;

        vector<double> next_segment_initial_s_state = get_s_state_at_trajectory_end(
            initial_s_trajectory, time_per_step) ;

        vector<double> next_segment_initial_d_state = get_d_state_at_trajectory_end(
            initial_d_trajectory, time_per_step) ;

        auto final_s_states = this->generate_final_s_states(next_segment_initial_s_state, time_horizon, time_per_step) ;
        auto final_d_states = this->generate_final_d_states(next_segment_initial_d_state, time_horizon, time_per_step) ;

        vector<Trajectory> trajectories ;

        for(auto final_s_state: final_s_states)
        {
            for(auto final_d_state: final_d_states)
            {
                auto trajectory = this->generate_trajectory(
                    initial_x_trajectory, initial_y_trajectory, initial_s_trajectory, initial_d_trajectory,
                    next_segment_initial_s_state, final_s_state, next_segment_initial_d_state, final_d_state,
                    time_horizon, time_per_step) ;

                trajectories.push_back(trajectory) ;

            }
        }

        return trajectories ;
    }


} ;


#endif //PATH_PLANNING_TRAJECTORY_H