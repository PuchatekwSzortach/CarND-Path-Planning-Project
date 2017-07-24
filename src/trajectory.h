#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H


#include <vector>
#include <random>

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

    void set_previous_trajectories_from_current_state(double car_x, double car_y, double car_s, double car_d)
    {
        for(int index = 0 ; index < 3 ; ++index)
        {
            this->previous_x_trajectory.push_back(car_x) ;
            this->previous_y_trajectory.push_back(car_y) ;

            this->previous_s_trajectory.push_back(car_s) ;
            this->previous_d_trajectory.push_back(car_d) ;
        }

    }

    // Generates multiple candidate final s states
    vector<vector<double>> generate_final_s_states(
        vector<double> &initial_s_state, double time_horizon, double time_per_step)
    {
        double initial_s = initial_s_state[0] ;
        double initial_speed = initial_s_state[1] ;
        double initial_acceleration = initial_s_state[2] ;

        vector<double> speed_values {this->configuration.target_speed, 0.5 * this->configuration.target_speed} ;
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

        vector<double> ideal_positions {6} ;
        vector<vector<double>> final_d_states ;

        for(auto ideal_position: ideal_positions)
        {
            auto final_state = get_final_d_state(initial_d_state, ideal_position, time_horizon, time_per_step) ;
            final_d_states.push_back(final_state) ;
        }

        return final_d_states ;
    }

    Trajectory generate_trajectory(
        vector<double> &initial_s_state, vector<double> &final_s_state,
        vector<double> &initial_d_state, vector<double> &final_d_state,
        vector<double> initial_x_trajectory, vector<double> initial_y_trajectory,
        vector<double> initial_s_trajectory, vector<double> initial_d_trajectory,
        double time_horizon, double time_per_step)
    {
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

        auto added_xy_trajectory = convert_frenet_trajectory_to_cartesian_trajectory(
            added_s_trajectory, added_d_trajectory, this->maps_s, this->maps_x, this->maps_y) ;

        auto added_x_trajectory = added_xy_trajectory[0] ;
        auto added_y_trajectory = added_xy_trajectory[1] ;

        // Remove last n elements from current trajectory and add them to new trajectory - so we will later smooth over
        // last 10 elements of old trajectory and added trajectory
        int elements_count = 10 ;
        if(initial_x_trajectory.size() > elements_count)
        {
            move_n_elements(initial_x_trajectory, added_x_trajectory, elements_count) ;
            move_n_elements(initial_y_trajectory, added_y_trajectory, elements_count) ;
            move_n_elements(initial_s_trajectory, added_s_trajectory, elements_count) ;
            move_n_elements(initial_d_trajectory, added_d_trajectory, elements_count) ;

            for(int index = 0 ; index < elements_count ; ++index)
            {
                double time = added_time_steps[0] ;
                double previous_instant_time = time - time_per_step ;
                added_time_steps.insert(added_time_steps.begin(), previous_instant_time) ;
            }
        }

        auto smooth_s_trajectory = get_smoothed_trajectory(added_time_steps, added_s_trajectory) ;
        auto smooth_d_trajectory = get_smoothed_trajectory(added_time_steps, added_d_trajectory) ;

        auto smooth_x_trajectory = get_smoothed_trajectory(added_time_steps, added_x_trajectory) ;
        auto smooth_y_trajectory = get_smoothed_trajectory(added_time_steps, added_y_trajectory) ;

        for(int index = 0 ; index < added_s_trajectory.size() ; ++index)
        {
            initial_x_trajectory.push_back(smooth_x_trajectory[index]) ;
            initial_y_trajectory.push_back(smooth_y_trajectory[index]) ;

            initial_s_trajectory.push_back(smooth_s_trajectory[index]) ;
            initial_d_trajectory.push_back(smooth_d_trajectory[index]) ;
        }

        Trajectory trajectory(
            initial_x_trajectory, initial_y_trajectory, initial_s_trajectory, initial_d_trajectory,
            initial_s_state, final_s_state, initial_d_state, final_d_state) ;

        return trajectory ;
    }

    // Generates candidate trajectories
    vector<Trajectory> generate_trajectories(
        double car_x, double car_y, double car_s, double car_d,
        vector<double> current_trajectory_x, vector<double> current_trajectory_y)
    {
        // Get index of closest point in saved trajectory to current car position
        double current_position_index = get_index_of_closest_previous_x_trajectory_point(
            car_x, car_y, this->previous_x_trajectory, this->previous_y_trajectory) ;

        vector<double> initial_x_trajectory ;
        vector<double> initial_y_trajectory ;
        vector<double> initial_s_trajectory ;
        vector<double> initial_d_trajectory ;

        // Copy old trajectories from found index till end
        for(int index = current_position_index ; index < this->previous_x_trajectory.size() ; ++index)
        {
            initial_x_trajectory.push_back(this->previous_x_trajectory[index]) ;
            initial_y_trajectory.push_back(this->previous_y_trajectory[index]) ;

            initial_s_trajectory.push_back(this->previous_s_trajectory[index]) ;
            initial_d_trajectory.push_back(this->previous_d_trajectory[index]) ;
        }

        double time_horizon = 2.0 ;
        double steps_per_second = 50.0 ;
        double time_per_step = 1.0 / steps_per_second ;

        vector<double> initial_s_state = get_initial_s_state(initial_s_trajectory, time_per_step) ;
        vector<double> initial_d_state = get_initial_d_state(initial_d_trajectory, time_per_step) ;

        auto final_s_states = this->generate_final_s_states(initial_s_state, time_horizon, time_per_step) ;
        auto final_d_states = this->generate_final_d_states(initial_d_state, time_horizon, time_per_step) ;

        vector<Trajectory> trajectories ;

        for(auto final_s_state: final_s_states)
        {
            for(auto final_d_state: final_d_states)
            {
                auto trajectory = this->generate_trajectory(
                    initial_s_state, final_s_state, initial_d_state, final_d_state,
                    initial_x_trajectory, initial_y_trajectory, initial_s_trajectory, initial_d_trajectory,
                    time_horizon, time_per_step) ;

                trajectories.push_back(trajectory) ;
            }
        }

        return trajectories ;
    }


} ;


#endif //PATH_PLANNING_TRAJECTORY_H