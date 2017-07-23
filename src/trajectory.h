#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H


#include <vector>
#include <random>

#include "processing.h"

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
} ;



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
    vector<double> maps_dx ;
    vector<double> maps_dy ;

    TrajectoryPlanner(
        vector<double> maps_x, vector<double> maps_y, vector<double> maps_s,
        vector<double> maps_dx, vector<double> maps_dy)
    {
        this->maps_x = maps_x ;
        this->maps_y = maps_y ;
        this->maps_s = maps_s ;

        this->maps_dx = maps_dx ;
        this->maps_dy = maps_dy ;
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

    // Return 4D vector with x, y, s and d trajectories
    vector<vector<double>> get_trajectory_based_on_previous_trajectory(
        double car_x, double car_y, double car_s, double car_d,
        vector<double> previous_trajectory_x, vector<double> previous_trajectory_y)
    {
        // Get index of closest point in saved trajectory to current car position
        double current_position_index = this->get_index_of_closest_saved_trajectory_point(car_x, car_y) ;

        vector<double> x_trajectory ;
        vector<double> y_trajectory ;
        vector<double> s_trajectory ;
        vector<double> d_trajectory ;

        // Copy old trajectories from found index till end
        for(int index = current_position_index ; index < this->saved_x_trajectory.size() ; ++index)
        {
            x_trajectory.push_back(this->saved_x_trajectory[index]) ;
            y_trajectory.push_back(this->saved_y_trajectory[index]) ;

            s_trajectory.push_back(this->saved_s_trajectory[index]) ;
            d_trajectory.push_back(this->saved_d_trajectory[index]) ;
        }

        double time_horizon = 2.0 ;
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

        auto added_xy_trajectory = convert_frenet_trajectory_to_cartesian_trajectory(
            added_s_trajectory, added_d_trajectory, this->maps_s, this->maps_x, this->maps_y) ;

        auto added_x_trajectory = added_xy_trajectory[0] ;
        auto added_y_trajectory = added_xy_trajectory[1] ;

        // Remove last 10 elements from current trajectory and add them to new trajectory - so we will later smooth over
        // last 10 elements of old trajectory and added trajectory
        if(x_trajectory.size() > 10)
        {
            for(int index = 0 ; index < 10 ; ++index)
            {
                auto x = x_trajectory.back() ;
                x_trajectory.pop_back() ;
                added_x_trajectory.insert(added_x_trajectory.begin(), x) ;

                auto y = y_trajectory.back() ;
                y_trajectory.pop_back() ;
                added_y_trajectory.insert(added_y_trajectory.begin(), y) ;

                auto s = s_trajectory.back() ;
                s_trajectory.pop_back() ;
                added_s_trajectory.insert(added_s_trajectory.begin(), s) ;

                auto d = d_trajectory.back() ;
                d_trajectory.pop_back() ;
                added_d_trajectory.insert(added_d_trajectory.begin(), d) ;

                auto time = added_time_steps[0] ;
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
            x_trajectory.push_back(smooth_x_trajectory[index]) ;
            y_trajectory.push_back(smooth_y_trajectory[index]) ;

            s_trajectory.push_back(smooth_s_trajectory[index]) ;
            d_trajectory.push_back(smooth_d_trajectory[index]) ;
        }


//        print_trajectory(d_trajectory) ;

        vector<vector<double>> xysd_trajectory {x_trajectory, y_trajectory, s_trajectory, d_trajectory} ;
        return xysd_trajectory ;
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

    std::default_random_engine random_generator;
    std::normal_distribution<double> s_speed_distribution ;

    TrajectoriesGenerator(
        vector<double> maps_x, vector<double> maps_y, vector<double> maps_s,
        vector<double> maps_dx, vector<double> maps_dy)
    {
        this->maps_x = maps_x ;
        this->maps_y = maps_y ;
        this->maps_s = maps_s ;

        this->maps_dx = maps_dx ;
        this->maps_dy = maps_dy ;

        this->random_generator = std::default_random_engine() ;
        this->s_speed_distribution = std::normal_distribution<double>(20.0, 10.0) ;
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

    int get_index_of_closest_previous_x_trajectory_point(double car_x, double car_y)
    {
        int best_index = 0 ;
        double best_distance = distance(
            car_x, car_y, this->previous_x_trajectory[best_index], this->previous_y_trajectory[best_index]) ;

        for(int index = 1 ; index < this->previous_x_trajectory.size() ; ++index)
        {
            double current_distance = distance(
            car_x, car_y, this->previous_x_trajectory[index], this->previous_y_trajectory[index]) ;

            if(current_distance < best_distance)
            {
                best_distance = current_distance ;
                best_index = index ;
            }

        }

        return best_index ;
    }

    // Generates multiple candidate final s states
    vector<vector<double>> generate_final_s_states(
        vector<double> &initial_s_state, double time_horizon, double time_per_step)
    {
        double initial_s = initial_s_state[0] ;
        double initial_speed = initial_s_state[1] ;
        double initial_acceleration = initial_s_state[2] ;

        vector<double> speed_values {0, 10, 15, 20, 25} ;

        for(auto speed : speed_values)
        {
            std::cout << speed << std::endl ;
        }

        vector<vector<double>> final_s_states ;
        return final_s_states ;
    }

    // Generates candidate trajectories
    vector<Trajectory> generate_trajectories(
        double car_x, double car_y, double car_s, double car_d,
        vector<double> current_trajectory_x, vector<double> current_trajectory_y)
    {
        // Get index of closest point in saved trajectory to current car position
        double current_position_index = this->get_index_of_closest_previous_x_trajectory_point(car_x, car_y) ;

        vector<double> x_trajectory ;
        vector<double> y_trajectory ;
        vector<double> s_trajectory ;
        vector<double> d_trajectory ;

        // Copy old trajectories from found index till end
        for(int index = current_position_index ; index < this->previous_x_trajectory.size() ; ++index)
        {
            x_trajectory.push_back(this->previous_x_trajectory[index]) ;
            y_trajectory.push_back(this->previous_y_trajectory[index]) ;

            s_trajectory.push_back(this->previous_s_trajectory[index]) ;
            d_trajectory.push_back(this->previous_d_trajectory[index]) ;
        }

        double time_horizon = 2.0 ;
        double steps_per_second = 50.0 ;
        double time_per_step = 1.0 / steps_per_second ;

        vector<double> initial_s_state = get_initial_s_state(s_trajectory, time_per_step) ;
        vector<double> initial_d_state = get_initial_d_state(d_trajectory, time_per_step) ;

        auto final_s_states = this->generate_final_s_states(initial_s_state, time_horizon, time_per_step) ;




        vector<Trajectory> trajectories ;
        return trajectories ;
    }


} ;


#endif //PATH_PLANNING_TRAJECTORY_H