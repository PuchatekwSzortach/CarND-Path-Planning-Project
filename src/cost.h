#ifndef PATH_PLANNING_COST_H
#define PATH_PLANNING_COST_H


#include <vector>

#include "trajectory.h"
#include "config.h"
#include "processing.h"

using namespace std ;

class CostComputer
{
    public:

    std::vector<std::vector<double>> sensory_data ;
    Configuration configuration ;

    vector<double> maps_x ;
    vector<double> maps_y ;
    vector<double> maps_s ;
    vector<double> maps_dx ;
    vector<double> maps_dy ;

    double previous_trajectory_final_d ;

    double huge_cost ;

    CostComputer(
        Configuration configuration, std::vector<std::vector<double>> sensory_data,
        vector<double> &maps_x, vector<double> &maps_y, vector<double> &maps_s,
        vector<double> &maps_dx, vector<double> &maps_dy, double previous_trajectory_final_d)
    {
        this->configuration = configuration ;
        this->sensory_data = sensory_data ;

        this->maps_x = maps_x ;
        this->maps_y = maps_y ;
        this->maps_s = maps_s ;

        this->maps_dx = maps_dx ;
        this->maps_dy = maps_dy ;

        this->huge_cost = 1000 ;

        this->previous_trajectory_final_d = previous_trajectory_final_d ;
    }

    int get_lowest_cost_trajectory_index(vector<Trajectory> &trajectories)
    {
        std::cout << "\n\nGetting costs" << std::endl ;
        vector<double> costs ;

        for(int index = 0 ; index < trajectories.size() ; ++index)
        {
            auto trajectory = trajectories[index] ;
            trajectory.print() ;
            double cost = 0 ;

            cost += 50.0 * this->get_target_speed_cost(trajectory) ;
            cost += this->huge_cost * this->get_speeding_cost(trajectory) ;
            cost += this->huge_cost * this->get_safety_cost(trajectory) ;
            cost += 5.0 * this->get_previous_trajectory_final_lane_change_cost(trajectory) ;

            std::cout << "\tCost: " << cost << std::endl ;

            costs.push_back(cost) ;

        }
        return get_arg_min(costs) ;
    }

    double get_target_speed_cost(Trajectory &trajectory)
    {
        double final_speed = trajectory.final_s_state[1] ;
        return std::abs(configuration.target_speed - final_speed) / configuration.target_speed ;
    }

    double get_safety_cost(Trajectory &trajectory)
    {
        auto ego_initial_s = trajectory.initial_s_state ;
        auto ego_final_s = trajectory.final_s_state ;

        auto ego_initial_d = trajectory.initial_d_state ;
        auto ego_final_d = trajectory.final_d_state ;

        double cost = 0 ;

        for(auto vehicle_data: sensory_data)
        {
            int vehicle_id = vehicle_data[0] ;
            double vehicle_x = vehicle_data[1] ;
            double vehicle_y = vehicle_data[2] ;
            double vehicle_vx = vehicle_data[3] ;
            double vehicle_vy = vehicle_data[4] ;
            double vehicle_s = vehicle_data[5] ;
            double vehicle_d = vehicle_data[6] ;

            auto vehicle_sd_speed = get_sd_velocity_from_xy_velocity(
                vehicle_x, vehicle_y, vehicle_vx, vehicle_vy,
                this->maps_x, this->maps_y, this->maps_dx, this->maps_dy) ;

            double mean_velocity = 0.5 * (trajectory.initial_s_state[1] + trajectory.final_s_state[1]) ;
            double safety_s_distance = 0.2 * std::pow(mean_velocity, 1.5) ;
            double safety_d_distance = 3.5 ;

            bool will_collide = will_ego_collide_with_vehicle(
                trajectory.s_trajectory, trajectory.d_trajectory,
                vehicle_s, vehicle_d, vehicle_sd_speed[0], vehicle_sd_speed[1],
                this->configuration.time_per_step,
                safety_s_distance, safety_d_distance) ;

            if(will_collide)
            {
                // Collision cost proportional to speed
                cost += trajectory.final_s_state[1] ;
            }

        }

        return cost ;
    }

    // Cost for going over speed limit at any point of the trajectory
    double get_speeding_cost(Trajectory &trajectory)
    {
        auto s_trajectory = trajectory.s_trajectory ;

        // Check at resolution of one step
        int step_size = 1 ;
        for(int index = step_size ; index < s_trajectory.size() ; index += step_size)
        {
            auto speed = (s_trajectory[index] - s_trajectory[index - step_size]) /
                (double(step_size) * this->configuration.time_per_step) ;

            if(speed > 0.99 * this->configuration.speed_limit)
            {
                return 1.0 ;
            }

        }

        // Check at resolution of two steps
        step_size = 2 ;
        for(int index = step_size ; index < s_trajectory.size() ; index += step_size)
        {
            auto speed = (s_trajectory[index] - s_trajectory[index - step_size]) /
                (double(step_size) * this->configuration.time_per_step) ;

            if(speed > 0.99 * this->configuration.speed_limit)
            {
                return 1.0 ;
            }

        }

        // Check at resolution of four steps
        step_size = 4 ;
        for(int index = step_size ; index < s_trajectory.size() ; index += step_size)
        {
            auto speed = (s_trajectory[index] - s_trajectory[index - step_size]) /
                (double(step_size) * this->configuration.time_per_step) ;

            if(speed > 0.99 * this->configuration.speed_limit)
            {
                return 1.0 ;
            }

        }

        // Check at resolution of 10 steps
        step_size = 10 ;
        for(int index = step_size ; index < s_trajectory.size() ; index += step_size)
        {
            auto speed = (s_trajectory[index] - s_trajectory[index - step_size]) /
                (double(step_size) * this->configuration.time_per_step) ;

            if(speed > 0.99 * this->configuration.speed_limit)
            {
                return 1.0 ;
            }

        }

        // Check at resolution of 20 steps
        step_size = 20 ;
        for(int index = step_size ; index < s_trajectory.size() ; index += step_size)
        {
            auto speed = (s_trajectory[index] - s_trajectory[index - step_size]) /
                (double(step_size) * this->configuration.time_per_step) ;

            if(speed > 0.99 * this->configuration.speed_limit)
            {
                return 1.0 ;
            }

        }

        // Check at resolution of 40 steps
        step_size = 40 ;
        for(int index = step_size ; index < s_trajectory.size() ; index += step_size)
        {
            auto speed = (s_trajectory[index] - s_trajectory[index - step_size]) /
                (double(step_size) * this->configuration.time_per_step) ;

            if(speed > 0.99 * this->configuration.speed_limit)
            {
                return 1.0 ;
            }

        }

        return 0 ;

    }

    double get_tangential_acceleration_cost(Trajectory &trajectory)
    {
        double cost = 0 ;

        auto s_trajectory = trajectory.s_trajectory ;

        for(int index = 2 ; index < s_trajectory.size() ; ++index)
        {
            auto first_speed = (s_trajectory[index - 1] - s_trajectory[index - 2]) / this->configuration.time_per_step ;
            auto second_speed = (s_trajectory[index] - s_trajectory[index - 1]) / this->configuration.time_per_step ;

            double acceleration = (second_speed - first_speed) / this->configuration.time_per_step ;

            if(std::abs(acceleration) > this->configuration.tangential_acceleration_limit)
            {
                cost += this->huge_cost ;
            }

        }

        return cost ;
    }

    double get_normal_acceleration_cost(Trajectory &trajectory)
    {
        double cost = 0 ;

        auto d_trajectory = trajectory.d_trajectory ;

        for(int index = 2 ; index < d_trajectory.size() ; ++index)
        {
            auto first_speed = (d_trajectory[index - 1] - d_trajectory[index - 2]) / this->configuration.time_per_step ;
            auto second_speed = (d_trajectory[index] - d_trajectory[index - 1]) / this->configuration.time_per_step ;

            double acceleration = (second_speed - first_speed) / this->configuration.time_per_step ;

            if(std::abs(acceleration) > this->configuration.normal_acceleration_limit)
            {
                cost += this->huge_cost ;
            }

        }

        return cost ;
    }

    double get_sharp_turns_cost(Trajectory &trajectory)
    {
        double cost = 0 ;

        int first_index = 0 ;
        int second_index = int(this->configuration.trajectory_update_interval / this->configuration.time_per_step) ;
        int third_index = 2 * second_index ;

        double first_x = trajectory.x_trajectory[first_index] ;
        double first_y = trajectory.y_trajectory[first_index] ;

        double second_x = trajectory.x_trajectory[second_index] ;
        double second_y = trajectory.y_trajectory[second_index] ;

        double third_x = trajectory.x_trajectory[third_index] ;
        double third_y = trajectory.y_trajectory[third_index] ;

        double angle = get_arc_angle(first_x, first_y, second_x, second_y, third_x, third_y) ;

        double angle_degrees = rad2deg(angle) ;

        double difference_from_line = std::abs(angle_degrees - 180.0) ;
        return difference_from_line * difference_from_line ;
    }

    double get_previous_trajectory_final_lane_change_cost(Trajectory &trajectory)
    {
        double cost = std::abs(trajectory.final_d_state[0] - this->previous_trajectory_final_d) < 1.0 ? 0 : 1 ;
        return cost ;
    }

} ;



#endif //PATH_PLANNING_COST_H