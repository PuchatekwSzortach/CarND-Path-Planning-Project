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

            double target_speed_cost = 100.0 * this->get_target_speed_cost(trajectory) ;
            double collision_cost = this->huge_cost * this->get_collision_cost(trajectory) ;
            double following_distance_cost = 2.0 *this->get_following_distance_cost(trajectory) ;
            double final_lane_change_cost = 0.1 * this->get_previous_trajectory_final_lane_change_cost(trajectory) ;
            double sharp_turn_speeding_cost = 100.0 * this->get_sharp_turn_speeding_cost(trajectory) ;

            std::cout << "\ttarget_speed_cost: " << target_speed_cost << ", collision_cost: " << collision_cost
                << ", following_distance_cost: " << following_distance_cost << ", \n\tfinal_lane_change_cost: "
                << final_lane_change_cost << ", sharp_turn_speeding_cost: " << sharp_turn_speeding_cost << std::endl ;

            cost = target_speed_cost + collision_cost + following_distance_cost + final_lane_change_cost +
                    sharp_turn_speeding_cost ;

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

    double get_collision_cost(Trajectory &trajectory)
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

            double front_safety_s_distance = 10.0 ;
            double back_safety_s_distance = 8.0 ;

            bool will_collide = will_ego_collide_with_vehicle(
                trajectory.s_trajectory, trajectory.d_trajectory,
                vehicle_s, vehicle_d, vehicle_sd_speed[0], vehicle_sd_speed[1],
                this->configuration.time_per_step, front_safety_s_distance, back_safety_s_distance) ;

            if(will_collide)
            {
                // Collision cost proportional to speed
                cost += trajectory.final_s_state[1] ;
            }

        }

        return cost ;
    }

    double get_following_distance_cost(Trajectory &trajectory)
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

            double front_safety_s_distance = 1.2 * this->configuration.target_speed ;
            double back_safety_s_distance = 0.3 * this->configuration.target_speed ;

            cost += this->get_ego_following_distance_to_vehicle_cost(
                trajectory, vehicle_s, vehicle_d, vehicle_sd_speed[0], vehicle_sd_speed[1],
                front_safety_s_distance, back_safety_s_distance) ;
        }

        return cost ;
    }


    double get_previous_trajectory_final_lane_change_cost(Trajectory &trajectory)
    {
        double cost = std::abs(trajectory.final_d_state[0] - this->previous_trajectory_final_d) < 1.0 ? 0 : 1 ;
        return cost ;
    }

    double get_ego_following_distance_to_vehicle_cost(
        Trajectory &trajectory, double vehicle_s, double vehicle_d, double vehicle_vs, double vehicle_vd,
        double front_safety_s_distance, double back_safety_s_distance)
    {
        double cost = 0 ;

        double ego_initial_s = ego_s_trajectory[0] ;
        double ego_initial_d = ego_d_trajectory[0] ;

        for(int index = 0 ; index < trajectory.s_trajectory.size() ; index++)
        {
            double ego_s = trajectory.s_trajectory[index] ;
            double ego_d = trajectory.d_trajectory[index] ;

            double time = this->configuration.time_per_step * double(index) ;

            double current_vehicle_s = vehicle_s + (vehicle_vs * time) ;
            double current_vehicle_d = vehicle_d + (vehicle_vd * time) ;

            double s_distance = current_vehicle_s - ego_s ;

            // If vehicle is in the same lane we started trajectory and was in front of us from the beginning
            if(are_ego_and_vehicle_in_same_lane(ego_initial_d, vehicle_d) && vehicle_s > ego_initial_s)
            {
                if(std::abs(s_distance) < front_safety_s_distance)
                {
                    cost += front_safety_s_distance / s_distance ;
                }
            }
            // Else vehicle is in different lane than we started - it might be coming fast from behind
            else
            {
                // If we will be in the same lane at some given time instant
                if(are_ego_and_vehicle_in_same_lane(ego_d, current_vehicle_d))
                {
                    if(std::abs(s_distance) < front_safety_s_distance)
                    {
                        cost += front_safety_s_distance / s_distance ;
                    }
                }
            }

        }
        return cost ;
    }

    double get_sharp_turn_speeding_cost(Trajectory &trajectory)
    {
        double cost = 0 ;

        if(is_car_going_through_sharp_turn(
            trajectory.x_trajectory.front(), trajectory.y_trajectory.front(), this->maps_x, this->maps_y) )
        {
            double final_speed = trajectory.final_s_state[1] ;
            cost += std::abs(configuration.target_speed - 10.0 - final_speed) / configuration.target_speed ;
        }

        return cost ;
    }

} ;



#endif //PATH_PLANNING_COST_H