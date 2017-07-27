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
            double final_lane_change_cost = 5.0 * this->get_previous_trajectory_final_lane_change_cost(trajectory) ;
            double speeding_cost = this->huge_cost * this->get_speeding_cost(trajectory) ;

            std::cout << "\ttarget_speed_cost: " << target_speed_cost << ", collision_cost: " << collision_cost
                << ", following_distance_cost: " << following_distance_cost << ", \n\tfinal_lane_change_cost: "
                << final_lane_change_cost << ", speeding cost: " << speeding_cost << std::endl ;

            cost = target_speed_cost + collision_cost + following_distance_cost + final_lane_change_cost + speeding_cost;

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
                trajectory.s_trajectory, trajectory.d_trajectory, trajectory.initial_s_state[1], trajectory.final_s_state[1],
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

            double front_safety_s_distance = 1.0 * this->configuration.target_speed ;
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

        double ego_initial_s = trajectory.s_trajectory[0] ;
        double ego_initial_d = trajectory.d_trajectory[0] ;
        double ego_final_d = trajectory.d_trajectory.back() ;

        double ego_minimum_speed = std::min(trajectory.initial_s_state[1], trajectory.final_s_state[1]) ;
        double ego_maximum_speed = std::max(trajectory.initial_s_state[1], trajectory.final_s_state[1]) ;

        for(int index = 0 ; index < trajectory.s_trajectory.size() ; index++)
        {
            double ego_s = trajectory.s_trajectory[index] ;
            double ego_d = trajectory.d_trajectory[index] ;

            double time = this->configuration.time_per_step * double(index) ;

            double current_vehicle_s = vehicle_s + (vehicle_vs * time) ;

            double s_distance = current_vehicle_s - ego_s ;

            // If ego and vehicle are in the same lane at given time instant
            if((are_ego_and_vehicle_in_same_lane(ego_d, vehicle_d)))
            {
                // If we are keeping lane
                if(are_ego_and_vehicle_in_same_lane(ego_initial_d, ego_final_d))
                {
                    // Only look at vehicles in front of us
                    if(vehicle_s > ego_initial_s)
                    {
                        // If vehicle is moving faster than us, allow smaller safety distance
                        if(vehicle_vs > ego_maximum_speed && std::abs(s_distance) < back_safety_s_distance)
                        {
                            cost += back_safety_s_distance / std::abs(s_distance) ;
                        }
                        else if(std::abs(s_distance) < front_safety_s_distance)
                        {
                            cost += front_safety_s_distance / std::abs(s_distance) ;
                        }

                    }
                }
                else // We are changing lanes
                {
                    // If vehicle is behind us
                    if(vehicle_s < ego_initial_s)
                    {
                        // If vehicle is slower than us, use smaller safety buffer
                        if(vehicle_vs < ego_minimum_speed && std::abs(s_distance) < back_safety_s_distance)
                        {
                            cost += back_safety_s_distance / std::abs(s_distance) ;
                        }
                        else if(std::abs(s_distance) < front_safety_s_distance)
                        {
                            cost += front_safety_s_distance / std::abs(s_distance) ;
                        }
                    }
                    else // Vehicle is in front of us
                    {

                        // If moving faster than us
                        if(vehicle_vs > ego_maximum_speed && std::abs(s_distance) < back_safety_s_distance)
                        {
                            cost += back_safety_s_distance / std::abs(s_distance) ;

                        }
                        else // Moving slower than us, use larger safety buffer
                        {
                            if(std::abs(s_distance) < front_safety_s_distance)
                            {
                                cost += front_safety_s_distance / std::abs(s_distance) ;
                            }
                        }
                    }
                }
            }

        }
        return cost ;
    }

    double get_speeding_cost(
        Trajectory &trajectory)
    {
        double cost = 0 ;

        auto s_trajectory = trajectory.s_trajectory ;

        for(int index = 1 ; index < s_trajectory.size() ; ++index)
        {
            double speed = (s_trajectory[index] - s_trajectory[index - 1]) / this->configuration.time_per_step ;

            if(speed > 0.95 * this->configuration.speed_limit)
            {
                cost +=1 ;
            }
        }

        return cost ;
    }

} ;



#endif //PATH_PLANNING_COST_H