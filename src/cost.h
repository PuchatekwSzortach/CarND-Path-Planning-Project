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

    CostComputer(
        Configuration configuration, std::vector<std::vector<double>> sensory_data,
        vector<double> &maps_x, vector<double> &maps_y, vector<double> &maps_s,
        vector<double> &maps_dx, vector<double> &maps_dy)
    {
        this->configuration = configuration ;
        this->sensory_data = sensory_data ;

        this->maps_x = maps_x ;
        this->maps_y = maps_y ;
        this->maps_s = maps_s ;

        this->maps_dx = maps_dx ;
        this->maps_dy = maps_dy ;
    }

    int get_lowest_cost_trajectory_index(vector<Trajectory> &trajectories)
    {
        std::cout << "\n\n" << std::endl ;
        vector<double> costs ;

        for(int index = 0 ; index < trajectories.size() ; ++index)
        {
            trajectories[index].print() ;
            double cost = 0 ;

            cost += 100.0 * this->get_target_speed_cost(trajectories[index]) ;

            double safety_cost = this->get_safety_cost_two(trajectories[index]) ;
            std:cout << "Safety cost is " << safety_cost << std::endl ;
            cost += safety_cost ;

            cost += this->get_lane_change_cost(trajectories[index]) ;

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
        double cost = 0 ;

        for(auto vehicle_data: sensory_data)
        {
            double vehicle_d = vehicle_data[6] ;

            // If vehicle is in the same lane - a very simplistic check for now
            if(std::abs(trajectory.final_d_state[0] - vehicle_d) < 2.0)
            {
                double start_s = trajectory.initial_s_state[0] ;
                double finish_s = trajectory.final_s_state[0] ;

                double vehicle_start_s = vehicle_data[5] ;

                // Simplified computations for now
                if(start_s < vehicle_start_s && vehicle_start_s < finish_s)
                {
                    double difference = finish_s - vehicle_start_s ;
                    cost += difference * difference ;

                }
            }

        }

        return cost ;
    }

    double get_safety_cost_two(Trajectory &trajectory)
    {
        double cost = 0 ;

        auto ego_initial_s = trajectory.initial_s_state ;
        auto ego_final_s = trajectory.final_s_state ;

        auto ego_initial_d = trajectory.initial_d_state ;
        auto ego_final_d = trajectory.final_d_state ;

        // Establish d-band we will be inside during this trajectory
        double ego_d_left = std::min(trajectory.initial_d_state[0], trajectory.final_d_state[0]) - 1.0 ;
        double ego_d_right = std::max(trajectory.initial_d_state[0], trajectory.final_d_state[0]) + 1.0 ;

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

            double vehicle_final_s = vehicle_s + (vehicle_sd_speed[0] * this->configuration.trajectory_time) ;
            double vehicle_final_d = vehicle_d + (vehicle_sd_speed[1] * this->configuration.trajectory_time) ;

            double vehicle_d_left = std::min(vehicle_d, vehicle_final_d) ;
            double vehicle_d_right = std::max(vehicle_d, vehicle_final_d) ;

            double safety_s_distance = 20.0 ;

            if(ego_initial_s[0] < vehicle_s && vehicle_s < ego_final_s[0])
            {

                if(ego_d_left < vehicle_d_left && vehicle_d_right < ego_d_right)
                {
                    std::cout << "\tVehicle " << vehicle_id << " is " << vehicle_s - ego_initial_s[0] << " metres in front of us" << std::endl ;
                    std::cout << "\tIts speed is " << vehicle_sd_speed[0] << " and " << vehicle_sd_speed[1] << std::endl ;
                    std::cout << "\tOur speed is " << ego_initial_s[1] << " and " << ego_initial_d[1] << std::endl ;
                    std::cout << "\tTime horizon is: " << this->configuration.trajectory_time << std::endl ;
                    std::cout << "\tOur final s is " << ego_final_s[0] + safety_s_distance << ", its " << vehicle_final_s << std::endl ;

                    if(vehicle_final_s < ego_final_s[0] + safety_s_distance)
                    {
                        std::cout << "\tIt is in our s range" << std::endl ;
                        std::cout << "\tAnd it is in our band too" << std::endl ;
                        double difference = ego_final_s[0] + safety_s_distance - vehicle_final_s ;
                        cost += difference * difference ;

                    }

                }
            }

        }
        return cost ;
    }

    double get_lane_change_cost(Trajectory &trajectory)
    {
        return std::abs(trajectory.final_d_state[0] - trajectory.initial_d_state[0]) ;
    }


} ;



#endif //PATH_PLANNING_COST_H