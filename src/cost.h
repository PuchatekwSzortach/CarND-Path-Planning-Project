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
        vector<double> costs ;

        for(int index = 0 ; index < trajectories.size() ; ++index)
        {
//            trajectories[index].print() ;
            double cost = 0 ;

            cost += 100.0 * this->get_target_speed_cost(trajectories[index]) ;
            cost += this->get_safety_cost_two(trajectories[index]) ;
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
            double safety_d_distance = 1.0 ;

            bool are_we_keeping_lane = ((std::abs(ego_initial_d[0] - ego_final_d[0])) < safety_d_distance) ;

            // We only need to look at cars in front of us in our lane
            if(are_we_keeping_lane)
            {
                double start_s = ego_initial_s[0] ;

                // If vehicle is in our lane
                if(std::abs(vehicle_d - ego_initial_d[0]) < safety_d_distance)
                {
                    // If vehicle is between where we are now and will be at the end of trajectory
                    if(start_s < vehicle_s && vehicle_s < ego_final_s[0])
                    {
                        // And we risk passing vehicle at the end of trajectory
                        if(vehicle_final_s < ego_final_s[0] + safety_s_distance)
                        {
                            double difference = ego_final_s[0] + safety_s_distance - vehicle_final_s ;
                            cost += difference * difference ;
                        }
                    }
                }
            }
            else // We need to look at cars behind us as well as in all from current lane to destination lane
            {
                // Look at cars up to 1 second behind us
                double start_s = ego_initial_s[0] - (ego_initial_s[1] * 1.0) ;

                // And at all cars that are in lanes from our current lane to final lane
                double left_d = std::min(ego_initial_d[0], ego_final_d[0]) - safety_d_distance ;
                double right_d = std::max(ego_initial_d[0], ego_final_d[0]) + safety_d_distance ;

                // If vehicle is in one of lanes we will cross
                if(left_d < vehicle_d && vehicle_d < right_d)
                {
                    double lane_changing_safety_s_distance = 10.0 ;
                    double lane_changing_safety_d_distance = 2.0 ;

                    bool will_collide = will_ego_collide_with_vehicle(
                        trajectory.s_trajectory, trajectory.d_trajectory,
                        vehicle_s, vehicle_d, vehicle_sd_speed[0], vehicle_sd_speed[1],
                        this->configuration.time_per_step,
                        lane_changing_safety_s_distance, lane_changing_safety_d_distance) ;

                    if(will_collide)
                    {
                        // Arbitrary high collision cost
                        cost += 10000 ;
                    }


                    /*// If vehicle is between where we are now and will be at the end of trajectory
                    // This is simplistic - after all by end of trajectory we might be in different lane than
                    // vehicle
                    if(start_s < vehicle_s && vehicle_s < ego_final_s[0])
                    {
                        // And we risk passing vehicle at the end of trajectory
                        if(vehicle_final_s < ego_final_s[0] + safety_s_distance)
                        {
                            double difference = ego_final_s[0] + safety_s_distance - vehicle_final_s ;
                            cost += difference * difference ;
                        }
                    }*/

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