#ifndef PATH_PLANNING_COST_H
#define PATH_PLANNING_COST_H


#include <vector>
#include "trajectory.h"
#include "config.h"

using namespace std ;

class CostComputer
{
    public:

    std::vector<std::vector<double>> sensory_data ;
    Configuration configuration ;

    CostComputer(Configuration configuration, std::vector<std::vector<double>> sensory_data)
    {
        this->configuration = configuration ;
        this->sensory_data = sensory_data ;
    }

    int get_lowest_cost_trajectory_index(vector<Trajectory> &trajectories)
    {
        vector<double> costs ;

        for(int index = 0 ; index < trajectories.size() ; ++index)
        {
            double cost = 0 ;

            cost += this->get_target_speed_cost(trajectories[index]) ;
            cost += this->get_safety_cost(trajectories[index]) ;

            costs.push_back(cost) ;

            std::cout << "Trajectory " << index << " has cost " << cost << std::endl ;
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
            if(std::abs(trajectory.final_d_state[0] - vehicle_d) < 1.5)
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


} ;



#endif //PATH_PLANNING_COST_H