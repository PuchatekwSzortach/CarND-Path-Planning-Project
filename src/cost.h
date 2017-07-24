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

            cost += this->get_safety_cost(trajectories[index]) ;
            cost += this->get_target_speed_cost(trajectories[index]) ;

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
        return 0 ;
    }


} ;



#endif //PATH_PLANNING_COST_H