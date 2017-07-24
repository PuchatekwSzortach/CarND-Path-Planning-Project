#ifndef PATH_PLANNING_COST_H
#define PATH_PLANNING_COST_H


#include <vector>
#include "trajectory.h"

using namespace std ;

class CostComputer
{
    public:

    std::vector<std::vector<double>> sensory_data ;

    CostComputer(std::vector<std::vector<double>> sensory_data)
    {
        this->sensory_data = sensory_data ;
    }

    int get_lowest_cost_trajectory_index(vector<Trajectory> &trajectories)
    {
        vector<double> costs ;

        for(auto trajectory: trajectories)
        {
            double cost = 0 ;

            cost += this->get_safety_cost(trajectory) ;
            costs.push_back(cost) ;
        }

        return get_arg_min(costs) ;
    }

    double get_safety_cost(Trajectory &trajectory)
    {
        double final_speed = trajectory.final_s_state[1] ;
        return final_speed ;
    }




} ;



#endif //PATH_PLANNING_COST_H