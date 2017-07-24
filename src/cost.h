#ifndef PATH_PLANNING_COST_H
#define PATH_PLANNING_COST_H


#include <vector>
#include "trajectory.h"

using namespace std ;

class CostComputer
{
    public:

    CostComputer() { }

    int get_lowest_cost_trajectory_index(vector<Trajectory> &trajectories)
    {
        return 0 ;
    }


} ;



#endif //PATH_PLANNING_COST_H