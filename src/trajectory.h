#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H


#include <vector>
#include "processing.h"

using namespace std ;

class TrajectoryPlanner
{

    public:

    vector<double> saved_x_trajectory ;
    vector<double> saved_y_trajectory ;
    vector<double> saved_s_trajectory ;
    vector<double> saved_d_trajectory ;



    TrajectoryPlanner()
    {
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
    void get_trajectory_based_on_previous_trajectory(
        double car_x, double car_y, double car_s, double car_d,
        vector<double> previous_trajectory_x, vector<double> previous_trajectory_y)
    {

        // Get index of closest point in saved trajectory to current car position
        double index = this->get_index_of_closest_saved_trajectory_point(car_x, car_y) ;

        std::cout << "Car is at " << car_x << ", " << car_y << std::endl ;
        std::cout << "Matched with " << this->saved_x_trajectory[index] << ", " << this->saved_y_trajectory[index] << std::endl ;


    }



} ;


#endif //PATH_PLANNING_TRAJECTORY_H