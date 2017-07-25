#ifndef PATH_PLANNING_CONFIG_H
#define PATH_PLANNING_CONFIG_H

class Configuration
{
    public:

    double target_speed ;
    double speed_limit ;

    double tangential_acceleration_limit ;
    double normal_acceleration_limit ;

    double trajectory_update_interval ;
    double trajectory_time ;
    double time_per_step ;

    Configuration()
    {
        this->target_speed = 20.0 ;
        this->speed_limit = 22.0 ;

        this->tangential_acceleration_limit = 5.0 ;
        this->normal_acceleration_limit = 5.0 ;

        this->trajectory_update_interval = 1.0 ;
        this->trajectory_time = 4.0 ;
        this->time_per_step = 0.02 ;
    }
} ;

#endif //PATH_PLANNING_CONFIG_H