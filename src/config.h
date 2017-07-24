#ifndef PATH_PLANNING_CONFIG_H
#define PATH_PLANNING_CONFIG_H

class Configuration
{
    public:

    double target_speed ;

    Configuration()
    {
        this->target_speed = 20.0 ;
    }
} ;

#endif //PATH_PLANNING_CONFIG_H