#ifndef PATH_PLANNING_SENSOR_H
#define PATH_PLANNING_SENSOR_H

// Module handling sensor data

#include <map>
#include <vector>
#include <cmath>

#include "processing.h"

using namespace std ;

class SensorDataHandler
{

    public:

    std::vector<std::vector<double>> data ;
    double car_s ;
    double car_d ;

    vector<double> maps_x ;
    vector<double> maps_y ;
    vector<double> maps_s ;
    vector<double> maps_dx ;
    vector<double> maps_dy ;

    SensorDataHandler(std::vector<std::vector<double>> data, double car_s, double car_d,
        vector<double> &maps_x, vector<double> &maps_y, vector<double> &maps_s,
        vector<double> &maps_dx, vector<double> &maps_dy)
    {
        this->data = data ;

        this->car_s = car_s ;
        this->car_d = car_d ;

        this->maps_x = maps_x ;
        this->maps_y = maps_y ;
        this->maps_s = maps_s ;

        this->maps_dx = maps_dx ;
        this->maps_dy = maps_dy ;

    }


    void is_vehicle_in_front_of_us()
    {
        for(auto vehicle_data : data)
        {
            double vehicle_id = vehicle_data[0] ;
            double vehicle_s = vehicle_data[5] ;
            double vehicle_d = vehicle_data[6] ;

            if(vehicle_s > this->car_s && vehicle_s - this->car_s < 40.0 )
            {
                if(std::abs(this->car_d - vehicle_d) < 1.5)
                {
                    std::cout << "Vehicle " << vehicle_id << " is "
                        << vehicle_s - this->car_s << " metres in front of us" << std::endl ;
                }

            }
        }

    }

    void sense_vehicles_sd_speeds()
    {
        for(auto vehicle_data : data)
        {
            double vehicle_id = vehicle_data[0] ;
            double vehicle_s = vehicle_data[5] ;
            double vehicle_d = vehicle_data[6] ;

            if(vehicle_s > this->car_s && vehicle_s - this->car_s < 40.0 )
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

                double ms_to_mph = 2.237 ;

                std::cout << vehicle_id << ": " << ms_to_mph * vehicle_sd_speed[0] << " and " << ms_to_mph * vehicle_sd_speed[1] << std::endl ;
            }

        }
    }



} ;

#endif //PATH_PLANNING_SENSOR_H