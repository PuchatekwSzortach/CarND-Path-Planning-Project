#ifndef PATH_PLANNING_SENSOR_H
#define PATH_PLANNING_SENSOR_H

// Module handling sensor data

#include <map>
#include <vector>
#include <cmath>

using namespace std ;

class SensorDataHandler
{

    public:

    std::vector<std::vector<double>> data ;
    double car_s ;
    double car_d ;

    SensorDataHandler(std::vector<std::vector<double>> data, double car_s, double car_d)
    {
        this->data = data ;

        this->car_s = car_s ;
        this->car_d = car_d ;

    }

    void sense()
    {
        for(auto vehicle_data : data)
        {
            double vehicle_id = vehicle_data[0] ;
            double vehicle_s = vehicle_data[5] ;
            double vehicle_d = vehicle_data[6] ;

            if(vehicle_s > this->car_s && vehicle_s - this->car_s < 15.0 )
            {
                if(std::abs(this->car_d - vehicle_d) < 1.0)
                {
                    std::cout << "Vehicle " << vehicle_id << " is "
                        << vehicle_s - this->car_s << " metres in front of us" << std::endl ;
                }


            }

        }
    }



} ;

#endif //PATH_PLANNING_SENSOR_H