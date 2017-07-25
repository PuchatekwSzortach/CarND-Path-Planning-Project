// Simplistic testing framework

#include <iostream>
#include <vector>
#include <cassert>

#include "processing.h"


using namespace std ;

void test_get_jerk_minimizing_trajectory_coefficients_simple()
{
    vector<double> initial_state = {0, 10, 0} ;
    vector<double> final_state = {10, 10, 0} ;
    double time = 1 ;

    vector<double> expected {0.0, 10.0, 0.0, 0.0, 0.0, 0.0} ;

    vector<double> actual = get_jerk_minimizing_trajectory_coefficients(
        initial_state, final_state, time) ;

    assert(expected.size() == actual.size()) ;

    for(int index = 0 ; index < expected.size() ; ++index)
    {
        assert(std::abs(expected[index] - actual[index]) < 0.01) ;
    }

}


void test_get_jerk_minimizing_trajectory_coefficients_medium()
{
    vector<double> initial_state = {0.0, 10.0, 0.0} ;
    vector<double> final_state = {20.0, 15.0, 20.0} ;
    double time = 2.0 ;

    vector<double> expected {0.0, 10.0, 0.0, 0.0, -0.625, 0.3125} ;

    vector<double> actual = get_jerk_minimizing_trajectory_coefficients(
        initial_state, final_state, time) ;

    assert(expected.size() == actual.size()) ;

    for(int index = 0 ; index < expected.size() ; ++index)
    {
        assert(std::abs(expected[index] - actual[index]) < 0.01) ;
    }

}


void test_get_jerk_minimizing_trajectory_coefficients_complex()
{
    vector<double> initial_state = {5.0, 10.0, 2.0} ;
    vector<double> final_state = {-30.0, -20.0, -4.0} ;
    double time = 5.0 ;

    vector<double> expected {5.0, 10.0, 1.0, -3.0,0.64,-0.0432} ;

    vector<double> actual = get_jerk_minimizing_trajectory_coefficients(
        initial_state, final_state, time) ;

    assert(expected.size() == actual.size()) ;

    for(int index = 0 ; index < expected.size() ; ++index)
    {
        assert(std::abs(expected[index] - actual[index]) < 0.01) ;
    }

}


void test_evaluate_polynomial_linear()
{
    vector<double> coefficients {1.0, 2.5} ;
    assert(6.0 == evaluate_polynomial(coefficients, 2.0)) ;
}


void test_evaluate_polynomial_square()
{
    vector<double> coefficients {1.0, 2.5, -0.5} ;
    assert(-11.0 == evaluate_polynomial(coefficients, -3.0)) ;
}

void test_get_arg_min()
{
    vector<double> values {0.8, 0.2, 0.7} ;

    int expected = 1 ;
    int actual = get_arg_min(values) ;

    assert(expected == actual) ;
}


void test_will_ego_collide_with_vehicle_different_lanes()
{
    vector<double> s_trajectory {0, 1, 2, 3} ;
    vector<double> d_trajectory {0, 0, 0, 0} ;

    double vehicle_s = 1.0 ;
    double vehicle_d = 6.0 ;
    double vehicle_vs = 0.5 ;
    double vehicle_vd = 0.0 ;

    double time_per_step = 1.0 ;
    double safety_s_distance = 0.5 ;
    double safety_d_distance = 0.5 ;

    bool expected = false ;
    bool actual = will_ego_collide_with_vehicle(
        s_trajectory, d_trajectory, vehicle_s, vehicle_d, vehicle_vs, vehicle_vd,
        time_per_step, safety_s_distance, safety_d_distance) ;

    assert(expected == actual) ;

}

void test_will_ego_collide_with_vehicle_same_lane_collision_speed()
{
    vector<double> s_trajectory {0, 1, 2, 3} ;
    vector<double> d_trajectory {0, 0, 0, 0} ;

    double vehicle_s = 1.0 ;
    double vehicle_d = 0 ;
    double vehicle_vs = 0.5 ;
    double vehicle_vd = 0.0 ;

    double time_per_step = 1.0 ;
    double safety_s_distance = 0.5 ;
    double safety_d_distance = 0.5 ;

    bool expected = true ;
    bool actual = will_ego_collide_with_vehicle(
        s_trajectory, d_trajectory, vehicle_s, vehicle_d, vehicle_vs, vehicle_vd,
        time_per_step, safety_s_distance, safety_d_distance) ;

    assert(expected == actual) ;

}


void test_will_ego_collide_with_vehicle_same_lane_safe_speed()
{
    vector<double> s_trajectory {0, 1, 2, 3} ;
    vector<double> d_trajectory {0, 0, 0, 0} ;

    double vehicle_s = 1.0 ;
    double vehicle_d = 0 ;
    double vehicle_vs = 2.0 ;
    double vehicle_vd = 0.0 ;

    double time_per_step = 1.0 ;
    double safety_s_distance = 0.5 ;
    double safety_d_distance = 0.5 ;

    bool expected = false ;
    bool actual = will_ego_collide_with_vehicle(
        s_trajectory, d_trajectory, vehicle_s, vehicle_d, vehicle_vs, vehicle_vd,
        time_per_step, safety_s_distance, safety_d_distance) ;

    assert(expected == actual) ;

}


void test_will_ego_collide_with_vehicle_crossing_lanes_safe_speed()
{
    vector<double> s_trajectory {0, 1, 2, 3} ;
    vector<double> d_trajectory {0, 0, 2, 2} ;

    double vehicle_s = 1.0 ;
    double vehicle_d = 2.0 ;
    double vehicle_vs = 2.0 ;
    double vehicle_vd = 0.0 ;

    double time_per_step = 1.0 ;
    double safety_s_distance = 0.5 ;
    double safety_d_distance = 0.5 ;

    bool expected = false ;
    bool actual = will_ego_collide_with_vehicle(
        s_trajectory, d_trajectory, vehicle_s, vehicle_d, vehicle_vs, vehicle_vd,
        time_per_step, safety_s_distance, safety_d_distance) ;

    assert(expected == actual) ;

}


void test_will_ego_collide_with_vehicle_crossing_lanes_collision_speed()
{
    vector<double> s_trajectory {0, 1, 2, 3} ;
    vector<double> d_trajectory {0, 0, 2, 2} ;

    double vehicle_s = 1.0 ;
    double vehicle_d = 2.0 ;
    double vehicle_vs = 0.5 ;
    double vehicle_vd = 0.0 ;

    double time_per_step = 1.0 ;
    double safety_s_distance = 0.5 ;
    double safety_d_distance = 0.5 ;

    bool expected = true ;
    bool actual = will_ego_collide_with_vehicle(
        s_trajectory, d_trajectory, vehicle_s, vehicle_d, vehicle_vs, vehicle_vd,
        time_per_step, safety_s_distance, safety_d_distance) ;

    assert(expected == actual) ;
}


int main()
{
    test_get_jerk_minimizing_trajectory_coefficients_simple() ;
    test_get_jerk_minimizing_trajectory_coefficients_medium() ;
    test_get_jerk_minimizing_trajectory_coefficients_complex() ;

    test_evaluate_polynomial_linear() ;
    test_evaluate_polynomial_square() ;

    test_get_arg_min() ;

    test_will_ego_collide_with_vehicle_different_lanes() ;
    test_will_ego_collide_with_vehicle_same_lane_collision_speed() ;
    test_will_ego_collide_with_vehicle_same_lane_safe_speed() ;
    test_will_ego_collide_with_vehicle_crossing_lanes_safe_speed() ;
    test_will_ego_collide_with_vehicle_crossing_lanes_collision_speed() ;

    std::cout << "All tests passed" << std::endl ;
}

