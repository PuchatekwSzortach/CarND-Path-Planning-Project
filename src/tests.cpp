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
    double front_safety_s_distance = 0.5 ;
    double back_safety_s_distance = 0.5 ;

    bool expected = false ;
    bool actual = will_ego_collide_with_vehicle(
        s_trajectory, d_trajectory, vehicle_s, vehicle_d, vehicle_vs, vehicle_vd,
        time_per_step, front_safety_s_distance, back_safety_s_distance) ;

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
    double front_safety_s_distance = 0.5 ;
    double back_safety_s_distance = 0.5 ;

    bool expected = true ;
    bool actual = will_ego_collide_with_vehicle(
        s_trajectory, d_trajectory, vehicle_s, vehicle_d, vehicle_vs, vehicle_vd,
        time_per_step, front_safety_s_distance, back_safety_s_distance) ;

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
    double front_safety_s_distance = 0.5 ;
    double back_safety_s_distance = 0.5 ;

    bool expected = false ;
    bool actual = will_ego_collide_with_vehicle(
        s_trajectory, d_trajectory, vehicle_s, vehicle_d, vehicle_vs, vehicle_vd,
        time_per_step, front_safety_s_distance, back_safety_s_distance) ;

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
    double front_safety_s_distance = 0.5 ;
    double back_safety_s_distance = 0.5 ;

    bool expected = false ;
    bool actual = will_ego_collide_with_vehicle(
        s_trajectory, d_trajectory, vehicle_s, vehicle_d, vehicle_vs, vehicle_vd,
        time_per_step, front_safety_s_distance, back_safety_s_distance) ;

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
    double front_safety_s_distance = 0.5 ;
    double back_safety_s_distance = 0.5 ;

    bool expected = true ;
    bool actual = will_ego_collide_with_vehicle(
        s_trajectory, d_trajectory, vehicle_s, vehicle_d, vehicle_vs, vehicle_vd,
        time_per_step, front_safety_s_distance, back_safety_s_distance) ;

    assert(expected == actual) ;
}


void test_get_arc_angle_all_in_one_line()
{
    double first_x = 0 ;
    double first_y = 0 ;

    double second_x = 1 ;
    double second_y = 0 ;

    double third_x = 2 ;
    double third_y = 0 ;

    double expected = pi() ;
    double actual = get_arc_angle(first_x, first_y, second_x, second_y, third_x, third_y) ;

    assert(std::abs(expected - actual) < 0.001) ;
}


void test_get_arc_angle_90_deg_angle()
{
    double first_x = 1 ;
    double first_y = 1 ;

    double second_x = 2 ;
    double second_y = 1 ;

    double third_x = 2 ;
    double third_y = 2 ;

    double expected = pi() / 2.0 ;
    double actual = get_arc_angle(first_x, first_y, second_x, second_y, third_x, third_y) ;

    assert(std::abs(expected - actual) < 0.001) ;
}


void test_get_arc_angle_120_deg_angle()
{
    double first_x = 1 ;
    double first_y = 1 ;

    double second_x = 2 ;
    double second_y = 1 ;

    double third_x = 3 ;
    double third_y = 2.732 ;

    double expected = 2.0 * pi() / 3.0 ;
    double actual = get_arc_angle(first_x, first_y, second_x, second_y, third_x, third_y) ;

    assert(std::abs(expected - actual) < 0.001) ;
}


void test_move_n_elements_from_end_of_first_to_beginning_of_second()
{
    vector<double> first {1, 2, 3, 4, 5} ;
    vector<double> second {6, 7, 8} ;
    int n = 3 ;

    vector<double> expected_first {1, 2} ;
    vector<double> expected_second {3, 4, 5, 6, 7, 8} ;

    move_n_elements_from_end_of_first_to_beginning_of_second(first, second, n) ;

    assert(expected_first.size() == first.size()) ;
    assert(expected_second.size() == second.size()) ;

    for(int index = 0 ; index < expected_first.size() ; ++index)
    {
        assert(expected_first[index] == first[index]) ;
    }

    for(int index = 0 ; index < expected_second.size() ; ++index)
    {
        assert(expected_second[index] == second[index]) ;
    }
}


void test_move_n_elements_from_beginning_of_first_to_end_of_second()
{
    vector<double> first {1, 2, 3, 4, 5} ;
    vector<double> second {6, 7, 8} ;
    int n = 3 ;

    vector<double> expected_first {4, 5} ;
    vector<double> expected_second {6, 7, 8, 1, 2, 3} ;

    move_n_elements_from_beginning_of_first_to_end_of_second(first, second, n) ;

    assert(expected_first.size() == first.size()) ;
    assert(expected_second.size() == second.size()) ;

    for(int index = 0 ; index < expected_first.size() ; ++index)
    {
        assert(expected_first[index] == first[index]) ;
    }

    for(int index = 0 ; index < expected_second.size() ; ++index)
    {
        assert(expected_second[index] == second[index]) ;
    }
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

    test_get_arc_angle_all_in_one_line() ;
    test_get_arc_angle_90_deg_angle() ;
    test_get_arc_angle_120_deg_angle() ;

    test_move_n_elements_from_end_of_first_to_beginning_of_second() ;
    test_move_n_elements_from_beginning_of_first_to_end_of_second() ;

    std::cout << "All tests passed" << std::endl ;
}

