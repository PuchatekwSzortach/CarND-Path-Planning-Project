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


void test_get_previous_trajectory_final_speed_horizontal_motion_only()
{
    vector<double> x_trajectory {1.0, 5.0} ;
    vector<double> y_trajectory {1.0, 1.0} ;
    double time_between_steps = 0.2 ;

    double expected = 20.0 ;
    double actual = get_previous_trajectory_final_speed(x_trajectory, y_trajectory, time_between_steps) ;

    assert(expected == actual) ;
}

void test_get_previous_trajectory_final_speed_vertical_motion_only()
{
    vector<double> x_trajectory {1.0, 1.0} ;
    vector<double> y_trajectory {1.0, 7.0} ;
    double time_between_steps = 0.4 ;

    double expected = 15.0 ;
    double actual = get_previous_trajectory_final_speed(x_trajectory, y_trajectory, time_between_steps) ;

    assert(expected == actual) ;
}


void test_get_previous_trajectory_final_speed_generic_motion()
{
    vector<double> x_trajectory {1.0, 5.0} ;
    vector<double> y_trajectory {1.0, 3.0} ;
    double time_between_steps = 0.5 ;

    double expected = 8.94427 ;
    double actual = get_previous_trajectory_final_speed(x_trajectory, y_trajectory, time_between_steps) ;

    assert(std::abs(expected - actual) < 0.01) ;
}


void test_get_previous_trajectory_final_acceleration_horizontal_motion()
{
    vector<double> x_trajectory {1.0, 6.0, 16.0} ;
    vector<double> y_trajectory {1.0, 1.0, 1.0} ;
    double time_between_steps = 0.2 ;

    double expected = 125.0 ;
    double actual = get_previous_trajectory_final_acceleration(x_trajectory, y_trajectory, time_between_steps) ;

    assert(std::abs(expected - actual) < 0.01) ;
}


void test_get_previous_trajectory_final_acceleration_vertical_motion()
{
    vector<double> x_trajectory {2.0, 2.0, 2.0} ;
    vector<double> y_trajectory {4.0, 2.0, -4.0} ;
    double time_between_steps = 0.5 ;

    double expected = -16.0 ;
    double actual = get_previous_trajectory_final_acceleration(x_trajectory, y_trajectory, time_between_steps) ;

    assert(std::abs(expected - actual) < 0.01) ;
}


void test_get_previous_trajectory_final_acceleration_generic_motion()
{
    vector<double> x_trajectory {2.0, 8.0, 10.0} ;
    vector<double> y_trajectory {4.0, 6.0, 16.0} ;
    double time_between_steps = 0.5 ;

    double expected = 15.492 ;
    double actual = get_previous_trajectory_final_acceleration(x_trajectory, y_trajectory, time_between_steps) ;

    assert(std::abs(expected - actual) < 0.01) ;
}


int main()
{
    test_get_jerk_minimizing_trajectory_coefficients_simple() ;
    test_get_jerk_minimizing_trajectory_coefficients_medium() ;
    test_get_jerk_minimizing_trajectory_coefficients_complex() ;

    test_evaluate_polynomial_linear() ;
    test_evaluate_polynomial_square() ;

    test_get_previous_trajectory_final_speed_horizontal_motion_only() ;
    test_get_previous_trajectory_final_speed_vertical_motion_only() ;
    test_get_previous_trajectory_final_speed_generic_motion() ;

    test_get_previous_trajectory_final_acceleration_horizontal_motion() ;

    std::cout << "All tests passed" << std::endl ;
}

