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

    vector<double> expected {5.0, 10.0, 2.0, -3.0,0.64,-0.0432} ;

    vector<double> actual = get_jerk_minimizing_trajectory_coefficients(
        initial_state, final_state, time) ;

    assert(expected.size() == actual.size()) ;

    for(int index = 0 ; index < expected.size() ; ++index)
    {
        assert(std::abs(expected[index] - actual[index]) < 0.01) ;
    }

}

int main()
{
    test_get_jerk_minimizing_trajectory_coefficients_simple() ;
    test_get_jerk_minimizing_trajectory_coefficients_medium() ;
    test_get_jerk_minimizing_trajectory_coefficients_complex() ;

    std::cout << "All tests passed" << std::endl ;

}

