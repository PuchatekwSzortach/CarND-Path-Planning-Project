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


int main()
{
    test_get_jerk_minimizing_trajectory_coefficients_simple() ;
    test_get_jerk_minimizing_trajectory_coefficients_medium() ;
    test_get_jerk_minimizing_trajectory_coefficients_complex() ;

    test_evaluate_polynomial_linear() ;
    test_evaluate_polynomial_square() ;

    test_get_arg_min() ;

    std::cout << "All tests passed" << std::endl ;
}

