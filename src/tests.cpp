// Simplistic testing framework

#include <iostream>
#include <vector>
#include <cassert>

#include "processing.h"
#include "polyfit.h"


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


void test_polyfit()
{
    const int countOfElements = 3 ;
    const int order = 2 ;
    double xData[countOfElements] = {0, 1, 2};
    double yData[countOfElements] = {-5, 0, 9};
    double actual_coefficients[order + 1]; // resulting array of coefs

    polyfit(xData, yData, countOfElements, order, actual_coefficients);

    double expected_coefficients[order + 1] = {-5, 3, 2} ;

    for(int index = 0 ; index < order + 1 ; ++index)
    {
        assert(expected_coefficients[index] == actual_coefficients[index]) ;
    }

}

void test_polyfit_vector_input()
{
    vector<double> x_data {0, 1, 2} ;
    vector<double> y_data {-5, 0, 9} ;

    const int countOfElements = x_data.size() ;
    const int order = 2 ;
    double *xData = x_data.data() ;
    double *yData = y_data.data() ;
    double actual_coefficients[order + 1]; // resulting array of coefs

    polyfit(xData, yData, countOfElements, order, actual_coefficients);

    double expected_coefficients[order + 1] = {-5, 3, 2} ;

    for(int index = 0 ; index < order + 1 ; ++index)
    {
        assert(expected_coefficients[index] == actual_coefficients[index]) ;
    }
}


void test_get_2nd_degree_fit_coefficients()
{
    vector<double> x_data {0, 1, 2} ;
    vector<double> y_data {-5, 0, 9} ;

    vector<double> expected_coefficients {-5, 3, 2} ;
    auto actual_coefficients = get_2nd_degree_fit_coefficients(x_data, y_data) ;

    for(int index = 0 ; index < 3 ; ++index)
    {
        assert(actual_coefficients[index] == expected_coefficients[index]) ;
    }
}


int main()
{
    test_get_jerk_minimizing_trajectory_coefficients_simple() ;
    test_get_jerk_minimizing_trajectory_coefficients_medium() ;
    test_get_jerk_minimizing_trajectory_coefficients_complex() ;

    test_evaluate_polynomial_linear() ;
    test_evaluate_polynomial_square() ;

    test_polyfit() ;
    test_polyfit_vector_input() ;
    test_get_2nd_degree_fit_coefficients() ;

    std::cout << "All tests passed" << std::endl ;
}

