//
//  PolynomialSolver.cpp
//  path_planning
//
//  Created by Jakob Klein on 01.06.21.
//

#include "PolynomialSolver.hpp"


QuinticPolynomial PolynomialSolver::CalculateQuinticPolynomialCoefficients(PlanningState start_state, PlanningState final_state, double planning_time)
{
    
    QuinticPolynomial result_polynomial;
    result_polynomial.a0 = start_state.position;
    result_polynomial.a1 = start_state.position_dot;
    result_polynomial.a2 = 0.5 * start_state.position_dot_dot;
    //some temporary variables to make solving the equation more easy
    double temp_C1 = start_state.position + start_state.position_dot * planning_time + 0.5 * start_state.position_dot_dot * planning_time * planning_time;
    double temp_C2 = start_state.position_dot + 0.5 * start_state.position_dot_dot * planning_time * planning_time;
    double temp_C3 = start_state.position_dot_dot;
    //Create linear quation system of form Ax = b
    Eigen::VectorXd lin_equation_sys_b = Eigen::VectorXd(3);
    lin_equation_sys_b << (final_state.position - temp_C1), (final_state.position_dot - temp_C2), (final_state.position_dot_dot - temp_C3);
    
    Eigen::MatrixXd quintic_time_matrix = Eigen::MatrixXd(3,3);
    double& T = planning_time;
    quintic_time_matrix(0,0) = T * T * T;
    quintic_time_matrix(0,1) = T * T * T * T;
    quintic_time_matrix(0,2) = T * T * T * T * T;
    quintic_time_matrix(1,0) = 3.0 * T * T;
    quintic_time_matrix(1,1) = 4.0 * T * T * T;
    quintic_time_matrix(1,2) = 5.0 * T * T * T * T;
    quintic_time_matrix(2,0) = 6.0 * T;
    quintic_time_matrix(2,1) = 12.0 * T * T;
    quintic_time_matrix(2,2) = 20.0 * T * T * T;

    Eigen::VectorXd lin_equation_sys_x = Eigen::VectorXd(3);
    //Solve linear equation system x = A.inverse * b
    lin_equation_sys_x = quintic_time_matrix.inverse() * lin_equation_sys_b;
    result_polynomial.a3 = lin_equation_sys_x(0);
    result_polynomial.a4 = lin_equation_sys_x(1);
    result_polynomial.a5 = lin_equation_sys_x(2);
    return result_polynomial;
}


QuarticPolynomial PolynomialSolver::CalculateQuarticPolynomialCoefficients(PlanningState start_state, PlanningState final_state, double planning_time)
{
    QuarticPolynomial result_polynomial;
    result_polynomial.a0 = start_state.position;
    result_polynomial.a1 = start_state.position_dot;
    result_polynomial.a2 = start_state.position_dot_dot;
    
    double& T = planning_time;
    
    //set up Ax + b and solve
    Eigen::MatrixXd time_matrix = Eigen::MatrixXd(2,2);
    time_matrix << 3*T*T, 4*T*T*T*T,
    6*T, 12*T*T;
    
    Eigen::VectorXd b = Eigen::VectorXd(2);
    b << final_state.position_dot - result_polynomial.a1 - 2*result_polynomial.a2*T ,
    final_state.position_dot_dot - 2 * result_polynomial.a2;
    
    
    Eigen::VectorXd x = Eigen::VectorXd(2);
    x = time_matrix.inverse() * b;
    
    result_polynomial.a3 = b(0);
    result_polynomial.a4 = b(1);
    return  result_polynomial;
}
