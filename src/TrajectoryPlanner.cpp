//
//  TrajectoryPlanner.cpp
//  path_planning
//
//  Created by Jakob Klein on 23.05.21.
//

#include "TrajectoryPlanner.hpp"
#include <vector>

void TrajectoryPlanner::step()
{
    /*std::vector<double> next_x_vals;
    std::vector<double> next_y_vals;
    double dist_inc = 0.5;
    for (int i = 0; i < 50; ++i) {
      next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
      next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
    }
    double pos_x;
    double pos_y;
    double angle;
    int path_size = previous_path_x.size();

    for (int i = 0; i < path_size; ++i) {
      next_x_vals.push_back(previous_path_x[i]);
      next_y_vals.push_back(previous_path_y[i]);
    }

    if (path_size == 0) {
      pos_x = car_x;
      pos_y = car_y;
      angle = deg2rad(car_yaw);
    } else {
      pos_x = previous_path_x[path_size-1];
      pos_y = previous_path_y[path_size-1];

      double pos_x2 = previous_path_x[path_size-2];
      double pos_y2 = previous_path_y[path_size-2];
      angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
    }

    double dist_inc = 0.5;
    for (int i = 0; i < 50-path_size; ++i) {
      next_x_vals.push_back(pos_x+(dist_inc)*cos(angle+(i+1)*(pi()/100)));
      next_y_vals.push_back(pos_y+(dist_inc)*sin(angle+(i+1)*(pi()/100)));
      pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
      pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
    }*/
}

FifthOrderPolynomial TrajectoryPlanner::CalculateFithOrderPolynomialCoefficients(PlanningState start_state, PlanningState final_state, int planning_time)
{
    
    FifthOrderPolynomial result_polynomial;
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
    //make sure the X.inverse() matrix is in the correct shape
    SetTimeMatrix_inverse(planning_time);
    Eigen::VectorXd lin_equation_sys_x = Eigen::VectorXd(3);
    //Solve linear equation system x = A.inverse * b
    lin_equation_sys_x = time_matrix_inverse_ * lin_equation_sys_b;
    
}

void TrajectoryPlanner::SetTimeMatrix_inverse(int planning_time)
{
    if(planning_time != current_planning_time_)
    {
        int& T = planning_time;
        current_planning_time_ = T;
        time_matrix_(0,0) = T * T * T;
        time_matrix_(0,1) = T * T * T * T;
        time_matrix_(0,2) = T * T * T * T * T;
        time_matrix_(1,0) = 3.0 * T * T;
        time_matrix_(1,1) = 4.0 * T * T * T;
        time_matrix_(1,2) = 5.0 * T * T * T * T;
        time_matrix_(2,0) = 6 * T;
        time_matrix_(2,1) = 12 * T * T;
        time_matrix_(2,2) = 20 * T * T * T;
        time_matrix_inverse_ = time_matrix_.inverse();
    }
}
