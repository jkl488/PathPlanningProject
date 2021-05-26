//
//  TrajectoryPlanner.cpp
//  path_planning
//
//  Created by Jakob Klein on 23.05.21.
//

#include "TrajectoryPlanner.hpp"
#include <vector>
#include <iostream>

void TrajectoryPlanner::init(std::shared_ptr<VehicleState> vehicle_state, std::shared_ptr<ObjectList> object_list , std::shared_ptr<WaypointMap> global_map,std::shared_ptr<OutputPath> output_path)
{
    vehicle_state_ = vehicle_state;
    object_list_ = object_list;
    global_map_ = global_map;
    output_path_ = output_path;
}

void TrajectoryPlanner::step()
{
    GoalCoordianteList trajectory;
    /*FifthOrderPolynomial polynomial_trajectory = CalculateFithOrderPolynomialCoefficients(current_state, final_state; 5.0);*/
    
    for(int i = 0; i < object_list_->objects.size(); ++i)
    {
        std::cout << "Traj Planner" << '\n';
        std::cout << "ID: " << object_list_->objects.at(i).id << '\n';
        std::cout << "x: " << object_list_->objects.at(i).position_x_world << '\n';
        std::cout << "y: " << object_list_->objects.at(i).position_y_world << '\n';
        std::cout << "vx: " << object_list_->objects.at(i).velocity_x_world << '\n';
        std::cout << "vy: " << object_list_->objects.at(i).velocity_y_world << '\n';
        std::cout << "s: " << object_list_->objects.at(i).position_s << '\n';
        std::cout << "d: " << object_list_->objects.at(i).position_d << '\n';
        std::cout << "lane_assignment: " << object_list_->objects.at(i).lane_assignment << '\n';
    }
    


}

FifthOrderPolynomial TrajectoryPlanner::CalculateFithOrderPolynomialCoefficients(PlanningState start_state, PlanningState final_state, double planning_time)
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
