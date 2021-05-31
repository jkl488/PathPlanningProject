//
//  TrajectoryPlanner.cpp
//  path_planning
//
//  Created by Jakob Klein on 23.05.21.
//

#include "TrajectoryPlanner.hpp"
#include <vector>
#include <iostream>
#include "HelperFunctions.hpp"
#include "config.hpp"


void TrajectoryPlanner::init(std::shared_ptr<VehiclePose> vehicle_pose, std::shared_ptr<ObjectList> object_list , std::shared_ptr<WaypointMap> global_map,std::shared_ptr<OutputPath> output_path)
{
    vehicle_pose_ptr_ = vehicle_pose;
    object_list_ptr_ = object_list;
    global_map_ptr_ = global_map;
    output_path_ptr_ = output_path;
    quintic_time_matrix_ = Eigen::MatrixXd::Zero(3, 3);
    quintic_time_matrix_inverse_ = Eigen::MatrixXd::Zero(3, 3);
}

void TrajectoryPlanner::step()
{
    //Generate a bunch of trajectories
    double goal_s_dot = CONFIGURATION::speed_limit_m_s;
    double goal_d =  6.0;
    double planning_time = CONFIGURATION::trajectory_planning_time_total;
    double offset = 0.0;
    
    std::vector<DiscretizedTrajectory> t = GenerateVelocityKeepingInLaneTrajectory(goal_s_dot , goal_d, planning_time, offset);

    //rank them
    //choose the best
    //convert it
    //output it
    
    output_path_ptr_->planned_coordinates_cartesian.next_x_vals.clear();
    output_path_ptr_->planned_coordinates_cartesian.next_y_vals.clear();
    
    //const double planning_time = 5.0;
    //GoalCoordianteList trajectory;
    const PlanningState current_state_lat{vehicle_pose_ptr_->position_d,vehicle_pose_ptr_->position_d_dot,vehicle_pose_ptr_->position_d_dot_dot};//d,d_dot,d_dotdot,T
    const PlanningState goal_state_lat{6.0,0.0,0.0};//d,d_dot,d_dotdot,T
    QuinticPolynomial lateral_trajectory = CalculateQuinticPolynomialCoefficients(current_state_lat, goal_state_lat, 5.0);
    const PlanningState current_state_long{vehicle_pose_ptr_->position_s,vehicle_pose_ptr_->position_s_dot,vehicle_pose_ptr_->position_s_dot_dot};//s,s_dot,s_dotdot,T
    const PlanningState goal_state_long{vehicle_pose_ptr_->position_s + 30,10.0,0.0};//s,s_dot,s_dotdot,T
    QuinticPolynomial longitudinal_trajectory = CalculateQuinticPolynomialCoefficients(current_state_long, goal_state_long, 5.0);
    
    //double zero_s = GetPolynomialValue(longitudinal_trajectory, 0.0);
    //double zero_d = GetPolynomialValue(lateral_trajectory, 0.0);
    
    int num_points = CONFIGURATION::num_trajectory_points;
    double delta_t = planning_time/num_points;
    
    std::cout << "Current s: " << vehicle_pose_ptr_->position_s << ", current d: " << vehicle_pose_ptr_->position_d <<'\n';
    std::cout << ">>>>>>>>>>>>>Trajectory<<<<<<<<<<< "<< '\n';
    
    /*for(int i = 1; i <= num_points; ++i)
    {
        //double next_s = GetPolynomialValue(longitudinal_trajectory, delta_t * i);
        //double next_d = GetPolynomialValue(lateral_trajectory, delta_t * i);
        std::cout << "Next s: " << next_s << ", next d: " << next_d << " time: " << (delta_t * i) <<'\n';
        std::vector<double> xy = helpers::getXY(next_s, next_d, global_map_ptr_->map_waypoints_s, global_map_ptr_->map_waypoints_x, global_map_ptr_->map_waypoints_y);
        output_path_ptr_->planned_coordinates_cartesian.next_x_vals.push_back(xy.at(0));
        output_path_ptr_->planned_coordinates_cartesian.next_y_vals.push_back(xy.at(1));
    }*/

}

std::vector<DiscretizedTrajectory> TrajectoryPlanner::GenerateVelocityKeepingInLaneTrajectory(double goal_s_dot , double goal_d, double planning_time, double offset)
{
    
    std::vector<DiscretizedTrajectory> candidate_trajectories_longitudinal;
    std::vector<DiscretizedTrajectory> candidate_trajectories_lateral;
    
    const PlanningState current_state_lat{vehicle_pose_ptr_->position_d,vehicle_pose_ptr_->position_d_dot,vehicle_pose_ptr_->position_d_dot_dot};//d,d_dot,d_dotdot,T
    const PlanningState current_state_long{vehicle_pose_ptr_->position_s,vehicle_pose_ptr_->position_s_dot,vehicle_pose_ptr_->position_s_dot_dot};//s,s_dot,s_dotdot,T
    
    
    ////double goal_s_dot = CONFIGURATION::speed_limit_m_s;
//    double goal_d = 6.0;//TODO: later we adjust that to the lane
    

    //const double planning_time = 5.0;
    //GoalCoordianteList trajectory;
    
    std::vector<QuinticPolynomial> polynomial_candidates_lateral;
    polynomial_candidates_lateral.reserve(10);
    std::vector<QuarticPolynomial> polynomial_candidates_longitudinal;
    polynomial_candidates_lateral.reserve(10);

    
    for(int i = 0; i <= static_cast<int>(planning_time) ;++i)
    {
        double planning_end_time = (static_cast<double>(i) - offset);
        if(planning_end_time <= 0.0)
        {continue;}
        std::cout << "planning duration: "<< planning_end_time<< '\n';
        for(int j = 0; j < 3;++j)
        {
            if(j>0)
            {
                //>>>>>lateral<<<<<
                //Lateral shifted traj
                PlanningState goal_state_lat_pos_shift{(goal_d+(j*CONFIGURATION::lateral_goal_shift)),0.0,0.0};//d,d_dot,d_dotdot}
                QuinticPolynomial lateral_trajectory_pos_shift = CalculateQuinticPolynomialCoefficients(current_state_lat, goal_state_lat_pos_shift, planning_end_time);
                polynomial_candidates_lateral.push_back(lateral_trajectory_pos_shift);
                //>>>>>longitudinal<<<<<
                PlanningState goal_state_long{0.0,CONFIGURATION::speed_limit_m_s - ((j+3) *CONFIGURATION::longitudinal_velocity_goal_shift),0.0};//s,s_dot,s_dotdot
                //Generate Polynomial for longitudinal trajectory
                QuarticPolynomial longitudinal_trajectory_shorter = CalculateQuarticPolynomialCoefficients(current_state_long, goal_state_long, planning_end_time);
                polynomial_candidates_longitudinal.push_back(longitudinal_trajectory_shorter);
            }
            //>>>>>lateral<<<<<
            //Generate Polynomial for lateral trajectory
            PlanningState goal_state_lat_neg_shift{(goal_d-(j*CONFIGURATION::lateral_goal_shift)),0.0,0.0};//d,d_dot,d_dotdot
            QuinticPolynomial lateral_trajectory_neg_shift = CalculateQuinticPolynomialCoefficients(current_state_lat, goal_state_lat_neg_shift, planning_end_time);
            polynomial_candidates_lateral.push_back(lateral_trajectory_neg_shift);
            
            //>>>>>longitudinal<<<<<
            //We do not care for goal position, but for the veloctiy, since not used in solver, safe to set to 0, we aim for velocity keeping in goal, so no more acceleration
            PlanningState goal_state_long{0.0,CONFIGURATION::speed_limit_m_s - (j *CONFIGURATION::longitudinal_velocity_goal_shift),0.0};//s,s_dot,s_dotdot
            //Generate Polynomial for longitudinal trajectory
            QuarticPolynomial longitudinal_trajectory_longer = CalculateQuarticPolynomialCoefficients(current_state_long, goal_state_long, planning_end_time);
            polynomial_candidates_longitudinal.push_back(longitudinal_trajectory_longer);
            
            
            
            

            
        }
    }
    //convert to discrete and add jerk weight
    int x = 5;
    
    

    
}

std::vector<DiscretizedTrajectory> TrajectoryPlanner::GenerateFollowingTrajectory(PlanningState start_state, PlanningState final_state, double planning_time, double offset)
{}

std::vector<DiscretizedTrajectory> TrajectoryPlanner::GenerateMergeTrajectory(PlanningState start_state, PlanningState final_state, double planning_time, double offset)
{}


QuinticPolynomial TrajectoryPlanner::CalculateQuinticPolynomialCoefficients(PlanningState start_state, PlanningState final_state, double planning_time)
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
    //make sure the X.inverse() matrix is in the correct shape
    SetQuinticTimeMatrix_inverse(planning_time);
    Eigen::VectorXd lin_equation_sys_x = Eigen::VectorXd(3);
    //Solve linear equation system x = A.inverse * b
    lin_equation_sys_x = quintic_time_matrix_inverse_ * lin_equation_sys_b;
    result_polynomial.a3 = lin_equation_sys_x(0);
    result_polynomial.a4 = lin_equation_sys_x(1);
    result_polynomial.a5 = lin_equation_sys_x(2);
    return result_polynomial;
}

void TrajectoryPlanner::SetQuinticTimeMatrix_inverse(double planning_time)
{
    if(planning_time != current_quintic_planning_time_)
    {
        double& T = planning_time;
        current_quintic_planning_time_ = T;
        quintic_time_matrix_(0,0) = T * T * T;
        quintic_time_matrix_(0,1) = T * T * T * T;
        quintic_time_matrix_(0,2) = T * T * T * T * T;
        quintic_time_matrix_(1,0) = 3.0 * T * T;
        quintic_time_matrix_(1,1) = 4.0 * T * T * T;
        quintic_time_matrix_(1,2) = 5.0 * T * T * T * T;
        quintic_time_matrix_(2,0) = 6 * T;
        quintic_time_matrix_(2,1) = 12 * T * T;
        quintic_time_matrix_(2,2) = 20 * T * T * T;
        quintic_time_matrix_inverse_ = quintic_time_matrix_.inverse();
    }
}



QuarticPolynomial TrajectoryPlanner::CalculateQuarticPolynomialCoefficients(PlanningState start_state, PlanningState final_state, double planning_time)
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





