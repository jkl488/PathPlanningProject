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
#include "PolynomialSolver.hpp"
#include "TrajectoryConverter.hpp"


void TrajectoryPlanner::init(std::shared_ptr<VehiclePose> vehicle_pose, std::shared_ptr<ObjectList> object_list , std::shared_ptr<WaypointMap> global_map,std::shared_ptr<OutputPath> output_path)
{
    vehicle_pose_ptr_ = vehicle_pose;
    object_list_ptr_ = object_list;
    global_map_ptr_ = global_map;
    output_path_ptr_ = output_path;
}

void TrajectoryPlanner::step()
{
    //Update the timestep
    IncrementPlanningTimeStep();
    
    //Generate a bunch of trajectories
    double goal_s_dot = CONFIGURATION::speed_limit_m_s;
    double goal_d =  6.0;
    double planning_time = CONFIGURATION::trajectory_planning_time_total;
    //TODO: maybe we need to take number of executed elements into account
    double offset = (current_planning_timestep_ )*CONFIGURATION::delta_t_trajectory_points;
    
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
    QuinticPolynomial lateral_trajectory = PolynomialSolver::CalculateQuinticPolynomialCoefficients(current_state_lat, goal_state_lat, 5.0);
    const PlanningState current_state_long{vehicle_pose_ptr_->position_s,vehicle_pose_ptr_->position_s_dot,vehicle_pose_ptr_->position_s_dot_dot};//s,s_dot,s_dotdot,T
    const PlanningState goal_state_long{vehicle_pose_ptr_->position_s + 30,10.0,0.0};//s,s_dot,s_dotdot,T
    QuinticPolynomial longitudinal_trajectory = PolynomialSolver::CalculateQuinticPolynomialCoefficients(current_state_long, goal_state_long, 5.0);
    
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
    
    std::vector<QuinticPolynomialTrajectory> polynomial_candidates_lateral;
    polynomial_candidates_lateral.reserve(30);
    std::vector<QuarticPolynomialTrajectory> polynomial_candidates_longitudinal;
    polynomial_candidates_lateral.reserve(30);

    
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
                QuinticPolynomialTrajectory lateral_trajectory_pos_shift;
                lateral_trajectory_pos_shift.max_planning_time = planning_end_time;
                lateral_trajectory_pos_shift.polynomial = PolynomialSolver::CalculateQuinticPolynomialCoefficients(current_state_lat, goal_state_lat_pos_shift, planning_end_time);
                polynomial_candidates_lateral.push_back(lateral_trajectory_pos_shift);
                //>>>>>longitudinal<<<<<
                PlanningState goal_state_long{0.0,CONFIGURATION::speed_limit_m_s - ((j+3) *CONFIGURATION::longitudinal_velocity_goal_shift),0.0};//s,s_dot,s_dotdot
                //Generate Polynomial for longitudinal trajectory
                QuarticPolynomialTrajectory longitudinal_trajectory_shorter;
                longitudinal_trajectory_shorter.max_planning_time = planning_end_time;
                longitudinal_trajectory_shorter.polynomial = PolynomialSolver::CalculateQuarticPolynomialCoefficients(current_state_long, goal_state_long, planning_end_time);
                polynomial_candidates_longitudinal.push_back(longitudinal_trajectory_shorter);
            }
            //>>>>>lateral<<<<<
            //Generate Polynomial for lateral trajectory
            PlanningState goal_state_lat_neg_shift{(goal_d-(j*CONFIGURATION::lateral_goal_shift)),0.0,0.0};//d,d_dot,d_dotdot
            QuinticPolynomialTrajectory lateral_trajectory_neg_shift;
            lateral_trajectory_neg_shift.max_planning_time = planning_end_time;
            lateral_trajectory_neg_shift.polynomial = PolynomialSolver::CalculateQuinticPolynomialCoefficients(current_state_lat, goal_state_lat_neg_shift, planning_end_time);
            polynomial_candidates_lateral.push_back(lateral_trajectory_neg_shift);
            
            //>>>>>longitudinal<<<<<
            //We do not care for goal position, but for the veloctiy, since not used in solver, safe to set to 0, we aim for velocity keeping in goal, so no more acceleration
            PlanningState goal_state_long{0.0,CONFIGURATION::speed_limit_m_s - (j *CONFIGURATION::longitudinal_velocity_goal_shift),0.0};//s,s_dot,s_dotdot
            //Generate Polynomial for longitudinal trajectory
            QuarticPolynomialTrajectory longitudinal_trajectory_longer;
            longitudinal_trajectory_longer.max_planning_time = planning_end_time;
            longitudinal_trajectory_longer.polynomial = PolynomialSolver::CalculateQuarticPolynomialCoefficients(current_state_long, goal_state_long, planning_end_time);
            polynomial_candidates_longitudinal.push_back(longitudinal_trajectory_longer);  
        }
    }
    //convert to discrete and add jerk weight
    int x = 5;
    
    //TODO: discretize, calculate costs, combine select
    DiscretizedTrajectory test = TrajectoryConverter::DiscretizeQuarticLongitudinalPolynomialTrajectory(polynomial_candidates_longitudinal.at(0));
    

    
}

std::vector<DiscretizedTrajectory> TrajectoryPlanner::GenerateFollowingTrajectory(PlanningState start_state, PlanningState final_state, double planning_time, double offset)
{}

std::vector<DiscretizedTrajectory> TrajectoryPlanner::GenerateMergeTrajectory(PlanningState start_state, PlanningState final_state, double planning_time, double offset)
{}

std::vector<DiscretizedTrajectory> TrajectoryPlanner::GenerateLaneChangeTrajectory()
{}

void TrajectoryPlanner::IncrementPlanningTimeStep()
{
    if(current_planning_timestep_ >= CONFIGURATION::num_trajectory_points)
    {
        current_planning_timestep_ = 0;
        return;
    }
    current_planning_timestep_++;
}

void TrajectoryPlanner::SetManeuverRequest(Maneuver requested_maneuver)
{
    if(requested_maneuver == current_maneuver_)
    {
        return;
    }
    
    current_planning_timestep_ = -1;
    current_maneuver_ = requested_maneuver;
}





