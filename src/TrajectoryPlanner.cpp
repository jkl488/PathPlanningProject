//
//  TrajectoryPlanner.cpp
//  path_planning
//
//  Created by Jakob Klein on 23.05.21.
//

#include "TrajectoryPlanner.hpp"
#include <vector>
#include <iostream>
#include <iomanip>
#include "HelperFunctions.hpp"
#include "config.hpp"
#include "PolynomialSolver.hpp"
#include "TrajectoryConverter.hpp"


//Todo
//goal in step should come from behavior (implicit)
//plan lateral trajectory
//combine in every possible way
//generate total score
//do a collision check on the best, if fails second best and so on
//select the best

void TrajectoryPlanner::Init(std::shared_ptr<VehiclePose> vehicle_pose, std::shared_ptr<ObjectList> object_list , std::shared_ptr<Map> global_map, std::shared_ptr<OutputPath> output_path, std::shared_ptr<OutputPath> previous_path)
{
    vehicle_pose_ptr_ = vehicle_pose;
    object_list_ptr_ = object_list;
    global_map_ptr_ = global_map;
    previous_path_ptr_ = previous_path;
    output_path_ptr_ = output_path;
}

void TrajectoryPlanner::Step()
{
    //Update the timestep
    IncrementPlanningTimeStep();
    
    //Generate a bunch of trajectories
    double goal_s_dot = 0.9 * CONFIGURATION::speed_limit_m_s;
    double goal_d =  6.0;
    double planning_time = CONFIGURATION::trajectory_planning_time_total;
    //TODO: maybe we need to take number of executed elements into account
    
    
    //just execute repanning every tenth step
    double offset = (current_planning_timestep_ ) * CONFIGURATION::delta_t_trajectory_points;
    
    if(std::abs((offset -4))<0.000000001)
    {
        std::cout << "Shit might happen" << std::endl;
    }
    
    //GenerateVelocityKeepingInLaneTrajectory(goal_s_dot , goal_d, planning_time, offset);
    if(previous_path_ptr_->planned_coordinates_cartesian.next_x_vals.size()>0)
    {
    std::cout << "First point previous, x: " << previous_path_ptr_->planned_coordinates_cartesian.next_x_vals.at(0) << ",y: " << previous_path_ptr_->planned_coordinates_cartesian.next_y_vals.at(0)<< std::endl;
    }
    
    
    if(((current_planning_timestep_%10) <= 0.0) || (current_planning_timestep_ == 0))
    {
        std::cout << "GenerateVelocityKeepingInLaneTrajectory: " << current_planning_timestep_ << std::endl;
        GenerateVelocityKeepingInLaneTrajectory(goal_s_dot , goal_d, planning_time, offset);
    }
    else{
        std::cout << "Use previous path: " << current_planning_timestep_ << std::endl;
        output_path_ptr_->planned_coordinates_cartesian.next_x_vals = previous_path_ptr_->planned_coordinates_cartesian.next_x_vals;
        output_path_ptr_->planned_coordinates_cartesian.next_y_vals = previous_path_ptr_->planned_coordinates_cartesian.next_y_vals;
    }
    
    std::cout << "First point output, x: " << output_path_ptr_->planned_coordinates_cartesian.next_x_vals.at(0) << ",y: " << output_path_ptr_->planned_coordinates_cartesian.next_y_vals.at(0)<< std::endl;
    


}

void TrajectoryPlanner::GenerateVelocityKeepingInLaneTrajectory(double goal_s_dot , double goal_d, double planning_time, double offset)
{
    //create traj container objects
    std::vector<DiscretizedTrajectory> candidate_trajectories_longitudinal;
    candidate_trajectories_longitudinal.reserve(30);
    std::vector<DiscretizedTrajectory> candidate_trajectories_lateral;
    candidate_trajectories_lateral.reserve(30);
    
    // Set initial states
    const PlanningState current_state_lat{vehicle_pose_ptr_->position_d,vehicle_pose_ptr_->position_d_dot,vehicle_pose_ptr_->position_d_dot_dot};//d,d_dot,d_dotdot,T
    const PlanningState current_state_long{vehicle_pose_ptr_->position_s,vehicle_pose_ptr_->position_s_dot,vehicle_pose_ptr_->position_s_dot_dot};//s,s_dot,s_dotdot,T


    
    for(int i = 0; i <= static_cast<int>(planning_time) ;++i)
    {
        double planning_end_time = (static_cast<double>(i) - offset);
        if(planning_end_time <= 0.0)
        {continue;}
        std::cout << "planning_end_time: "<< planning_end_time<< '\n';

        for(int j = 0; j < 1;++j)//TODO: CHANGE THAT TO 3
        {
            if(j>0)
            {
                //>>>>>lateral<<<<<
                //Lateral shifted traj
                const PlanningState goal_state_lat_pos_shift{(goal_d+(j*CONFIGURATION::lateral_goal_shift)),0.0,0.0};//d,d_dot,d_dotdot}
                QuinticPolynomialTrajectory lateral_trajectory_pos_shift;
                lateral_trajectory_pos_shift.trajectory_type = TrajectoryType::VelocityKeeping;
                lateral_trajectory_pos_shift.max_planning_time = planning_end_time;
                lateral_trajectory_pos_shift.polynomial = PolynomialSolver::CalculateQuinticPolynomialCoefficients(current_state_lat, goal_state_lat_pos_shift, planning_end_time);
                DiscretizedTrajectory lateral_trajectory_pos_shift_disc = TrajectoryConverter::DiscretizeQuinticPolynomialTrajectory(lateral_trajectory_pos_shift);
                candidate_trajectories_lateral.push_back(lateral_trajectory_pos_shift_disc);
                //                if(!(longitudinal_trajectory_shorter_disc.exceeds_acceleration_limit||longitudinal_trajectory_shorter_disc.exceeds_acceleration_limit))
                //                   {
                //                    candidate_trajectories_longitudinal.push_back(longitudinal_trajectory_shorter_disc);
                //                   }
                

                //>>>>>longitudinal<<<<<
                const PlanningState goal_state_long{0.0,CONFIGURATION::speed_limit_m_s - ((j+3) *CONFIGURATION::longitudinal_velocity_goal_shift),0.0};//s,s_dot,s_dotdot
                //Generate Polynomial for longitudinal trajectory
                QuarticPolynomialTrajectory longitudinal_trajectory_shorter;
                longitudinal_trajectory_shorter.trajectory_type = TrajectoryType::VelocityKeeping;
                longitudinal_trajectory_shorter.max_planning_time = planning_end_time;
                longitudinal_trajectory_shorter.polynomial = PolynomialSolver::CalculateQuarticPolynomialCoefficients(current_state_long, goal_state_long, planning_end_time);
                
                DiscretizedTrajectory longitudinal_trajectory_shorter_disc  = TrajectoryConverter::DiscretizeQuarticLongitudinalPolynomialTrajectory(longitudinal_trajectory_shorter);
                longitudinal_trajectory_shorter_disc.score = TrajectoryConverter::ScoreTrajectory(longitudinal_trajectory_shorter_disc, longitudinal_trajectory_shorter, goal_state_long);
                
                
                
                candidate_trajectories_longitudinal.push_back(longitudinal_trajectory_shorter_disc);
//                if(!(longitudinal_trajectory_shorter_disc.exceeds_acceleration_limit||longitudinal_trajectory_shorter_disc.exceeds_acceleration_limit))
//                   {
//                    candidate_trajectories_longitudinal.push_back(longitudinal_trajectory_shorter_disc);
//                   }
                
                
                //polynomial_candidates_longitudinal.push_back(longitudinal_trajectory_shorter);
            }
            //>>>>>lateral<<<<<
            //Generate Polynomial for lateral trajectory
            const PlanningState goal_state_lat_neg_shift{(goal_d-(j*CONFIGURATION::lateral_goal_shift)),0.0,0.0};//d,d_dot,d_dotdot
            QuinticPolynomialTrajectory lateral_trajectory_neg_shift;
            lateral_trajectory_neg_shift.trajectory_type = TrajectoryType::VelocityKeeping;
            lateral_trajectory_neg_shift.max_planning_time = planning_end_time;
            lateral_trajectory_neg_shift.polynomial = PolynomialSolver::CalculateQuinticPolynomialCoefficients(current_state_lat, goal_state_lat_neg_shift, planning_end_time);
            DiscretizedTrajectory goal_state_lat_neg_shift_disc = TrajectoryConverter::DiscretizeQuinticPolynomialTrajectory(lateral_trajectory_neg_shift);
            candidate_trajectories_lateral.push_back(goal_state_lat_neg_shift_disc);
            //                if(!(longitudinal_trajectory_shorter_disc.exceeds_acceleration_limit||longitudinal_trajectory_shorter_disc.exceeds_acceleration_limit))
            //                   {
            //                    candidate_trajectories_longitudinal.push_back(longitudinal_trajectory_shorter_disc);
            //                   }
            
            //>>>>>longitudinal<<<<<
            //We do not care for goal position, but for the veloctiy, since not used in solver, safe to set to 0, we aim for velocity keeping in goal, so no more acceleration
            const PlanningState goal_state_long{0.0,CONFIGURATION::speed_limit_m_s - (j *CONFIGURATION::longitudinal_velocity_goal_shift),0.0};//s,s_dot,s_dotdot
            //Generate Polynomial for longitudinal trajectory
            QuarticPolynomialTrajectory longitudinal_trajectory_longer;
            longitudinal_trajectory_longer.trajectory_type = TrajectoryType::VelocityKeeping;
            longitudinal_trajectory_longer.max_planning_time = planning_end_time;
            longitudinal_trajectory_longer.polynomial = PolynomialSolver::CalculateQuarticPolynomialCoefficients(current_state_long, goal_state_long, planning_end_time);
            
            DiscretizedTrajectory longitudinal_trajectory_longer_disc  = TrajectoryConverter::DiscretizeQuarticLongitudinalPolynomialTrajectory(longitudinal_trajectory_longer);
            longitudinal_trajectory_longer_disc.score = TrajectoryConverter::ScoreTrajectory(longitudinal_trajectory_longer_disc, longitudinal_trajectory_longer, goal_state_long);
            
            candidate_trajectories_longitudinal.push_back(longitudinal_trajectory_longer_disc);
//            if(!(longitudinal_trajectory_longer_disc.exceeds_acceleration_limit||longitudinal_trajectory_longer_disc.exceeds_acceleration_limit))
//               {
//                candidate_trajectories_longitudinal.push_back(longitudinal_trajectory_longer_disc);
//               }
            
            
            //polynomial_candidates_longitudinal.push_back(longitudinal_trajectory_longer);
        }
    }
    
    if(candidate_trajectories_longitudinal.size()<1)
    {
        std::cout << "Shit happened" << std::endl;
    }

    //for now we just take the best ones and combine them, later we might wanna chack every combination
    trajectory_longitudinal_ =  SelectTrajectory(candidate_trajectories_longitudinal);
    trajectory_lateral_ =  SelectTrajectory(candidate_trajectories_lateral);

    
    output_path_ptr_->planned_coordinates_cartesian.next_x_vals.clear();
    output_path_ptr_->planned_coordinates_cartesian.next_y_vals.clear();
    
    for(int i = 1; i < CONFIGURATION::num_trajectory_points; ++i)
    {
        const std::vector<double> xy_vec = global_map_ptr_->GetCartesian(trajectory_longitudinal_.points.at(i).position, trajectory_lateral_.points.at(i).position);
        if(i<50)
        {
//            std::cout << "point: " << i << ", x: " << xy_vec.at(0) << ",y: " << xy_vec.at(1) <<std::endl;
            std::cout << "point: " << i << ", s: " << trajectory_longitudinal_.points.at(i).position << ",d: " << trajectory_lateral_.points.at(i).position <<std::endl;
        }
        output_path_ptr_->planned_coordinates_cartesian.next_x_vals.push_back(xy_vec.at(0));
        output_path_ptr_->planned_coordinates_cartesian.next_y_vals.push_back(xy_vec.at(1));
        
        
        
    }
}

OutputPath TrajectoryPlanner::GenerateFollowingTrajectory(PlanningState start_state, PlanningState final_state, double planning_time, double offset)
{}

OutputPath TrajectoryPlanner::GenerateMergeTrajectory(PlanningState start_state, PlanningState final_state, double planning_time, double offset)
{}

OutputPath TrajectoryPlanner::GenerateLaneChangeTrajectory()
{}

void TrajectoryPlanner::IncrementPlanningTimeStep()
{
    if(current_planning_timestep_ >= (CONFIGURATION::num_trajectory_points-1))
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

DiscretizedTrajectory TrajectoryPlanner::SelectTrajectory(std::vector<DiscretizedTrajectory>& trajectory_candidates)
{
    std::sort(trajectory_candidates.begin(), trajectory_candidates.end(), [](const DiscretizedTrajectory& lhs, const DiscretizedTrajectory& rhs)->bool{return lhs.score < rhs.score;});
    return trajectory_candidates.front();
}


DiscretizedTrajectory TrajectoryPlanner::GetLongitudinalTrajectory()
{
    return trajectory_longitudinal_;
}

DiscretizedTrajectory TrajectoryPlanner::GetLateralTrajectory()
{
    return trajectory_lateral_;
}

//bool TrajectoryPlanner::CheckTrajectoryCollides(const DiscretizedTrajectory& discrete_trajectory)
//{
//    bool trajectory_collides = false;
//
//    for(int i = 0; i < discrete_trajectory.points.size(); i=i+5)
//    {
//        double dist_s = discrete_trajectory.points.at(i).
//
//    }
//
//
//
//    return trajectory_collides;
//}





