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
    //just execute replanning every tenth step
    double offset = (current_planning_timestep_ ) * CONFIGURATION::delta_t_trajectory_points;
    
    if(std::abs((offset -4))<0.000000001)
    {
        std::cout << "Shit might happen" << std::endl;
    }
    
    
    if(((current_planning_timestep_%10) <= 0.0) || (current_planning_timestep_ == 0))
    {
        switch (current_trajectory_type_) {
            case TrajectoryType::VelocityKeeping:
                std::cout << "GenerateVelocityKeepingInLaneTrajectory: " << current_planning_timestep_ << std::endl;
                GenerateVelocityKeepingInLaneTrajectory(current_goal_state_.longitudinal.position_dot , current_goal_state_.lateral.position, CONFIGURATION::trajectory_planning_time_total, offset);
                break;
            case TrajectoryType::VehicleFollowing:
                std::cout << "GenerateVehicleFollowingInlaneTrajectory: " << current_planning_timestep_ << std::endl;
                GenerateVehicleFollowingTrajectory(current_goal_state_.longitudinal.position, current_goal_state_.longitudinal.position_dot, current_goal_state_.longitudinal.position_dot_dot, current_goal_state_.lateral.position,CONFIGURATION::trajectory_planning_time_total, offset);
                break;
                
            default:
                break;
        }
        
    }
    else{
        std::cout << "Use previous path: " << current_planning_timestep_ << std::endl;
        output_path_ptr_->planned_coordinates_cartesian.next_x_vals = previous_path_ptr_->planned_coordinates_cartesian.next_x_vals;
        output_path_ptr_->planned_coordinates_cartesian.next_y_vals = previous_path_ptr_->planned_coordinates_cartesian.next_y_vals;
    }
}

void TrajectoryPlanner::GenerateVelocityKeepingInLaneTrajectory(double goal_s_dot , double goal_d, double planning_time, double offset)
{
    //create traj container objects
    std::vector<DiscretizedTrajectory> candidate_trajectories_longitudinal;
    candidate_trajectories_longitudinal.reserve(30);
    std::vector<DiscretizedTrajectory> candidate_trajectories_lateral;
    candidate_trajectories_lateral.reserve(30);
    
    std::cout << "[TrajectoryPlanner::GenerateVelocityKeepingInLaneTrajectory] Goal s_dot: " << goal_s_dot << '\n';
    
    // Set initial states
    const PlanningState current_state_lat{vehicle_pose_ptr_->position_d,vehicle_pose_ptr_->position_d_dot,vehicle_pose_ptr_->position_d_dot_dot};//d,d_dot,d_dotdot,T
    const PlanningState current_state_long{vehicle_pose_ptr_->position_s,vehicle_pose_ptr_->position_s_dot,vehicle_pose_ptr_->position_s_dot_dot};//s,s_dot,s_dotdot,T


    
    for(int i = 0; i <= static_cast<int>(planning_time) ;++i)
    {
        double planning_end_time = (static_cast<double>(i) - offset);
        if(planning_end_time <= 0.0)
        {continue;}

        for(int j = 0; j < 3;++j)
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
//                if(!(lateral_trajectory_pos_shift_disc.exceeds_acceleration_limit||lateral_trajectory_pos_shift_disc.exceeds_acceleration_limit))
//                {
//                    candidate_trajectories_lateral.push_back(lateral_trajectory_pos_shift_disc);
//                }
                

                //>>>>>longitudinal<<<<<
                const PlanningState goal_state_long{0.0,goal_s_dot - ((j+3) *CONFIGURATION::longitudinal_velocity_goal_shift),0.0};//s,s_dot,s_dotdot
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
            }
            //>>>>>lateral<<<<<
            //Generate Polynomial for lateral trajectory
            const PlanningState goal_state_lat_neg_shift{(goal_d-(j*CONFIGURATION::lateral_goal_shift)),0.0,0.0};//d,d_dot,d_dotdot
            QuinticPolynomialTrajectory lateral_trajectory_neg_shift;
            lateral_trajectory_neg_shift.trajectory_type = TrajectoryType::VelocityKeeping;
            lateral_trajectory_neg_shift.max_planning_time = planning_end_time;
            lateral_trajectory_neg_shift.polynomial = PolynomialSolver::CalculateQuinticPolynomialCoefficients(current_state_lat, goal_state_lat_neg_shift, planning_end_time);
            DiscretizedTrajectory lateral_trajectory_neg_shift_disc = TrajectoryConverter::DiscretizeQuinticPolynomialTrajectory(lateral_trajectory_neg_shift);
            candidate_trajectories_lateral.push_back(lateral_trajectory_neg_shift_disc);
//                            if(!(goal_state_lat_neg_shift_disc.exceeds_acceleration_limit||goal_state_lat_neg_shift_disc.exceeds_acceleration_limit))
//                               {
//                                   candidate_trajectories_lateral.push_back(goal_state_lat_neg_shift_disc);
//                               }
            
            //>>>>>longitudinal<<<<<
            //We do not care for goal position, but for the veloctiy, since not used in solver, safe to set to 0, we aim for velocity keeping in goal, so no more acceleration
            const PlanningState goal_state_long{0.0,goal_s_dot - (j *CONFIGURATION::longitudinal_velocity_goal_shift),0.0};//s,s_dot,s_dotdot
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
        }
    }
    
    if(candidate_trajectories_longitudinal.size()<1)
    {
        std::cout << "No valid longitudinal trajectory generated" << std::endl;
    }
    if(candidate_trajectories_lateral.size()<1)
    {
        std::cout << "No valid lateral trajectory generated" << std::endl;
    }

    //for now we just take the best ones and combine them, later we might wanna check every combination
    trajectory_longitudinal_ =  SelectTrajectory(candidate_trajectories_longitudinal);
    trajectory_lateral_ =  SelectTrajectory(candidate_trajectories_lateral);
    //HACK, low speed planning, keep lateral constant
    if(vehicle_pose_ptr_->position_s_dot <= CONFIGURATION::low_speed_limit)
    {
        std::cout << "[TrajectoryPlanner::GenerateVelocityKeepingInLaneTrajectory] Low speed hack active..." << std::endl;
        for(int i = 0; i < trajectory_lateral_.points.size();++i)
        {
            trajectory_lateral_.points.at(i).position = vehicle_pose_ptr_->position_d;
        }
    }

    
    output_path_ptr_->planned_coordinates_cartesian.next_x_vals.clear();
    output_path_ptr_->planned_coordinates_cartesian.next_y_vals.clear();
    
    for(int i = 1; i < CONFIGURATION::num_trajectory_points; ++i)
    {
        const std::vector<double> xy_vec = global_map_ptr_->GetCartesianSpline(trajectory_longitudinal_.points.at(i).position, trajectory_lateral_.points.at(i).position);
        output_path_ptr_->planned_coordinates_cartesian.next_x_vals.push_back(xy_vec.at(0));
        output_path_ptr_->planned_coordinates_cartesian.next_y_vals.push_back(xy_vec.at(1));
    }
}

void TrajectoryPlanner::GenerateVehicleFollowingTrajectory(double goal_s,double goal_s_dot,double goal_s_dot_dot, double goal_d, double planning_time, double offset)
{
    // We would vary: s->object s + delta and use the objects velocity and acceleration at the end of the planning tome, we do not have enought information, we just vary the position and planning time
    //
    //create traj container objects
    std::vector<DiscretizedTrajectory> candidate_trajectories_longitudinal;
    candidate_trajectories_longitudinal.reserve(30);
    std::vector<DiscretizedTrajectory> candidate_trajectories_lateral;
    candidate_trajectories_lateral.reserve(30);
    
    std::cout << "[TrajectoryPlanner::GenerateVehicleFollowingTrajectory] Goal s_dot: " << goal_s_dot << '\n';
    
    // Set initial states
    const PlanningState current_state_lat{vehicle_pose_ptr_->position_d,vehicle_pose_ptr_->position_d_dot,vehicle_pose_ptr_->position_d_dot_dot};//d,d_dot,d_dotdot,T
    const PlanningState current_state_long{vehicle_pose_ptr_->position_s,vehicle_pose_ptr_->position_s_dot,vehicle_pose_ptr_->position_s_dot_dot};//s,s_dot,s_dotdot,T
    //for longitudinal planning, s_target = s_leading_vehicle at (planning time) - (d0 + tau * s_leading)
    Object target;
    double min_dist = std::numeric_limits<double>::max();
    // Get the target in lane
    for(auto& obj : object_list_ptr_->objects)
    {
        if(obj.lane_assignment == vehicle_pose_ptr_->lane_assignment && obj.lane_assignment != Lane::unknown)
        {
            double dist = obj.position_s - vehicle_pose_ptr_->position_s;
            if(dist>=0.0 && dist <= min_dist)
            {
                min_dist = dist;
                target = obj;
            }
        }
    }
    double target_speed = (sqrt(target.velocity.x*target.velocity.x + target.velocity.y*target.velocity.y));
    double safety_part = (CONFIGURATION::min_dist_constant + CONFIGURATION::tau_time_gap * CONFIGURATION::frenet_absolute_speed_hack_factor * target_speed);
    
    
    
    
    
    for(int i = 0; i <= static_cast<int>(planning_time) ;++i)
    {
        double planning_end_time = (static_cast<double>(i) - offset);
        if(planning_end_time <= 0.0)
        {continue;}

        for(int j = 0; j < 3;++j)
        {
            if(j>0)
            {
                //>>>>>lateral<<<<<
                //Lateral shifted traj
                const PlanningState goal_state_lat_pos_shift{(goal_d+(j*CONFIGURATION::lateral_goal_shift)),0.0,0.0};//d,d_dot,d_dotdot}
                QuinticPolynomialTrajectory lateral_trajectory_pos_shift;
                lateral_trajectory_pos_shift.trajectory_type = TrajectoryType::VehicleFollowing;
                lateral_trajectory_pos_shift.max_planning_time = planning_end_time;
                lateral_trajectory_pos_shift.polynomial = PolynomialSolver::CalculateQuinticPolynomialCoefficients(current_state_lat, goal_state_lat_pos_shift, planning_end_time);
                DiscretizedTrajectory lateral_trajectory_pos_shift_disc = TrajectoryConverter::DiscretizeQuinticPolynomialTrajectory(lateral_trajectory_pos_shift);
                candidate_trajectories_lateral.push_back(lateral_trajectory_pos_shift_disc);
//                if(!(lateral_trajectory_pos_shift_disc.exceeds_acceleration_limit||lateral_trajectory_pos_shift_disc.exceeds_acceleration_limit))
//                {
//                    candidate_trajectories_lateral.push_back(lateral_trajectory_pos_shift_disc);
//                }
                
                //>>>>>longitudinal<<<<<
//                int idx = planning_end_time / CONFIGURATION::delta_t_trajectory_points;
//                const PlanningState goal_state_long_pos_shift{target.predicted_longitudinal_position.at(idx) - safety_part,goal_s_dot,goal_s_dot_dot};
//                QuinticPolynomialTrajectory longitudinal_trajectory_pos_shift;
//                longitudinal_trajectory_pos_shift.trajectory_type = TrajectoryType::VehicleFollowing;
//                longitudinal_trajectory_pos_shift.max_planning_time = planning_end_time;
//                longitudinal_trajectory_pos_shift.polynomial = PolynomialSolver::CalculateQuinticPolynomialCoefficients(current_state_long, goal_state_long_pos_shift, planning_end_time);
//                DiscretizedTrajectory longitudinal_trajectory_pos_shift_disc = TrajectoryConverter::DiscretizeQuinticPolynomialTrajectory(longitudinal_trajectory_pos_shift);
//                candidate_trajectories_longitudinal.push_back(longitudinal_trajectory_pos_shift_disc);
                
            }
            //>>>>>lateral<<<<<
            //Generate Polynomial for lateral trajectory
            const PlanningState goal_state_lat_neg_shift{(goal_d-(j*CONFIGURATION::lateral_goal_shift)),0.0,0.0};//d,d_dot,d_dotdot
            QuinticPolynomialTrajectory lateral_trajectory_neg_shift;
            lateral_trajectory_neg_shift.trajectory_type = TrajectoryType::VehicleFollowing;
            lateral_trajectory_neg_shift.max_planning_time = planning_end_time;
            lateral_trajectory_neg_shift.polynomial = PolynomialSolver::CalculateQuinticPolynomialCoefficients(current_state_lat, goal_state_lat_neg_shift, planning_end_time);
            DiscretizedTrajectory lateral_trajectory_neg_shift_disc = TrajectoryConverter::DiscretizeQuinticPolynomialTrajectory(lateral_trajectory_neg_shift);
            candidate_trajectories_lateral.push_back(lateral_trajectory_neg_shift_disc);
//                            if(!(goal_state_lat_neg_shift_disc.exceeds_acceleration_limit||goal_state_lat_neg_shift_disc.exceeds_acceleration_limit))
//                               {
//                                   candidate_trajectories_lateral.push_back(goal_state_lat_neg_shift_disc);
//                               }
            
            //>>>>>longitudinal<<<<<
            int idx = planning_end_time / CONFIGURATION::delta_t_trajectory_points;
            std::cout << "Goal s: " << target.predicted_longitudinal_position.at(idx) - safety_part << '\n';
            const PlanningState goal_state_long_pos_shift{target.predicted_longitudinal_position.at(idx) - safety_part,goal_s_dot,goal_s_dot_dot};
            const PlanningState goal_state_long_neg_shift{goal_s_dot - CONFIGURATION::longitudinal_position_goal_shift,goal_s_dot,goal_s_dot_dot};
            QuinticPolynomialTrajectory longitudinal_trajectory_neg_shift;
            longitudinal_trajectory_neg_shift.trajectory_type = TrajectoryType::VehicleFollowing;
            longitudinal_trajectory_neg_shift.max_planning_time = planning_end_time;
            longitudinal_trajectory_neg_shift.polynomial = PolynomialSolver::CalculateQuinticPolynomialCoefficients(current_state_long, goal_state_long_neg_shift, planning_end_time);
            DiscretizedTrajectory longitudinal_trajectory_neg_shift_disc = TrajectoryConverter::DiscretizeQuinticPolynomialTrajectory(longitudinal_trajectory_neg_shift);
            candidate_trajectories_longitudinal.push_back(longitudinal_trajectory_neg_shift_disc);
        }
        
    }
    
    if(candidate_trajectories_longitudinal.size()<1)
    {
        std::cout << "No valid longitudinal trajectory generated" << std::endl;
    }
    if(candidate_trajectories_lateral.size()<1)
    {
        std::cout << "No valid lateral trajectory generated" << std::endl;
    }

    //for now we just take the best ones and combine them, later we might wanna check every combination
    trajectory_longitudinal_ =  SelectTrajectory(candidate_trajectories_longitudinal);
    trajectory_lateral_ =  SelectTrajectory(candidate_trajectories_lateral);

    
    output_path_ptr_->planned_coordinates_cartesian.next_x_vals.clear();
    output_path_ptr_->planned_coordinates_cartesian.next_y_vals.clear();
    std::cout << "Vehicle following trajectory" << std::endl;
    for(int i = 1; i < CONFIGURATION::num_trajectory_points; ++i)
    {
        const std::vector<double> xy_vec = global_map_ptr_->GetCartesianSpline(trajectory_longitudinal_.points.at(i).position, trajectory_lateral_.points.at(i).position);
        std::cout << "Point: "<< i << "s: " << trajectory_longitudinal_.points.at(i).position << "d: " << trajectory_lateral_.points.at(i).position << std::endl;
        output_path_ptr_->planned_coordinates_cartesian.next_x_vals.push_back(xy_vec.at(0));
        output_path_ptr_->planned_coordinates_cartesian.next_y_vals.push_back(xy_vec.at(1));
    }

}

void TrajectoryPlanner::GenerateMergeTrajectory(PlanningState start_state, PlanningState final_state, double planning_time, double offset)
{}

void TrajectoryPlanner::GenerateLaneChangeTrajectory()
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

void TrajectoryPlanner::SetManeuverRequest(TrajectoryType requested_trajectory, GoalState requested_state)
{
    current_goal_state_ = requested_state;
    if(requested_trajectory == current_trajectory_type_)
    {
        return;
    }
    
    current_planning_timestep_ = -1;
    
    current_trajectory_type_ = requested_trajectory;
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





