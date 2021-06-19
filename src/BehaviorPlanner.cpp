//
//  BehaviorPlanner.cpp
//  path_planning
//
//  Created by Jakob Klein on 23.05.21.
//

#include "BehaviorPlanner.hpp"
#include <iostream>
#include "HelperFunctions.hpp"

void BehaviorPlanner::Init(std::shared_ptr<VehiclePose> vehicle_pose, std::shared_ptr<ObjectList> object_list , std::shared_ptr<Map> global_map)
{
    vehicle_pose_ptr_ = vehicle_pose;
    object_list_ptr_ = object_list;
    global_map_ptr_ = global_map;
    
    current_state_ = Maneuver::ManeuverKeepLane; //could also be init, but not yet needed
}

void BehaviorPlanner::Step()
{
    RunStateMachine();
    
}

void BehaviorPlanner::RunStateMachine()
{
    
    std::vector<Maneuver> possible_maneuvers;
    Maneuver next_maneuver;
    switch (current_state_) {
        case Maneuver::ManeuverKeepLane:
            std::cout << "ManeuverKeepLane" << '\n';
            possible_maneuvers = SearchPossibleStates();
            next_maneuver = SelectNextState(possible_maneuvers);
            current_goal_ = GenerateLaneKeepingGoalPosition();
            
            //if follow
            //if slower then our speed we would like do a lane change (preferably left (as long as we are not on th leftmost lane, else right)
            
            //if freedrive
            
            //if stop
            
            //POSSIBLE TRANSITION: ManeuverPrepareLaneChangeLeft, ManeuverLaneChangeRight
            break;
        case Maneuver::ManeuverPrepareLaneChangeLeft:
            std::cout << "ManeuverPrepareLaneChangeLeft" << '\n';
            possible_maneuvers = SearchPossibleStates();
            //follow until LC possible
            
            //POSSIBLE TRANSITION: ManeuverLaneChangeLeft, ManeuverKeepLane
            break;
        case Maneuver::ManeuverPrepareLaneChangeRight:
            std::cout << "ManeuverPrepareLaneChangeRight" << '\n';
            possible_maneuvers = SearchPossibleStates();
            //follow until LC possible
            
            //POSSIBLE TRANSITION: ManeuverLaneChangeLeft, ManeuverKeepLane
            break;
        case Maneuver::ManeuverLaneChangeLeft:
            std::cout << "ManeuverLaneChangeLeft" << '\n';
            possible_maneuvers = SearchPossibleStates();
            //until we are fully in that lane
            
            //POSSIBLE TRANSITION: ManeuverKeepLane
            break;
        case Maneuver::ManeuverLaneChangeRight:
            std::cout << "ManeuverLaneChangeRight" << '\n';
            possible_maneuvers = SearchPossibleStates();
            //until we are fully in that lane
            
            //POSSIBLE TRANSITION: ManeuverKeepLane
            break;
        
            
        default:
            break;
    }
}

Maneuver BehaviorPlanner::GetCurrentManeuver()
{
    return current_state_;
}

GoalState BehaviorPlanner::GetCurrentGoal()
{
    return current_goal_;
}

TrajectoryType BehaviorPlanner::GetCurrentTrajectoryType()
{
    return current_trajectory_type_;
}


std::vector<Maneuver> BehaviorPlanner::SearchPossibleStates()
{
    std::vector<Maneuver> possible_maneuvers;
    const Maneuver& current_maneuver = GetCurrentManeuver();
    
    if(current_maneuver == Maneuver::ManeuverKeepLane)
    {
        possible_maneuvers.push_back(Maneuver::ManeuverKeepLane);
//        possible_maneuvers.push_back(Maneuver::ManeuverPrepareLaneChangeLeft);
//        possible_maneuvers.push_back(Maneuver::ManeuverPrepareLaneChangeRight);
    }
//    else if(current_maneuver == Maneuver::ManeuverPrepareLaneChangeLeft)
//    {
//        possible_maneuvers.push_back(Maneuver::ManeuverPrepareLaneChangeLeft);
//        possible_maneuvers.push_back(Maneuver::ManeuverLaneChangeLeft);
//        possible_maneuvers.push_back(Maneuver::ManeuverKeepLane);
//    }
//    else if(current_maneuver == Maneuver::ManeuverPrepareLaneChangeRight)
//    {
//        possible_maneuvers.push_back(Maneuver::ManeuverPrepareLaneChangeRight);
//        possible_maneuvers.push_back(Maneuver::ManeuverLaneChangeRight);
//        possible_maneuvers.push_back(Maneuver::ManeuverKeepLane);
//    }
//    else if(current_maneuver == Maneuver::ManeuverLaneChangeLeft)
//    {
//        possible_maneuvers.push_back(Maneuver::ManeuverLaneChangeLeft);
//        possible_maneuvers.push_back(Maneuver::ManeuverKeepLane);
//    }
//    else if(current_maneuver == Maneuver::ManeuverLaneChangeRight)
//    {
//        possible_maneuvers.push_back(Maneuver::ManeuverLaneChangeRight);
//        possible_maneuvers.push_back(Maneuver::ManeuverKeepLane);
//    }
    return possible_maneuvers;
    
}


void BehaviorPlanner::GenerateGoalState(Maneuver next_maneuver)
{
    
}


Maneuver BehaviorPlanner::SelectNextState(std::vector<Maneuver> possible_maneuvers)
{
    std::vector<double> costs(possible_maneuvers.size());
    for(auto& man : possible_maneuvers)
    {
        GoalState goal_state = GenerateGoalPosition(man);
//        costs.push_back(CalculateMenueverCost(goal_state));
    }
    
}

GoalState BehaviorPlanner::GenerateGoalPosition(Maneuver man)
{
    GoalState return_state;
    switch (man) {
        case Maneuver::ManeuverKeepLane:
            return_state = GenerateLaneKeepingGoalPosition();
            break;
        case Maneuver::ManeuverPrepareLaneChangeLeft:

            break;
        case Maneuver::ManeuverPrepareLaneChangeRight:
 
            break;
        case Maneuver::ManeuverLaneChangeLeft:

            break;
        case Maneuver::ManeuverLaneChangeRight:

            break;
        default:
            break;
    }
    return return_state;
    
}

GoalState BehaviorPlanner::GenerateLaneKeepingGoalPosition()
{
    GoalState return_goal_state;
    Object target;
    double min_dist = std::numeric_limits<double>::max();
    // Get the target in lane
    for(auto& obj : object_list_ptr_->objects)
    {
        if(obj.lane_assignment == vehicle_pose_ptr_->lane_assignment && target.lane_assignment != Lane::unknown)
        {
            double dist = obj.position_s - vehicle_pose_ptr_->position_s;
            if(dist>=0.0 && dist <= min_dist)
            {
                min_dist = dist;
                target = obj;
            }
        }
    }
    //Check if we want to follow it
    double max_following_dist = 2.0 * 0.9 * vehicle_pose_ptr_->position_s_dot;
    if(target.lane_assignment != Lane::unknown && max_following_dist <= min_dist)
    {
        //Vehicle following goal
        current_trajectory_type_ = TrajectoryType::VehicleFollowing;
        //longitudinal
        return_goal_state.longitudinal.position = target.position_s;
        return_goal_state.longitudinal.position_dot = 0.9 * (sqrt(target.velocity.x*target.velocity.x + target.velocity.y*target.velocity.y));
        return_goal_state.longitudinal.position_dot_dot = 0.0;//we should use s_dot_dot of target, but let's aim for 0 acceleration here
    }
    else
    {
        //Velocity Keeping goal
        current_trajectory_type_ = TrajectoryType::VelocityKeeping;
        //longitudinal
        return_goal_state.longitudinal.position = 0.0;
        return_goal_state.longitudinal.position_dot = 0.9 * CONFIGURATION::speed_limit_m_s;
        return_goal_state.longitudinal.position_dot_dot = 0.0;
    }
    
    //lateral
    return_goal_state.lateral.position_dot = 0.0;
    return_goal_state.lateral.position_dot_dot = 0.0;
    if(vehicle_pose_ptr_->lane_assignment == Lane::leftmost)
    {
        return_goal_state.lateral.position = CONFIGURATION::left_lane_center;
    }
    else if (vehicle_pose_ptr_->lane_assignment == Lane::middle)
    {
        return_goal_state.lateral.position = CONFIGURATION::middle_lane_center;
    }
    else
    {
        return_goal_state.lateral.position = CONFIGURATION::right_lane_center;
    }
    
    return return_goal_state;
}
//GoalState BehaviorPlanner::GeneratePrepareLaneLeftChangeGoalPosition(Maneuver man)
//{
//
//}
//GoalState BehaviorPlanner::GeneratePrepareLaneChangeRightGoalPosition(Maneuver man)
//{
//
//}
//GoalState BehaviorPlanner::GenerateLaneChangeLeftGoalPosition(Maneuver)
//{
//
//}
//GoalState BehaviorPlanner::GenerateLaneChangeRightGoalPosition(Maneuver man)
//{
//
//}

double BehaviorPlanner::CalculateMenueverCost(Maneuver man)
{
    
}


