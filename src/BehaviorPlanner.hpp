//
//  BehaviorPlanner.hpp
//  path_planning
//
//  Created by Jakob Klein on 23.05.21.
//

#ifndef BehaviorPlanner_hpp
#define BehaviorPlanner_hpp

#include <stdio.h>
#include "DataTypes.hpp"
#include "Map.hpp"


class BehaviorPlanner
{
public:
    BehaviorPlanner() = default;
    ~BehaviorPlanner() = default;
    BehaviorPlanner(const BehaviorPlanner& rhs) = default;
    BehaviorPlanner& operator= (const BehaviorPlanner& rhs) = default;
    BehaviorPlanner(BehaviorPlanner&& rhs) = default;
    BehaviorPlanner& operator= (BehaviorPlanner&& rhs) = default;
    
    
    void Init(std::shared_ptr<VehiclePose> vehicle_pose, std::shared_ptr<ObjectList> object_list, std::shared_ptr<Map> global_map);
    void Step();
    Maneuver GetCurrentManeuver();
    TrajectoryType GetCurrentTrajectoryType();
    GoalState GetCurrentGoal();
    
    
    
private:
    bool is_initialized_ = false;
    std::shared_ptr<VehiclePose> vehicle_pose_ptr_ = nullptr;
    std::shared_ptr<ObjectList> object_list_ptr_ = nullptr;
    std::shared_ptr<OutputPath> output_path_ptr_ = nullptr;
    std::shared_ptr<Map> global_map_ptr_ = nullptr;


    GoalState current_goal_;
    Maneuver current_state_;
    TrajectoryType current_trajectory_type_;
    //counts how long we are in the current state
    unsigned int state_counter_;
    //run the whole state machine
    void RunStateMachine();
    //Select which state will be executed
    Maneuver SelectNextState(std::vector<Maneuver> possible_maneuvers);
    //Generate the next lateral and longitudinal goals
    void GenerateGoalState(Maneuver next_maneuver);
    //Search for states we can transition to
    std::vector<Maneuver> SearchPossibleStates();
    //calculate the cost for a maneuver
    double CalculateMenueverCost(Maneuver man);
    //Generate the goal position, dependend on Maneuver
    GoalState GenerateGoalPosition(Maneuver man);
    //Generate Goal position for the maneuvers
    GoalState GenerateLaneKeepingGoalPosition();
//    GoalState GeneratePrepareLaneLeftChangeGoalPosition();
//    GoalState GeneratePrepareLaneChangeRightGoalPosition();
//    GoalState GenerateLaneChangeLeftGoalPosition();
//    GoalState GenerateLaneChangeRightGoalPosition();
    
    
    
    
    //Run lane keeping maneuver
//    void RunManeuverKeepLane();
//    void transitionToLaneChangeLeft();
//    void transitionToLaneChangeRight();
//    void runManeuverLaneChangeLeft();
//    void runManeuverLaneChangeRight();
    
    
    //cost functions
    
};

#endif /* BehaviorPlanner_hpp */
