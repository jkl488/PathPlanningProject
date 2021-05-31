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


class BehaviorPlanner
{
public:
    BehaviorPlanner();
    ~BehaviorPlanner();
    void init(std::shared_ptr<VehiclePose> vehicle_pose, std::shared_ptr<ObjectList> object_list, std::shared_ptr<WaypointMap> global_map);
    void step();
    
private:
    bool is_initialized_ = false;
    std::shared_ptr<VehiclePose> vehicle_pose_ptr_ = nullptr;
    std::shared_ptr<ObjectList> object_list_ptr_ = nullptr;
    std::shared_ptr<OutputPath> output_path_ptr_ = nullptr;
    std::shared_ptr<WaypointMap> global_map_ptr_ = nullptr;


    GoalState current_goal_;
    Maneuver current_state_;
    //counts how long we are in the current state
    unsigned int state_counter_;
    //run the whole state machine
    void runStateMachine();
    //Run lane keeping maneuver
    void runManeuverKeepLane();
    void transitionToLaneChangeLeft();
    void transitionToLaneChangeRight();
    void runManeuverLaneChangeLeft();
    void runManeuverLaneChangeRight();
    
    
    //cost functions
    
};

#endif /* BehaviorPlanner_hpp */
