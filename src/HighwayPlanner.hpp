//
//  HighwayPlanner.hpp
//  path_planning
//
//  Created by Jakob Klein on 23.05.21.
//

#ifndef HighwayPlanner_hpp
#define HighwayPlanner_hpp

#include <stdio.h>
#include "TrajectoryPlanner.hpp"
#include "BehaviorPlanner.hpp"
#include "DataTypes.hpp"
#include "ObjectPrediction.hpp"
#include "config.hpp"
#include "Map.hpp"


class HighwayPlanner{
    
public:
    HighwayPlanner();
    ~HighwayPlanner()= default;
    //default copy behavior
    HighwayPlanner(const HighwayPlanner& rhs) = default;
    HighwayPlanner& operator= (const HighwayPlanner& rhs) = default;
    //default move behavior
    HighwayPlanner(HighwayPlanner&& rhs) = default;
    HighwayPlanner& operator= ( HighwayPlanner&& rhs) = default;
    
    void Init();
    void Step();
    void SetMap(const Map& highway_map);
    void SetVehiclePose(double x, double y, double s, double d, double heading, double speed, int num_previous_path_left);
    GoalCoordianteList GetOutputPath();
    void SetPreviousPath(OutputPath previous_path);
    void InsertObjects(std::vector<std::vector<double>> sensor_fusion);
    
private:
    TrajectoryPlanner trajectory_planner_;
    BehaviorPlanner behavior_planner_;
    Prediction object_prediction_;
    bool is_initialized_ = false;
    std::shared_ptr<VehiclePose> vehicle_pose_ptr_ = nullptr;
    std::shared_ptr<ObjectList> object_list_ptr_ = nullptr;
    std::shared_ptr<OutputPath> output_path_ptr_ = nullptr;
    std::shared_ptr<Map> global_map_ptr_ = nullptr;
    std::shared_ptr<OutputPath> previous_path_ptr_ = nullptr;
    long long cycle_counter_{0};
    
    
};



#endif /* HighwayPlanner_hpp */
