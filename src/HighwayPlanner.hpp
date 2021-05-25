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



class HighwayPlanner{
    
public:
    HighwayPlanner();
    ~HighwayPlanner();
    void init();
    void step();
    //void shutdown();
    void setMap(std::vector<double> waypoints_x, std::vector<double> waypoints_y, std::vector<double> waypoints_s,std::vector<double> waypoints_dx, std::vector<double> waypoints_dy);
    void setVehicleState(double x, double y, double s, double d, double heading, double speed);
    void setObjectToIndex(int index, unsigned int id, double pos_x, double pos_y, double vx, double vy, double s, double d);
    void clearObjectList();
    GoalCoordianteList GetOutputPath();
    
private:
    TrajectoryPlanner trajectory_planner_;
    BehaviorPlanner behavior_planner_;
    bool is_initialized_ = false;
    std::shared_ptr<VehicleState> vehicle_state_ = nullptr;
    std::shared_ptr<ObjectList> object_list_ = nullptr;
    std::shared_ptr<OutputPath> output_path_ = nullptr;
    std::shared_ptr<WaypointMap> global_map_ = nullptr;
    
    
};



#endif /* HighwayPlanner_hpp */
