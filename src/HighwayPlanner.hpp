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
    void setVehicleState(double x, double y, double s, double d, double heading, double speed);
    void setObjectList();
    
private:
    TrajectoryPlanner trajectory_planner_;
    std::shared_ptr<VehicleState> vehicle_state_ = nullptr;
    std::shared_ptr<ObjectList> object_list_ = nullptr;
    
};



#endif /* HighwayPlanner_hpp */
