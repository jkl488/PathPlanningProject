//
//  ObjectPrediction.cpp
//  path_planning
//
//  Created by Jakob Klein on 23.05.21.
//

#include "ObjectPrediction.hpp"
#include "DataTypes.hpp"
#include <iostream>
#include "config.hpp"
#include <cmath>




void Prediction::Init(std::shared_ptr<ObjectList> object_list)
{
    object_list_ptr_ = object_list;
}

void Prediction::PredictObjectTrajectories()
{
    //create a path for each object
    //should have as many pts as the trajectory same dt
    //position is sufficient
    if(!object_list_ptr_)
    {
        std::cout << "ERROR: object_list_ptr_ is nullptr..." << '\n';
        return;
    }
    for(auto& object: object_list_ptr_->objects)
    {
        for(int i = 0; i < CONFIGURATION::num_trajectory_points; ++i)
        {
            double velocity_total = std::sqrt(object.velocity.x*object.velocity.x + object.velocity.y * object.velocity.y);
            //s + toal_velo * time elapsed
            object.predicted_longitudinal_position.at(i) = object.position_s + i * CONFIGURATION::delta_t_trajectory_points * velocity_total;
        }
    }
    
}
