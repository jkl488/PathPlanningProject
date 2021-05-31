//
//  ObjectPrediction.hpp
//  path_planning
//
//  Created by Jakob Klein on 23.05.21.
//

#ifndef ObjectPrediction_hpp
#define ObjectPrediction_hpp

#include <stdio.h>
#include <memory>
#include "DataTypes.hpp"

class Prediction
{
public:
    //Prediction(std::shared_ptr<ObjectList> object_list);
    void init(std::shared_ptr<ObjectList> object_list);
    void predictObjectTrajectories();
    
private:
    std::shared_ptr<ObjectList> object_list_ptr_ = nullptr;
    
    
};
#endif /* ObjectPrediction_hpp */
