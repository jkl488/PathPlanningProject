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
    Prediction() = default;
    ~Prediction()= default;
    //default copy behavior
    Prediction(const Prediction& rhs) = default;
    Prediction& operator= (const Prediction& rhs) = default;
    //default move behavior
    Prediction(Prediction&& rhs) = default;
    Prediction& operator= ( Prediction&& rhs) = default;
    
    
    //Prediction(std::shared_ptr<ObjectList> object_list);
    void Init(std::shared_ptr<ObjectList> object_list);
    void PredictObjectTrajectories();
    
private:
    std::shared_ptr<ObjectList> object_list_ptr_ = nullptr;
    
    
};
#endif /* ObjectPrediction_hpp */
