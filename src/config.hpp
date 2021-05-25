//
//  config.hpp
//  path_planning
//
//  Created by Jakob Klein on 25.05.21.
//

#ifndef config_hpp
#define config_hpp

namespace CONFIGURATION {
//minimum number, the behavior planner has to stay in one state to reduce too many changes of behavior and toggling
unsigned int min_state_iterations = 10;
//maximum number, the behavior planner stays in the state ManeuverPrepareLaneChangeLeft/ManeuverPrepareLaneChangeRight, if it has to abort, LC in both directions becomes allowed
unsigned int max_state_iterations_plc = 50;
}

#endif /* config_hpp */
