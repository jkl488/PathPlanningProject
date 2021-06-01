//
//  TrajectroyConverter.hpp
//  path_planning
//
//  Created by Jakob Klein on 01.06.21.
//

#ifndef TrajectoryConverter_hpp
#define TrajectoryConverter_hpp

#include <stdio.h>
#include "DataTypes.hpp"

class TrajectoryConverter
{
public:
    
    static DiscretizedTrajectory DiscretizeQuarticLongitudinalPolynomialTrajectory(const QuarticPolynomialTrajectory& polynomial_trajectory);

    static DiscretizedTrajectory DiscretizeQuinticPolynomialTrajectory(const QuinticPolynomialTrajectory& polynomial_trajectory);
    
private:
    
};

#endif /* TrajectoryConverter_hpp */
