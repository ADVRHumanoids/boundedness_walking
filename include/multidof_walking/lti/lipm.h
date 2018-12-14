#ifndef __MULTIDOF_WALKING_LIPM_H__
#define __MULTIDOF_WALKING_LIPM_H__

#include <OpenMpC/dynamics/LtiDynamics.h>

namespace mdof {
 
    OpenMpC::dynamics::LtiDynamics::Ptr MakeLipmContinuousTime(double omega);
    OpenMpC::dynamics::LtiDynamics::Ptr MakeLipmExtended(double omega, double dt);
    
}



#endif

