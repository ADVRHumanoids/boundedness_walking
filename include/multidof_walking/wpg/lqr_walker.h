#ifndef __MDOF_WALKER_H__
#define __MDOF_WALKER_H__

#include <multidof_walking/lti/lipm.h>

namespace mdof {
 
    class Walker 
    {
      
    public:
        
        struct Options
        {
            double com_height = 1.0;
            double step_duration = 1.0;
            double step_width = 0.2;
        };
        
        Walker(Options opt = Options());
        
    private:
        
        
    };
    
}

#endif