#include <multidof_walking/wpg/lqr_walker.h>
#include <XBotInterface/MatLogger.hpp>

int main()
{
    auto logger = XBot::MatLogger::getLogger("/tmp/mdof_walker");
    
    double dt = 0.002;
    mdof::Walker::Options opt;
    mdof::Walker walker(dt, opt);
    
    mdof::RobotState state, ref;
    ref.com_pos << 0.0, 0.0, opt.com_height;
    ref.foot_pos[0] << 0.0, 0.10, 0.0;
    ref.foot_pos[1] << 0.0, -0.10, 0.0;
    ref.foot_pos_goal = ref.foot_pos_start = ref.foot_pos;
    
    double time = 0.0;
    walker.set_vref(0.1);
    walker.start(time, state);
    for(int iter = 0; iter < 40000; iter++)
    {
        if(iter == 20000) walker.set_vref(0.0);
        
        
        state = ref;
//         state.foot_pos = state.foot_pos_goal;
        walker.run(time, state, ref);
        time += dt;
        
        logger->add("pos", state.com_pos);
        logger->add("vel", state.com_vel);
        logger->add("zmp", state.zmp);
        logger->add("lfoot", state.foot_pos[0]);
        logger->add("rfoot", state.foot_pos[1]);
    }
    
    logger->flush();
}