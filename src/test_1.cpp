#include <multidof_walking/wpg/simple_walker.h>
#include <XBotInterface/MatLogger.hpp>

int main()
{
    auto logger = XBot::MatLogger::getLogger("/tmp/mdof_walker");
    
    double dt = 0.002;
    mdof::Walker::Options opt;
    mdof::Walker walker(dt, opt);
    
    mdof::RobotState state, ref;
    ref.com_pos << 0.0, 0.0, opt.com_height;
    ref.com_vel << 0.1, 0.0, 0.0;
    ref.foot_pos[0] << 0.0, 0.0, 0.0;
    ref.foot_pos_goal = ref.foot_pos_start = ref.foot_pos;
    
    int f_idx = 0;
    double time = 0.0;
    walker.set_vref(-0.2);
    walker.start(time - 2.0, state);
    
    for(int iter = 0; iter < 40000; iter++)
    {
        
        state = ref;
        walker.run(time, state, ref);
        time += dt;
        

        for(int i : {0, 1})
        {
            ref.foot_pos[i] = mdof::compute_swing_trajectory(
                ref.foot_pos_start[i], 
                ref.foot_pos_goal[i],
                0.10,
                ref.t_start[i],
                ref.t_goal[i],
                time,
                1.5);
        }
        
        logger->add("pos", state.com_pos);
        logger->add("vel", state.com_vel);
        logger->add("zmp", state.zmp);
        logger->add("lfoot", state.foot_pos[0]);
        logger->add("rfoot", state.foot_pos[1]);
        
    }
    
    logger->flush();
}
