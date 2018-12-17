#ifndef __MDOF_WALKING_ROBOT_STATE_H__
#define __MDOF_WALKING_ROBOT_STATE_H__

#include <XBotInterface/MatLogger.hpp>

namespace mdof {
    
    struct RobotState
    {
      
        Eigen::Vector3d com_pos, com_vel, zmp, eq_foot_pos;
        std::array<Eigen::Vector3d, 2> foot_pos;
        std::array<Eigen::Vector3d, 2> foot_pos_start;
        std::array<Eigen::Vector3d, 2> foot_pos_goal;
        std::array<double, 2> t_start;
        std::array<double, 2> t_goal;
        std::array<bool, 2> foot_contact;
        
        RobotState()
        {
            memset(this, 0, sizeof(*this));
        }
        
        void log(XBot::MatLogger::Ptr logger)
        {
            logger->add("com_pos", com_pos);
            logger->add("com_vel", com_vel);
            logger->add("zmp", zmp);
            logger->add("lfoot", foot_pos[0]);
            logger->add("rfoot", foot_pos[1]);
            logger->add("contact", Eigen::Vector2d(foot_contact[0], foot_contact[1]));
        }
    };
    
}

#endif