/*
 * Copyright (C) 2017 IIT-ADVR
 * Author:
 * email:
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#ifndef mdof_walking_plugin_PLUGIN_H_
#define mdof_walking_plugin_PLUGIN_H_

#include <atomic>

#include <XCM/XBotControlPlugin.h>
#include <XBotCore-interfaces/XBotRosUtils.h>

#include <cartesian_interface/CartesianInterfaceImpl.h>

#include <multidof_walking/wpg/simple_walker.h>
#include <multidof_walking/controllers/momentum.h>

#include <geometry_msgs/TwistStamped.h>
#include <std_srvs/SetBool.h>

namespace XBotPlugin {

/**
 * @brief mdof_walking_plugin XBot RT Plugin
 *
 **/
class mdof_walking_plugin : public XBot::XBotControlPlugin
{

public:

    virtual bool init_control_plugin(XBot::Handle::Ptr handle);

    virtual bool close();

    virtual void on_start(double time);

    virtual void on_stop(double time);
    
    virtual ~mdof_walking_plugin();

protected:

    virtual void control_loop(double time, double period);

private:
    
    enum class State 
    {
        IDLE, HOMING, WALKING
    };
    
    void run_idle(double time, double period);
    void run_homing(double time, double period);
    void run_walking(double time, double period);
    void run_momentum(double time, double period);

    void set_world_pose();
    Eigen::Vector3d get_lfoot_pos() const;
    Eigen::Vector3d get_rfoot_pos() const;
    
    void on_vref_recv(const geometry_msgs::TwistStampedConstPtr& msg);
    bool on_start_stop_requested(std_srvs::SetBoolRequest& req, 
                                 std_srvs::SetBoolResponse& res);


    State _current_state;
    std::atomic<bool> _started;
    
    XBot::RosUtils::RosHandle::Ptr _ros_handle;
    XBot::RosUtils::SubscriberWrapper::Ptr _vref_sub;
    XBot::RosUtils::ServiceServerWrapper::Ptr _start_srv;
    
    
    XBot::RobotInterface::Ptr _robot;
    XBot::ModelInterface::Ptr _model;
    
    mdof::MomentumController::Ptr _momentum;

    double _start_time;
    double _time;
    double _period;

    Eigen::VectorXd _q, _qdot;

    XBot::MatLogger::Ptr _logger;
    
    std::vector<std::string> _feet_links;
    mdof::Walker::Ptr _walker;
    mdof::RobotState _state, _ref;
    double _vref;
    Eigen::Vector3d _delta_lfoot_2, _delta_lfoot_3, _delta_rfoot_1, _delta_rfoot_4;
    
    XBot::Cartesian::CartesianInterfaceImpl::Ptr _ci;
    

};

}

#endif // mdof_walking_plugin_PLUGIN_H_
