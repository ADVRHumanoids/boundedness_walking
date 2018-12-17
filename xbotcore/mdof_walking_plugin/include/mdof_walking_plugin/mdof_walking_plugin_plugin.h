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

#include <XCM/XBotControlPlugin.h>

#include <multidof_walking/wpg/lqr_walker.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>

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
    
    void set_world_pose();

    XBot::RobotInterface::Ptr _robot;
    XBot::ModelInterface::Ptr _model;

    double _start_time;

    Eigen::VectorXd _q, _qdot;

    XBot::MatLogger::Ptr _logger;
    
    std::vector<std::string> _feet_links;
    mdof::Walker::Ptr _walker;
    mdof::RobotState _state, _ref;
    Eigen::Affine3d _T_lsole, _T_rsole;
    
    XBot::Cartesian::CartesianInterfaceImpl::Ptr _ci;
    

};

}

#endif // mdof_walking_plugin_PLUGIN_H_
