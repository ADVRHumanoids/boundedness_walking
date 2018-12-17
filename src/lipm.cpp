#include <multidof_walking/lti/lipm.h>

namespace mdof {
    
OpenMpC::dynamics::LtiDynamics::Ptr MakeLipmContinuousTime(double omega)
{
    Eigen::Matrix2d Alipm;
    Eigen::Vector2d Blipm;
    
    Alipm <<      0     , 1,
             omega*omega, 0;
             
    Blipm <<      0      , 
             -omega*omega;
             
    auto lipm = boost::make_shared<OpenMpC::dynamics::LtiDynamics>(Alipm, Blipm);
    
    Eigen::MatrixXd Cpos(1,2);
    Eigen::MatrixXd Cvel(1,2);
    Cpos << 1, 0;
    Cvel << 0, 1;
    
    lipm->addOutput("pos", Cpos);
    lipm->addOutput("vel", Cvel);
    
    return lipm;
}

OpenMpC::dynamics::LtiDynamics::Ptr MakeLipmExtended(double omega, double dt)
{
    auto lipm_dt = mdof::MakeLipmContinuousTime(omega)->makeDiscreteTime(dt);
    
    Eigen::MatrixXd Alipm_dt_delay_incr(3,3);
    Alipm_dt_delay_incr << lipm_dt->getA(), lipm_dt->getB(),
                              0    ,   0  ,        1       ;
     
    Eigen::Vector3d Blipm_dt_delay_incr(0, 0, 1);
    
    auto ext =  boost::make_shared<OpenMpC::dynamics::LtiDynamics>(Alipm_dt_delay_incr, 
                                                               Blipm_dt_delay_incr, 
                                                               true);
    
    Eigen::MatrixXd Cpos(1,3);
    Eigen::MatrixXd Cvel(1,3);
    Eigen::MatrixXd Czmp(1,3);
    Eigen::MatrixXd Cdeltafoot(1,3);
    Cpos  << 1, 0, 0;
    Cvel  << 0, 1, 0;
    Czmp  << 0, 0, 1;
    Cdeltafoot << -1.0, 0.0, 1.0;
    
    ext->addOutput("pos", Cpos);
    ext->addOutput("vel", Cvel);
    ext->addOutput("zmp", Czmp);
    ext->addOutput("delta_foot", Cdeltafoot);
    
    return ext;
    
}

}