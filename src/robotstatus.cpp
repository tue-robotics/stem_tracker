#include "robotstatus.h"

RobotStatus::RobotStatus(int n_joints_to_monitor)
{
    if(n_joints_to_monitor <= 0){
        INFO_STREAM("trying to initialize robot status object with zero or negative number of joints to monitor!");
    }
    m_joints_to_monitor = KDL::JntArray(n_joints_to_monitor);
    m_n_joints_monitoring = n_joints_to_monitor;
}

bool RobotStatus::selfCheck(){

    bool IamOK = true;

    if(m_n_joints_monitoring <= 0){
        INFO_STREAM("trying to use robot status object while number of joints to monitor is zero or negative!");
        IamOK = false;
    }

    return IamOK;
}

void RobotStatus::receivedTorsoMsg(const sensor_msgs::JointState & msg){

    if(!selfCheck()){
        return;
    }

    m_last_update = ros::Time::now();

    m_joints_to_monitor(0) = msg.position[0];
}

void RobotStatus::receivedArmMsg(const sensor_msgs::JointState & msg){

    if(!selfCheck()){
        return;
    }

    for(int i = 1; i < m_n_joints_monitoring; ++i){
        m_joints_to_monitor(i) = msg.position[i-1];
    }

    m_last_update = ros::Time::now();

}

ros::Time RobotStatus::getLastUpdateTime(){
    return m_last_update;
}

double RobotStatus::getTimeSinceLastUpdate(){
    // returns n seconds since last update in any of
    // the joints we are monitoring
    ros::Duration interval = ros::Time::now() - RobotStatus::getLastUpdateTime();
    return interval.toSec();
}

bool RobotStatus::isUpToDate(){
    if(getTimeSinceLastUpdate() < m_up_to_date_threshold){
        return true;
    } else {
        return false;
    }
}

KDL::JntArray RobotStatus::getJointStatus(){
    return m_joints_to_monitor;
}

void RobotStatus::setUpToDateThreshold(double threshold){
    m_up_to_date_threshold = threshold;
}

std::vector<float> RobotStatus::getGripperXYZ(RobotConfig* robot_config ){

    m_gripper_xyz.clear();

    KDL::ChainFkSolverPos_recursive forward_kinematics_solver = KDL::ChainFkSolverPos_recursive(robot_config->getKinematicChain());

    KDL::Frame cartpos;
    int fk_ret;

    fk_ret = forward_kinematics_solver.JntToCart(getJointStatus(),cartpos);

    m_gripper_xyz.push_back(cartpos.p.x());
    m_gripper_xyz.push_back(cartpos.p.y());
    m_gripper_xyz.push_back(cartpos.p.z());

    if( fk_ret < 0 ){
        INFO_STREAM("Warning: something wrong in solving forward kinematics in getGripperXYZ");
    }

    return m_gripper_xyz;

}

bool RobotStatus::isGripperXYZvalid(){
    if(m_gripper_xyz.size() == 3){
        return true;
    } else {
        return false;
    }
}

void RobotStatus::printAll(){

    INFO_STREAM("===============");
    INFO_STREAM("Robot status: ");
    //    INFO_STREAM("\t initialized with robot config of: " << m_robot_config.getName());
    for(int i; i<m_n_joints_monitoring; ++i){
        INFO_STREAM("\t" << m_joints_to_monitor(i));
    }
    INFO_STREAM("===============");
}

RobotStatus::~RobotStatus(){
    // destructor
}
