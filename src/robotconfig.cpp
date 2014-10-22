#include "robotconfig.h"

RobotConfig::RobotConfig(const std::string name = "defaultRobot")
{
    m_preferred_arm_set = false;
    m_n_joints_in_chain = -1;
    m_name = name;
}


bool RobotConfig::selfCheck(){

    if(!m_preferred_arm_set){
        INFO_STREAM("In RobotConfig of " << m_name << " preferred arm was not set");
        return false;
    }

    if(m_n_joints_in_chain <= 0){
        INFO_STREAM("In RobotConfig of " << m_name << " kinematic chain contains no joints");
    }

    return true;
}

/* get robot description from ros parameter server */
void RobotConfig::loadUrdfFromRosparam(ros::NodeHandle n, const std::string urdf_rosparam){

    std::string urdf;
    if (!n.getParam(urdf_rosparam, urdf)) {
        INFO_STREAM("Loading of robot urdf from rosparam \"" << urdf_rosparam << "\" failed!");
        return;
    }
    else{
        INFO_STREAM("Urdf xml loaded from rosparam \"" << urdf_rosparam << "\"");
    }

    if (!m_urdf_model.initString(urdf))
    {
        INFO_STREAM("Could not initialize urdf model from xml");
    }
}

urdf::Model RobotConfig::getUrdfModel(){
    return m_urdf_model;
}

void RobotConfig::loadKinematicTreeFromUrdf(){

    if (!kdl_parser::treeFromUrdfModel(m_urdf_model, m_kinematic_tree)) {
        INFO_STREAM("Turning urdf model into kdl tree failed!");
    }
    else{
        INFO_STREAM("Urdf model has been turned in kdl tree.");
    }
}

void RobotConfig::loadKinematicChainFromTree(const std::string root_link_name, const std::string tip_link_name){

    if (!m_kinematic_tree.getChain(root_link_name, tip_link_name, m_kinematic_chain)){
        INFO_STREAM("Could not initialize chain object");
    }
    INFO_STREAM("Kinematic chain initialized from tree");

    m_n_joints_in_chain = m_kinematic_chain.getNrOfJoints();
}

void RobotConfig::loadJointLimits(){

    m_q_min.resize(m_n_joints_in_chain);
    m_q_max.resize(m_n_joints_in_chain);
    m_q_joint_names.resize(m_n_joints_in_chain);

    int j=0;

    for(int i = 0; i < m_kinematic_chain.getNrOfSegments(); ++i){

        const KDL::Joint& kdl_joint = m_kinematic_chain.getSegment(i).getJoint();

        if (kdl_joint.getType() != KDL::Joint::None){

            m_q_joint_names[j] = kdl_joint.getName();

            boost::shared_ptr<const urdf::Joint> urdf_joint = m_urdf_model.getJoint(kdl_joint.getName());

            urdf_joint = m_urdf_model.getJoint(kdl_joint.getName());

            if (urdf_joint && urdf_joint->limits){
                m_q_min(j) = urdf_joint->limits->lower;
                m_q_max(j) = urdf_joint->limits->upper;
            } else{
                m_q_min(j) = -1e9;
                m_q_max(j) = 1e9;
            }
            ++j;
        }
    }

}

KDL::Tree RobotConfig::getKinematicTree(){
    return m_kinematic_tree;
}

KDL::Chain RobotConfig::getKinematicChain(){
    return m_kinematic_chain;
}

void RobotConfig::setLeftArmIsPreferred(){
    m_prefer_left_arm = true;

    m_arm_joint_msg.name.clear();
    m_arm_joint_msg.name.push_back("shoulder_roll_joint_left");
    m_arm_joint_msg.name.push_back("shoulder_pitch_joint_left");
    m_arm_joint_msg.name.push_back("shoulder_yaw_joint_left");
    m_arm_joint_msg.name.push_back("elbow_roll_joint_left");
    m_arm_joint_msg.name.push_back("elbow_pitch_joint_left");
    m_arm_joint_msg.name.push_back("wrist_pitch_joint_left");
    m_arm_joint_msg.name.push_back("wrist_yaw_joint_left");

    m_preferred_arm_set = true;
}

void RobotConfig::setRightArmIsPreferred(){
    m_prefer_left_arm = false;

    m_arm_joint_msg.name.clear();
    m_arm_joint_msg.name.push_back("shoulder_roll_joint_right");
    m_arm_joint_msg.name.push_back("shoulder_pitch_joint_right");
    m_arm_joint_msg.name.push_back("shoulder_yaw_joint_right");
    m_arm_joint_msg.name.push_back("elbow_roll_joint_right");
    m_arm_joint_msg.name.push_back("elbow_pitch_joint_right");
    m_arm_joint_msg.name.push_back("wrist_pitch_joint_right");
    m_arm_joint_msg.name.push_back("wrist_yaw_joint_right");

    m_preferred_arm_set = true;
}

bool RobotConfig::isLeftArmPreferred(){
    return m_prefer_left_arm;
}

bool RobotConfig::isRightArmPreferred(){
    return !m_prefer_left_arm;
}

const std::string RobotConfig::getName(){
    return m_name;
}

sensor_msgs::JointState RobotConfig::getInitialPoseMsg(){

    m_arm_joint_msg.header.stamp = ros::Time::now();
    m_arm_joint_msg.position.clear();

    /* amigo 'give' position */
    m_arm_joint_msg.position.push_back(0.0);
    m_arm_joint_msg.position.push_back(0.4);
    m_arm_joint_msg.position.push_back(-0.1);
    m_arm_joint_msg.position.push_back(0.0);
    m_arm_joint_msg.position.push_back(1.2);
    m_arm_joint_msg.position.push_back(0.0);
    m_arm_joint_msg.position.push_back(0.0);

    return m_arm_joint_msg;

}

void RobotConfig::printAll(){

    std::stringstream tmp_stream;

    INFO_STREAM("===============");
    INFO_STREAM("Robot name: " << m_name);

    if (m_preferred_arm_set && m_prefer_left_arm){
        INFO_STREAM("Preferred arm is set to left arm.");
    } else if (m_preferred_arm_set && !m_prefer_left_arm){
        INFO_STREAM("Preferred arm is set to right arm.");
    } else {
        INFO_STREAM("Preferred arm was not set!");
    }

    INFO_STREAM("KDL tree:");
    INFO_STREAM("\tNumber of Joints: " << m_kinematic_tree.getNrOfJoints() );
    INFO_STREAM("\tNumber of Segments: " << m_kinematic_tree.getNrOfSegments() );
    INFO_STREAM("KDL chain:");
    INFO_STREAM("\tNumber of Joints: " << m_kinematic_chain.getNrOfJoints() );
    INFO_STREAM("\tNumber of Segments: " << m_kinematic_chain.getNrOfSegments() );

    tmp_stream.str(""); tmp_stream << "Jointnames:";
    for(int i=0;i<m_n_joints_in_chain;++i){
        tmp_stream << std::endl << "\t\t\t\t\t " << m_q_joint_names.at(i);
    }
    INFO_STREAM(tmp_stream.str());

    tmp_stream.str(""); tmp_stream << "Joint min:";
    for(int i=0;i<m_n_joints_in_chain;++i){
        tmp_stream << std::endl << "\t\t\t\t\t " << m_q_min.data[i];
    }
    INFO_STREAM(tmp_stream.str());

    tmp_stream.str(""); tmp_stream << "Joint max:";
    for(int i=0;i<m_n_joints_in_chain;++i){
        tmp_stream << std::endl << "\t\t\t\t\t " << m_q_max.data[i];
    }
    INFO_STREAM(tmp_stream.str());

    INFO_STREAM("===============");

}

RobotConfig::~RobotConfig(){
    // destructor
}
