#include "robotrepresentation.h"

#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>

#define INFO_STREAM     ROS_INFO_STREAM


RobotRepresentation::RobotRepresentation(const std::string& name = "defaultRobot")
{
    m_preferred_arm_set = false;
    m_n_joints_in_chain = -1;
    m_name = name;
    m_whisker_state.assign(8,0.0);
}

void RobotRepresentation::setWhiskersState(std::vector<float> whisker_state)
{
    for(int i=0; i<8; ++i)
    {
        m_whisker_state[i] = whisker_state[i];
    }
}

bool RobotRepresentation::selfCheck()
{

    if(!m_preferred_arm_set)
    {
        INFO_STREAM("In RobotConfig of " << m_name << " preferred arm was not set");
        return false;
    }

    if(m_n_joints_in_chain <= 0)
        INFO_STREAM("In RobotConfig of " << m_name << " kinematic chain contains no joints");

    return true;
}

void RobotRepresentation::loadUrdfFromFile(const std::string& filename)
{
    m_urdf_model.clear();
    if ( !m_urdf_model.initFile(filename) )
        INFO_STREAM("Could not initialize urdf model from file: " << filename);

    return;
}

void RobotRepresentation::loadKinematicTreeFromUrdf()
{
    if (!kdl_parser::treeFromUrdfModel(m_urdf_model, m_kinematic_tree))
        INFO_STREAM("Turning urdf model into kdl tree failed!");
    else
        INFO_STREAM("Urdf model has been turned in kdl tree.");
}

void RobotRepresentation::loadKinematicChainFromTree(const std::string& root_link_name, const std::string& tip_link_name)
{
    m_root_link_name = root_link_name;
    m_tip_link_name = tip_link_name;

    if (!m_kinematic_tree.getChain(root_link_name, tip_link_name, m_kinematic_chain))
    {
        INFO_STREAM("Could not initialize chain object");
        return;
    }

    INFO_STREAM("Kinematic chain initialized from tree");

    m_n_joints_in_chain = m_kinematic_chain.getNrOfJoints();

}

void RobotRepresentation::loadJointLimits()
{
    m_q_min.resize(m_n_joints_in_chain);
    m_q_max.resize(m_n_joints_in_chain);
    m_q_seed.resize(m_n_joints_in_chain);
    m_q_joint_names.resize(m_n_joints_in_chain);

    int j=0;

    for(int i = 0; i < m_kinematic_chain.getNrOfSegments(); ++i)
    {
        const KDL::Joint& kdl_joint = m_kinematic_chain.getSegment(i).getJoint();

        if (kdl_joint.getType() != KDL::Joint::None)
        {
            m_q_joint_names[j] = kdl_joint.getName();

            boost::shared_ptr<const urdf::Joint> urdf_joint = m_urdf_model.getJoint(kdl_joint.getName());

            urdf_joint = m_urdf_model.getJoint(kdl_joint.getName());

            if (urdf_joint && urdf_joint->limits)
            {
                m_q_min(j) = urdf_joint->limits->lower;
                m_q_max(j) = urdf_joint->limits->upper;
                m_q_seed(j) = ( m_q_min(j) + m_q_max(j) ) / 2;
            }
            else
            {
                m_q_min(j) = -1e9;
                m_q_max(j) = 1e9;
                m_q_seed(j) = 0.0;
            }
            ++j;
        }
    }
}

void RobotRepresentation::setLeftArmIsPreferred()
{
    m_prefer_left_arm = true;
    m_preferred_arm_set = true;
}

void RobotRepresentation::setRightArmIsPreferred()
{
    m_prefer_left_arm = false;
    m_preferred_arm_set = true;
}

bool RobotRepresentation::isLeftArmPreferred()
{
    return m_prefer_left_arm;
}

bool RobotRepresentation::isRightArmPreferred()
{
    return !m_prefer_left_arm;
}

void RobotRepresentation::setInitialPoseJointRefs(std::vector<float> joint_refs)
{
    if(joint_refs.size() != m_n_joints_in_chain)
    {
        INFO_STREAM("number of joints monitoring is " << m_n_joints_in_chain << ", trying to set initial pose with " << joint_refs.size() << " joints");
        return;
    }

    m_q_initial_pose.resize(m_n_joints_in_chain);

    for(int i = 0; i< m_n_joints_in_chain; ++i)
        m_q_initial_pose(i) = joint_refs.at(i);
}

void RobotRepresentation::printAll()
{
    std::stringstream tmp_stream;

    INFO_STREAM("Robot name: " << m_name);

    if (m_preferred_arm_set && m_prefer_left_arm)
        INFO_STREAM("Preferred arm is set to left arm.");
    else if (m_preferred_arm_set && !m_prefer_left_arm)
        INFO_STREAM("Preferred arm is set to right arm.");
    else
        INFO_STREAM("Preferred arm was not set!");

    INFO_STREAM("KDL tree:");

    INFO_STREAM("\tNumber of Joints: " << m_kinematic_tree.getNrOfJoints() );
    INFO_STREAM("\tNumber of Segments: " << m_kinematic_tree.getNrOfSegments() );

    INFO_STREAM("KDL chain:");

    INFO_STREAM("\tRoot link: " << m_root_link_name);
    INFO_STREAM("\tTip link: " << m_tip_link_name);
    INFO_STREAM("\tNumber of Joints: " << m_kinematic_chain.getNrOfJoints() );
    INFO_STREAM("\tNumber of Segments: " << m_kinematic_chain.getNrOfSegments() );

    tmp_stream.str(""); tmp_stream << "Jointnames:";
    for(int i=0;i<m_n_joints_in_chain;++i)
        tmp_stream << std::endl << "\t\t\t\t\t " << m_q_joint_names.at(i);

    INFO_STREAM(tmp_stream.str());

    tmp_stream.str(""); tmp_stream << "Joint min:";
    for(int i=0;i<m_n_joints_in_chain;++i)
        tmp_stream << std::endl << "\t\t\t\t\t " << m_q_min.data[i];

    INFO_STREAM(tmp_stream.str());

    tmp_stream.str(""); tmp_stream << "Joint max:";
    for(int i=0;i<m_n_joints_in_chain;++i)
        tmp_stream << std::endl << "\t\t\t\t\t " << m_q_max.data[i];

    INFO_STREAM(tmp_stream.str());

    tmp_stream.str(""); tmp_stream << "Joint seeds:";
    for(int i=0;i<m_n_joints_in_chain;++i)
        tmp_stream << std::endl << "\t\t\t\t\t " << m_q_seed.data[i];

    INFO_STREAM(tmp_stream.str());

}

RobotRepresentation::~RobotRepresentation()
{
    // destructor
}
