#include "whiskerinterpreter.h"

WhiskerInterpreter::WhiskerInterpreter(int gripper_id)
{
    m_gripper_id = gripper_id;
    m_status = 0;
}

int WhiskerInterpreter::getGripperID()
{
    return m_gripper_id;
}

void WhiskerInterpreter::setNumberOfWhiskers(int n_whiskers)
{
    m_n_whiskers = n_whiskers;
}

void WhiskerInterpreter::setGripperDiameter(float gripper_diameter)
{
    m_gripper_diameter = gripper_diameter;
    m_gripper_radius = gripper_diameter / 2.0f;
}

void WhiskerInterpreter::setWhiskerLength(float whisker_length)
{
    m_whisker_length = whisker_length;
}

void WhiskerInterpreter::setMaxWhiskerForce(float max_whisker_force)
{
    m_max_whisker_force = max_whisker_force;
}

int WhiskerInterpreter::getStatus()
{
    return m_status;
}

bool WhiskerInterpreter::selfCheck()
{

    bool IamOK = true;

    if (m_n_whiskers <= 0)
    {
        INFO_STREAM("In whisker gripper with id " << m_gripper_id << " number of whiskers set to zero or negative number");
        IamOK = false;
    }

    if (m_whisker_length > m_gripper_radius)
    {
        INFO_STREAM("in whisker gripper with id " << m_gripper_id << " length of whiskers is larger than radius of gripper");
        IamOK = false;
    }

    return IamOK;
}

void WhiskerInterpreter::simulateWhiskerGripper(std::vector<float> gripper_center, std::vector<float> stem_center)
{

    /* takes two doubles (xy) as coordinates of the gripper two doubles as coordinate of
             * the stem (both are in the same z-plane).
             * returns a force (xy) with origin at gripper center */

    m_whisker_force.clear();
    m_estimated_pos_error.clear();

    if(gripper_center.size() < 2 )
    {
        INFO_STREAM("in simulateWhiskerGripper gripper center xy needed!");
        m_status = 0;
        return;
    }

    if(stem_center.size() < 2 )
    {
        INFO_STREAM("in simulateWhiskerGripper stem center xy needed!");
        m_status = 0;
        return;
    }

    m_whisker_force.assign(2,0.0);
    m_estimated_pos_error.assign(2,0.0);

    m_estimated_pos_error[0] = gripper_center[0] - stem_center[0];
    m_estimated_pos_error[1] = gripper_center[1] - stem_center[1];

    float dist_gripper_to_stem = sqrt( m_estimated_pos_error[0] * m_estimated_pos_error[0] + m_estimated_pos_error[1] * m_estimated_pos_error[1] );

    if( dist_gripper_to_stem > m_gripper_radius )
    {
        m_status = 1;
    }
    else if (dist_gripper_to_stem < m_gripper_radius - m_whisker_length)
    {
        m_status = 3;
    }
    else
    {
        m_status = 2;

        /* simulated force is inverse of distance to force */
        float whisker_fraction_deformed = ( dist_gripper_to_stem - (m_gripper_radius - m_whisker_length) ) / m_whisker_length;
        m_whisker_force.at(0) = (m_estimated_pos_error[0] / dist_gripper_to_stem) * m_max_whisker_force * whisker_fraction_deformed;
        m_whisker_force.at(1) = (m_estimated_pos_error[1] / dist_gripper_to_stem) * m_max_whisker_force * whisker_fraction_deformed;
    }

}

std::vector<float> WhiskerInterpreter::getXYerror()
{
    // to do: turn measured force/deflection into estimation of position error
    return m_estimated_pos_error;
}

std::vector<float> WhiskerInterpreter::getWhiskerNetForce()
{
    return m_whisker_force;
}

void WhiskerInterpreter::showForceInRviz(ros::Publisher* p_vis_pub, std::vector<float> gripper_xyz)
{

    if(!selfCheck()){
        return;
    }

    if(gripper_xyz.size() != 3){
        INFO_STREAM("need gripper location xyz to visualize whisker force!");
        return;
    }

    if(m_whisker_force.size() != 2){
        INFO_STREAM("trying to show a force vector that does not exist!");
        return;
    }

    /* construct line strip marker object */
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/amigo/base_link";
    marker.header.stamp = ros::Time().now();
    marker.id = 3;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.ns = "whisker_force";
    marker.scale.x = 0.01;
    marker.scale.y = 0.02;
    marker.color.a = 1.0f;
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;

    /* construct nodes point */
    geometry_msgs::Point p_start, p_end;

    p_start.x = gripper_xyz.at(0) - m_whisker_force.at(0);
    p_start.y = gripper_xyz.at(1) - m_whisker_force.at(1);
    p_start.z = gripper_xyz.at(2);
    marker.points.push_back(p_start);

    p_end.x = gripper_xyz.at(0);
    p_end.y = gripper_xyz.at(1);
    p_end.z = gripper_xyz.at(2);
    marker.points.push_back(p_end);

    /* publish marker */
    p_vis_pub->publish( marker );

}

WhiskerInterpreter::~WhiskerInterpreter()
{
    //destructor
}
