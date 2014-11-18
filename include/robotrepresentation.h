#ifndef ROBOTREPRESENTATION_H
#define ROBOTREPRESENTATION_H

#include <string>
#include <vector>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <urdf/model.h>

class RobotRepresentation
{
private:
    std::string m_name;             // name of robot instance
    urdf::Model m_urdf_model;       // urdf formatted model
    bool m_preferred_arm_set;
    bool m_prefer_left_arm;         // preferring left arm if true, right arm if false
    KDL::Tree m_kinematic_tree;
    KDL::Chain m_kinematic_chain;
    int m_n_joints_in_chain;
    bool m_initial_pose_set;
    KDL::JntArray m_q_min, m_q_max, m_q_seed, m_q_initial_pose;
    std::vector<std::string> m_q_joint_names;
    std::string m_root_link_name;
    std::string m_tip_link_name;

public:
    RobotRepresentation(const std::string name);
    bool selfCheck();
    const KDL::JntArray& getJointMinima() const { return m_q_min; }
    const KDL::JntArray& getJointMaxima() const { return m_q_max; }
    const KDL::JntArray& getJointSeeds() const { return m_q_seed; }
    const std::vector<std::string>& getJointNames() const { return m_q_joint_names; }
    void loadUrdfFromFile(const std::string filename);
    urdf::Model getUrdfModel();
    void loadKinematicTreeFromUrdf();
    void loadKinematicChainFromTree(const std::string root_link_name, const std::string tip_link_name);
    void loadJointLimits();
    KDL::Tree getKinematicTree();
    KDL::Chain getKinematicChain();
    void setLeftArmIsPreferred();
    void setRightArmIsPreferred();
    bool isLeftArmPreferred();
    bool isRightArmPreferred();
    const std::string getName();
    void setInitialPoseJointRefs(std::vector<float> joint_refs);
    KDL::JntArray getInitialPoseJointRefs();
    void printAll();

    ~RobotRepresentation();
};

#endif // ROBOTREPRESENTATION_H
