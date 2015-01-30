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
    std::string m_name;
    urdf::Model m_urdf_model;
    bool m_preferred_arm_set;
    bool m_prefer_left_arm;
    KDL::Tree m_kinematic_tree;
    KDL::Chain m_kinematic_chain;
    int m_n_joints_in_chain;
    KDL::JntArray m_q_min, m_q_max, m_q_seed, m_q_initial_pose;
    std::vector<std::string> m_q_joint_names;
    std::string m_root_link_name;
    std::string m_tip_link_name;

public:
    RobotRepresentation(const std::string& name);

    void printAll();

    void setInitialPoseJointRefs(const std::vector<float> joint_refs);
    void setLeftArmIsPreferred();
    void setRightArmIsPreferred();

    bool isLeftArmPreferred();
    bool isRightArmPreferred();

    const KDL::JntArray& getJointMinima() const { return m_q_min; }
    const KDL::JntArray& getJointMaxima() const { return m_q_max; }
    const KDL::JntArray& getJointSeeds() const { return m_q_seed; }
    const std::vector<std::string>& getJointNames() const { return m_q_joint_names; }
    const urdf::Model& getUrdfModel() const { return m_urdf_model; }
    const KDL::Tree& getKinematicTree() const { return m_kinematic_tree; }
    const KDL::Chain& getKinematicChain() const { return m_kinematic_chain; }
    const std::string& getName() const { return m_name; }
    const KDL::JntArray& getInitialPoseJointRefs() const { return m_q_initial_pose; }

    void loadUrdfFromFile(const std::string& filename);
    void loadKinematicTreeFromUrdf();
    void loadKinematicChainFromTree(const std::string& root_link_name, const std::string& tip_link_name);
    void loadJointLimits();

    ~RobotRepresentation();
};

#endif // ROBOTREPRESENTATION_H
