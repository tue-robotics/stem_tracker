#include "stemtrackmonitor.h"

#include "stemrepresentation.h"
#include "robotrepresentation.h"
#include "robotstatus.h"
#include "stemtrackcontroller.h"
#include "whiskergripperinterpreter.h"
#include "stemtrackconfigurer.h"
#include "robotinterface.h"
#include "visualizationinterface.h"

#include "loggingmacros.h"
#include "debugfunctions.h"


const std::string StemTrackMonitor::stateToString(stemtrack_state_t state) const
{
    switch( state )
    {
    case INIT:
        return "INIT";
    case CALIBRATE:
        return "CALIBRATE";
    case PREPOS:
        return "PREPOS";
    case GRASP:
        return "GRASP";
    case FOLLOW:
        return "FOLLOW";
    case LOST:
        return "LOST";
    case END:
        return "END";
    case SIDEBRANCH:
        return "SIDEBRANCH";
    case ERROR:
        return "ERROR";
    default:
        WARNING_STREAM("Unknown state in StemTrackMonitor::stateToString!");
        return "UNKNOWN STATE!";
    }
}

void StemTrackMonitor::updateState()
{
    stemtrack_state_t state_prev = m_state;

    switch (m_state)
    {
    case INIT:
        m_state = PREPOS;
        break;

    case PREPOS:
        if( m_p_robot_status->reachedPosition( m_p_stem_representation->getStemTrackingStartXYZ() ) && !m_find_max_touched_values)
        {
            m_state = CALIBRATE;
            INFO_STREAM("=============================================");
            INFO_STREAM("==> Arrived at my pre position, going to calibrate now");
        }
        break;

    case CALIBRATE:
        if( m_p_whisker_gripper_interpreter->isInitialized())
        {
            m_state = GRASP;
            INFO_STREAM("=============================================");
            INFO_STREAM("==> Obtained nominal whisker values, going to grasp the stem now");
        }
        break;

    case GRASP:
        if( m_p_robot_status->getGripperXYZ()[0] - m_p_stem_representation->getStemTrackingStartXYZ()[0] > 0.14 )
        {
            m_state = FOLLOW;
            INFO_STREAM("=============================================");
            INFO_STREAM("==> I have the stem, going to move up now");
        }
        break;

    case FOLLOW:
        if( m_p_whisker_gripper_interpreter->pressureSensorIsTouched() )
        {
            m_state = END;
            INFO_STREAM("=============================================");
            INFO_STREAM("==> I am done with my task");
        }
        break;
    }

    if(m_debug_state_par && m_state != state_prev)
        INFO_STREAM("In stemtrack monitor, state was: " << stateToString(state_prev) << " now set to: " << stateToString(m_state) << ".");
}

void StemTrackMonitor::reconfigureAndReset()
{
    m_p_stemtrack_configurer->configureStemRepresentation(*m_p_stem_representation);
    m_p_stemtrack_configurer->configureRobotRepresentation(*m_p_robot_representation);
    m_p_stemtrack_configurer->configureRobotStatus(*m_p_robot_status);
    m_p_stemtrack_configurer->configureWhiskerGripperInterpreter(*m_p_whisker_gripper_interpreter);
    m_p_stemtrack_configurer->configureStemTrackController(*m_p_stemtrack_control);
    m_p_stemtrack_configurer->configureRobotInterface(*m_p_robot_interface);
    m_p_stemtrack_configurer->configureStemTrackMonitor(*this);
    m_p_stemtrack_configurer->configureVisualizationInterface(*m_p_visualization_interface);
    m_p_whisker_gripper_interpreter->resetInitialization();
    m_p_robot_status->resetUpToDateStatus();

    m_state = INIT;

    return;
}

bool StemTrackMonitor::inputIsUpToDate()
{

    if(!m_p_robot_status->jointStatusIsUpToDate())
    {
        if(m_prev_sample_joint_status_up_to_date)
            INFO_STREAM("Waiting for up to date joint status information");
        m_prev_sample_joint_status_up_to_date = false;
    }
    else
    {
        m_prev_sample_joint_status_up_to_date = true;
    }

    if(!m_p_robot_status->gripperSensingIsUpToDate())
    {
        if(m_prev_sample_gripper_sensing_up_to_date)
            INFO_STREAM("Waiting for up to date gripper status information");
        m_prev_sample_gripper_sensing_up_to_date = false;
    }
    else
    {
        m_prev_sample_gripper_sensing_up_to_date = true;
    }

    if(!m_prev_sample_gripper_sensing_up_to_date || !m_prev_sample_joint_status_up_to_date)
        return false;
    else
        return true;
}

void StemTrackMonitor::doPreposBehavior()
{
    m_trial_is_done = false;

    /* go to cartesian prepos */
    m_p_stemtrack_control->setCartSetpoint(m_p_stem_representation->getStemTrackingStartXYZ());
    /* translate cartesian setpoint to joint coordinates */
    m_p_stemtrack_control->updateJointPosReferences();
    /* send references to joint controllers */
    m_p_robot_interface->publishAmigoJointPosRefs(m_p_stemtrack_control->getJointPosRefs());

    m_p_robot_interface->publishAmigoOpenGripperMessage();

    if(m_find_max_touched_values)
    {
        m_p_whisker_gripper_interpreter->findPressureSensorMaxTouchedValues();
        m_p_whisker_gripper_interpreter->findWhiskerMaxTouchedValues();
        m_p_stemtrack_configurer->storePressureSensorTouchedMaxValues( m_p_whisker_gripper_interpreter->getPressureSensorTouchedMax() );
        m_p_stemtrack_configurer->storeWhiskerTouchedMaxValues( m_p_whisker_gripper_interpreter->getWhiskersTouchedMax() );
    }

    return;
}

void StemTrackMonitor::doGraspBehavior()
{
    m_p_whisker_gripper_interpreter->updateWhiskerInterpretation();
    m_p_visualization_interface->showArrows( m_p_robot_status->gripperFrameVectorsToBaseFrameVectors(
                                                 m_p_whisker_gripper_interpreter->getTouchedWhiskerVectorTips() ),
                                             m_p_robot_status->gripperFrameVectorsToBaseFrameVectors(
                                                 m_p_whisker_gripper_interpreter->getTouchedWhiskerVectorOrigins() ), whisker_touch );

    /* set reference to forward in same plane */
    m_p_stemtrack_control->setPointMoveForward(m_p_robot_status->getGripperXYZ(),0.83);

    /* translate cartesian setpoint to joint coordinates */
    m_p_stemtrack_control->updateJointPosReferences();

    /* send references to joint controllers */
    m_p_robot_interface->publishAmigoJointPosRefs(m_p_stemtrack_control->getJointPosRefs());

    return;
}

void StemTrackMonitor::doFollowBehavior()
{
    m_p_visualization_interface->showXYZ(m_p_robot_status->getGripperXYZ(), gripper_center);

    m_p_whisker_gripper_interpreter->updateWhiskerInterpretation();
    m_p_visualization_interface->showArrows( m_p_robot_status->gripperFrameVectorsToBaseFrameVectors(
                                                 m_p_whisker_gripper_interpreter->getTouchedWhiskerVectorTips() ),
                                             m_p_robot_status->gripperFrameVectorsToBaseFrameVectors(
                                                 m_p_whisker_gripper_interpreter->getTouchedWhiskerVectorOrigins() ), whisker_touch );

    /* add or remove stem nodes */
    if(m_p_whisker_gripper_interpreter->getTouchedWhiskerVectorTips().size() > 0)
        m_p_stem_representation->updateStemNodes( m_p_robot_status->gripperFrameVectorsToBaseFrameVectors( m_p_whisker_gripper_interpreter->getTouchedWhiskerVectorTips() ));
    else
        m_p_stem_representation->updateStemNodes( m_p_robot_status->getGripperXYZ());

    /* update stem tangent */
    m_p_stem_representation->updateTangent();
    m_p_visualization_interface->showArrow( m_p_stem_representation->getTangent(),
            m_p_robot_status->getGripperXYZ()[0] - 0.5 * m_p_visualization_interface->getStemTangentVectorElongation() * m_p_stem_representation->getTangent()[0],
            m_p_robot_status->getGripperXYZ()[1] - 0.5 * m_p_visualization_interface->getStemTangentVectorElongation() * m_p_stem_representation->getTangent()[1],
            m_p_robot_status->getGripperXYZ()[2] - 0.5 * m_p_visualization_interface->getStemTangentVectorElongation() * m_p_stem_representation->getTangent()[2], stem_tangent );

    /* update position setpoint in cartesian space */
    m_p_stemtrack_control->updateSetpointAndPose( m_p_whisker_gripper_interpreter->getEstimatedPosError() );
    m_p_visualization_interface->showXYZ(m_p_stemtrack_control->getCartSetpointXYZ(), cartesian_setpoint);
    if(m_p_stemtrack_configurer->debugDesiredGripperPose())
    {
        m_p_visualization_interface->showArrow(m_p_stemtrack_control->getDesiredGripperPoseVectors()[0],m_p_stemtrack_control->getCartSetpointXYZ(), red_debug_arrow);
        m_p_visualization_interface->showArrow(m_p_stemtrack_control->getDesiredGripperPoseVectors()[1],m_p_stemtrack_control->getCartSetpointXYZ(), green_debug_arrow);
        m_p_visualization_interface->showArrow(m_p_stemtrack_control->getDesiredGripperPoseVectors()[2],m_p_stemtrack_control->getCartSetpointXYZ(), blue_debug_arrow);
    }

    /* translate cartesian setpoint to joint coordinates */
    m_p_stemtrack_control->updateJointPosReferences();

    /* send references to joint controllers */
    m_p_robot_interface->publishAmigoJointPosRefs(m_p_stemtrack_control->getJointPosRefs());

    /* check if bumped into side branch */
    m_p_whisker_gripper_interpreter->checkForTopSensorTouched();

    return;
}

void StemTrackMonitor::doEndBehaviour()
{
    m_p_whisker_gripper_interpreter->updateWhiskerInterpretation();
    m_p_visualization_interface->showArrows( m_p_robot_status->gripperFrameVectorsToBaseFrameVectors(
                                                 m_p_whisker_gripper_interpreter->getTouchedWhiskerVectorTips() ),
                                             m_p_robot_status->gripperFrameVectorsToBaseFrameVectors(
                                                 m_p_whisker_gripper_interpreter->getTouchedWhiskerVectorOrigins() ), whisker_touch );
    if(!m_trial_is_done)
    {
        m_p_stem_representation->updateStemNodes(m_p_robot_status->getGripperXYZ(),true);
        m_p_stem_representation->updateStemNodes(
                    m_p_robot_status->getGripperXYZ()[0] + m_p_stemtrack_configurer->getAdditionalStemHeight() * m_p_stem_representation->getTangent()[0],
                    m_p_robot_status->getGripperXYZ()[1] + m_p_stemtrack_configurer->getAdditionalStemHeight() * m_p_stem_representation->getTangent()[1],
                    m_p_robot_status->getGripperXYZ()[2] + m_p_stemtrack_configurer->getAdditionalStemHeight() * m_p_stem_representation->getTangent()[2], true);
        m_trial_is_done = true;
    }

    m_p_visualization_interface->showTomatoTruss(m_p_whisker_gripper_interpreter->getEstimatedTopSensorTouchLocation(), m_p_robot_status->getGripperXYZ());

    return;
}

bool StemTrackMonitor::update()
{
    if ( m_p_stemtrack_configurer->configChanged() )
        reconfigureAndReset();

    if (m_p_stemtrack_configurer->configIsOk())
    {
        /* check for state transition */
        updateState();

        /* visualize stem */
        m_p_visualization_interface->showLineStrip(m_p_stem_representation->getNodesX(), m_p_stem_representation->getNodesY(),
                                                   m_p_stem_representation->getNodesZ(), stem);

        /* check for up to date joint and gripper status */
        if( inputIsUpToDate() )
        {

            /* safety check */
            if(!m_p_robot_status->hasValidGripperXYZ())
            {
                ERROR_STREAM("Gripper xyz unknown or not valid!");
                return false;
            }

            /* do state dependent behavior */
            switch(m_state)
            {

            case PREPOS:
                /* bring arm to preposition */
                doPreposBehavior();
                break;

            case CALIBRATE:
                /* obtain nominal whisker values */
                m_p_whisker_gripper_interpreter->obtainNominalValues();
                break;

            case GRASP:
                /* move forward until stem is in gripper */
                doGraspBehavior();
                break;

            case FOLLOW:
                /* move up along stem */
                doFollowBehavior();
                break;

            case END:
                /* stop moving, show gripper sensing interpretation */
                doEndBehaviour();
                break;

            default:
                ERROR_STREAM("Unknown state in stemtrack monitor!");
                return false;
            }

        }

        return true;
    }
}

StemTrackMonitor::~StemTrackMonitor()
{
    //destructor
}
