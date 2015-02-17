#ifndef ROBOTSTATUS_H
#define ROBOTSTATUS_H

#include <vector>
#include <iostream>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>

class RobotRepresentation;

class RobotStatus
{
    private:
        std::vector<long int> m_last_joint_update_times; //microseconds since epoch
        long int m_last_whiskers_update;
        long int m_last_pressure_sensors_update;
        std::vector<bool> m_wait_for_joint_update;
        bool m_wait_for_whiskers_update;
        bool m_wait_for_pressure_sensors_update;
        KDL::JntArray m_joints_to_monitor; // for amigo: torso / shoulder-jaw / shoulder-pitch / shoulder-roll / elbow-pitch / elbow-roll / wrist-pitch / wrist-yaw
        int m_n_joints_monitoring;
        std::vector<float> m_gripper_xyz;
        double m_joints_up_to_date_threshold, m_whiskers_up_to_date_threshold, m_pressure_sensors_up_to_date_threshold;
        KDL::Frame m_gripper_kdlframe;
        float m_pos_reached_threshold;
        RobotRepresentation* m_p_robot_representation;
        double m_xyz_reached_threshold;
        std::vector<float> m_whisker_measurements, m_pressure_sensor_measurements;
        uint m_n_whiskers, m_n_pressure_sensors;

    public:
        RobotStatus(RobotRepresentation* p_robot_representation);

        inline double setXYZreachedThreshold(double xyz_reached_threshold) { m_xyz_reached_threshold = xyz_reached_threshold; }
        inline void setJointsUpToDateThreshold(double threshold) {     m_joints_up_to_date_threshold = threshold; }
        inline void setWhiskersUpToDateThreshold(double threshold) {     m_whiskers_up_to_date_threshold = threshold; }
        inline void setPressureSensorsUpToDateThreshold(double threshold) {     m_pressure_sensors_up_to_date_threshold = threshold; }
        inline void setPosReachedThreshold(float pos_reached_threshold) {  m_pos_reached_threshold = pos_reached_threshold; }
        inline void setNumberOfWhiskers(uint n_whiskers) { m_n_whiskers = n_whiskers; m_whisker_measurements.assign(m_n_whiskers,0.0); }
        inline void setNumberOfPressureSensors(uint n_pressure_sensors) { m_n_pressure_sensors = n_pressure_sensors; m_pressure_sensor_measurements.assign(m_n_pressure_sensors,0.0); }

        bool reachedPosition(KDL::JntArray reference);
        bool reachedPosition(std::vector<float> reference);
        void updateJointStatus(KDL::JntArray updated_joint_status, std::vector<int> joints_updated);
        void updateWhiskerMeasurements(std::vector<float> updated_whisker_measurements);
        void updatePressureSensorMeasurements(std::vector<float> updated_pressure_sensor_measurements);
        bool waitingForFirstJointStatusUpdate();
        bool isGripperXYZvalid();
        bool hasValidGripperXYZ();
        void updateGripperXYZ();
        void resetUpToDateStatus();
        const bool isUpToDate();
        const bool jointStatusIsUpToDate();
        const bool whiskerMeasurementsAreUpToDate();
        const bool pressureSensorMeasurementsAreUpToDate();

        inline const KDL::JntArray& getJointStatus() const { return m_joints_to_monitor; }
        inline const int& getNjointsMonitoring() const { return m_n_joints_monitoring; }
        const std::vector<float>& getGripperXYZ();
        const KDL::Frame& getGripperKDLframe();
        const long int getWorstCaseTimeSinceLastJointUpdate() const;
        const long int getTimeSinceLastWhiskersUpdate() const;
        const long int getTimeSinceLastPressureSensorsUpdate() const;
        inline const std::vector<float>& getWhiskerMeasurements() const { return m_whisker_measurements; }
        inline const std::vector<float>& getPressureSensorMeasurements() const { return m_pressure_sensor_measurements; }

        ~RobotStatus();
};

#endif // ROBOTSTATUS_H
