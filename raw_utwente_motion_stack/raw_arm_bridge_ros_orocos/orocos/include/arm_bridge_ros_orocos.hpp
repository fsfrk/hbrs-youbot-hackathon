#pragma once

/* OROCOS include files */
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Port.hpp>
#include <rtt/Activity.hpp>
#include <rtt/RTT.hpp>
#include <rtt/Property.hpp>
#include <rtt/PropertyBag.hpp>
#include <rtt/Time.hpp>
#include <ocl/Component.hpp>

#include <string>
#include <vector>

#include <geometry_msgs/typekit/Types.h>
#include <std_msgs/Float64MultiArray.h>
#include <brics_actuator/typekit/Types.h>
#include <actionlib/server/simple_action_server.h>
#include <raw_arm_navigation/MoveToCartesianPoseAction.h>
#include <raw_arm_navigation/MoveToJointConfigurationAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <tf/transform_datatypes.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>

#include <boost/ptr_container/ptr_vector.hpp>

using namespace RTT;
using namespace std;

class ArmBridgeRosOrocos: public TaskContext
{
  public:
    ArmBridgeRosOrocos(const string& name);
    virtual ~ArmBridgeRosOrocos();

  protected:
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
    virtual void stopHook();
    virtual void cleanupHook();


	//void armJointTrajectoryCancelCallback(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle youbot_arm_goal);

    /**
     * @brief Callback that is executed when an action goal to perform a arm movement in joint space comes in.
     * @param joint_cfg_goal Actionlib goal that contains the joint configuration.
     */
    void armJointConfigurationGoalCallback(actionlib::ActionServer<raw_arm_navigation::MoveToJointConfigurationAction>::GoalHandle joint_cfg_goal);


    /**
	  * @brief Callback that is executed when an action goal to go to a Cartesian pose with the arm comes in.
	  * @param cartesian_pose_goal Actionlib goal that contains the 6DOF pose.
	  */
    void armCartesianPoseWithImpedanceCtrlGoalCallback(actionlib::ActionServer<raw_arm_navigation::MoveToCartesianPoseAction>::GoalHandle cartesian_pose_goal);

    void gripperCallback(actionlib::ActionServer<raw_arm_navigation::MoveToJointConfigurationAction>::GoalHandle joint_cfg_goal);

	bool enableNavigationCtrlModeCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    void setJointPositions(brics_actuator::JointPositions brics_joint_positions);

  private:
    bool isJointCfgReached();
    bool isCartPoseReached();

    InputPort<brics_actuator::JointPositions> ip_brics_joint_positions;
    brics_actuator::JointPositions m_brics_joint_positions;

    InputPort<sensor_msgs::JointState> ip_joint_states;
    sensor_msgs::JointState m_current_joint_states;

    InputPort<std_msgs::Float64MultiArray> ip_current_force;
    std_msgs::Float64MultiArray m_current_force;
    std_msgs::Float64MultiArray m_previous_force;
    double m_sum_force_diff;

    OperationCaller<void(void)> m_joint_space_ctrl_op;
    OperationCaller<void(void)> m_use_arm_only_op;
    OperationCaller<void(void)> m_cartesian_ctrl_op;
    OperationCaller<void(void)> m_gravity_compensation_ctrl_op;
    OperationCaller<void(void)> m_execute_op;
    OperationCaller<void(void)> m_navigation_ctrl_op;
    OperationCaller<void(void)> m_open_gripper_op;
    OperationCaller<void(void)> m_close_gripper_op;
    
    OperationCaller<void(std::vector<double>)> m_set_arm_joint_angles_op;
    OperationCaller<void(std::vector<double>)> m_set_cartesian_damping_op;
    OperationCaller<void(std::vector<double>)> m_set_cartesian_stiffness_op;
    OperationCaller<void(std::vector<double>)> m_set_h_tip_cc_op;
    OperationCaller<void(std::vector<double>)> m_set_h_vp_0_op;

	ros::NodeHandle m_nh;
	tf::TransformListener* m_tf_listener;

    actionlib::ActionServer<raw_arm_navigation::MoveToJointConfigurationAction> *m_joint_config_as;
    actionlib::ActionServer<raw_arm_navigation::MoveToCartesianPoseAction> *m_cartesian_pose_with_impedance_ctrl_as;
    actionlib::ActionServer<raw_arm_navigation::MoveToJointConfigurationAction> *m_gripper_as;

    actionlib::ActionServer<raw_arm_navigation::MoveToCartesianPoseAction>::GoalHandle m_cart_pose_goal_handle;
    bool m_check_cart_pose_reached;

    actionlib::ActionServer<raw_arm_navigation::MoveToJointConfigurationAction>::GoalHandle m_joint_cfg_goal_handle;
    bool m_check_joint_cfg_reached;
    sensor_msgs::JointState m_current_joint_cfg;
    brics_actuator::JointPositions m_desired_joint_cfg;
    bool m_joint_states_received;

	ros::ServiceServer m_srv_enable_navigation_ctrl_mode;

    const size_t m_youbot_arm_dof;
};

static const double joint_initial_state_offsets[] = {-2.9496, -1.13, 2.5482, -1.789, -2.9234}; 
