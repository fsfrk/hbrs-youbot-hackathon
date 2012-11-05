#include "arm_bridge_ros_orocos.hpp"


ArmBridgeRosOrocos::ArmBridgeRosOrocos(const string& name) :  TaskContext(name, PreOperational), m_youbot_arm_dof(5)
{
	m_tf_listener = new tf::TransformListener();
	m_check_cart_pose_reached = false;
	m_check_joint_cfg_reached = false;

	m_srv_enable_navigation_ctrl_mode = m_nh.advertiseService("/raw_arm_bridge_ros_orocos/enable_navigation_ctrl_mode", &ArmBridgeRosOrocos::enableNavigationCtrlModeCallback, this);

	m_joint_config_as = new actionlib::ActionServer<raw_arm_navigation::MoveToJointConfigurationAction > (
          m_nh, "/arm_1/arm_controller/MoveToJointConfigurationDirect", boost::bind(&ArmBridgeRosOrocos::armJointConfigurationGoalCallback, this, _1), false);

	m_cartesian_pose_with_impedance_ctrl_as = new actionlib::ActionServer<raw_arm_navigation::MoveToCartesianPoseAction > (
	      m_nh, "/arm_1/arm_controller/MoveToCartesianPoseDirect", boost::bind(&ArmBridgeRosOrocos::armCartesianPoseWithImpedanceCtrlGoalCallback, this, _1), false);

	m_gripper_as = new actionlib::ActionServer<raw_arm_navigation::MoveToJointConfigurationAction > (
	      m_nh, "/arm_1/gripper_controller/MoveToJointConfigurationDirect", boost::bind(&ArmBridgeRosOrocos::gripperCallback, this, _1), false);

	// Ports
	this->addPort("brics_joint_positions", ip_brics_joint_positions).doc("Input of joint positions in BRICS data types");
    this->addPort("current_force", ip_current_force).doc("Input of current forces as float array");
    this->addPort("joint_states", ip_joint_states).doc("Input of joint states as sensor_msgs::JointState");
}

ArmBridgeRosOrocos::~ArmBridgeRosOrocos()
{
	delete m_cartesian_pose_with_impedance_ctrl_as;
	delete m_joint_config_as;
    delete m_gripper_as;
}

bool ArmBridgeRosOrocos::configureHook()
{
	//Operations
	if(this->getPeer("executive"))
	{
		log(Info) << "get operation jointspaceControl" << endlog();
		m_joint_space_ctrl_op = this->getPeer("executive")->getOperation("jointspaceControl");

		log(Info) << "get operation useArmOnly" << endlog();
		m_use_arm_only_op = this->getPeer("executive")->getOperation("useArmOnly");

		log(Info) << "get operation cartesianControl" << endlog();
		m_cartesian_ctrl_op = this->getPeer("executive")->getOperation("cartesianControl");

		log(Info) << "get operation gravityCompensation" << endlog();
		m_gravity_compensation_ctrl_op = this->getPeer("executive")->getOperation("gravityCompensation");

		log(Info) << "get operation execute" << endlog();
		m_execute_op = this->getPeer("executive")->getOperation("execute");

		log(Info) << "get operation navigationControl" << endlog();
		m_navigation_ctrl_op = this->getPeer("executive")->getOperation("navigationControl");

		log(Info) << "get operation setArmJointAngles" << endlog();
		m_set_arm_joint_angles_op = this->getPeer("executive")->getOperation("setArmJointAngles");

        
		log(Info) << "get operation setCartesianDamping" << endlog();
		m_set_cartesian_damping_op = this->getPeer("executive")->getOperation("setCartesianDamping");

		log(Info) << "get operation setCartesianStiffness" << endlog();
		m_set_cartesian_stiffness_op = this->getPeer("executive")->getOperation("setCartesianStiffness");

		log(Info) << "get operation setHtipCC" << endlog();
		m_set_h_tip_cc_op = this->getPeer("executive")->getOperation("setHtipCC");

		log(Info) << "get operation setHvp0" << endlog();
		m_set_h_vp_0_op = this->getPeer("executive")->getOperation("setHvp0");


		log(Info) << "get operation openGripper" << endlog();
		m_open_gripper_op = this->getPeer("executive")->getOperation("openGripper");

		log(Info) << "get operation closeGripper" << endlog();
		m_close_gripper_op = this->getPeer("executive")->getOperation("closeGripper");

		if(!m_joint_space_ctrl_op.ready() || !m_use_arm_only_op.ready() || !m_execute_op.ready() || !m_gravity_compensation_ctrl_op.ready() || !m_execute_op.ready() || !m_set_arm_joint_angles_op.ready() || !m_set_cartesian_damping_op.ready() || !m_set_cartesian_stiffness_op.ready() || !m_set_h_tip_cc_op.ready() || !m_set_h_vp_0_op.ready() || !m_open_gripper_op.ready() || !m_close_gripper_op.ready())
		{
			log(Error) << "operation(s) not available" << endlog();
			return false;
		}
	}

	return TaskContext::configureHook();
}

bool ArmBridgeRosOrocos::startHook()
{
	m_cartesian_pose_with_impedance_ctrl_as->start();
	m_joint_config_as->start();
	m_gripper_as->start();

	ROS_INFO("arm actions started");

	if (!ip_brics_joint_positions.connected())
	{
		log(Error) << "BRICS joint positions not connected." << endlog();
		return false;
	}

	if (!ip_joint_states.connected())
	{
		log(Error) << "joint_states not connected." << endlog();
		return false;
	}

    if (!ip_current_force.connected())
	{
		log(Error) << "current_force is not connected." << endlog();
		return false;
	}

	return TaskContext::startHook();
}

void ArmBridgeRosOrocos::updateHook()
{
	TaskContext::updateHook();

	m_joint_states_received = false;
	ros::spinOnce();


	if(m_check_cart_pose_reached && isCartPoseReached())
	{
		m_check_cart_pose_reached = false;
		m_cart_pose_goal_handle.setSucceeded();
	}

	if(m_check_joint_cfg_reached)
	{
		if (ip_joint_states.read(m_current_joint_states) != NoData)
		{
			if(isJointCfgReached())
			{
				m_check_joint_cfg_reached = false;
				m_joint_cfg_goal_handle.setSucceeded();
			}
		}
	}


	// check if new joint position arrived at topic
	if (ip_brics_joint_positions.read(m_brics_joint_positions) != NoData)
	{
		ROS_INFO("received new joint configuration on topic");
		setJointPositions(m_brics_joint_positions);
	}
}

void ArmBridgeRosOrocos::stopHook()
{
	TaskContext::stopHook();
}

void ArmBridgeRosOrocos::cleanupHook()
{
	TaskContext::cleanupHook();
}

void ArmBridgeRosOrocos::setJointPositions(brics_actuator::JointPositions brics_joint_positions)
{
	std::vector<double> joint_position(brics_joint_positions.positions.size(), 0.0);

	for (size_t i = 0; i < brics_joint_positions.positions.size(); i++)
	{	
		joint_position[i] = brics_joint_positions.positions[i].value + joint_initial_state_offsets[i];
	}

    m_set_arm_joint_angles_op(joint_position);
}

void ArmBridgeRosOrocos::armJointConfigurationGoalCallback(actionlib::ActionServer<raw_arm_navigation::MoveToJointConfigurationAction>::GoalHandle joint_cfg_goal)
{
 	ROS_INFO("MoveToJointConfigurationDirect action called");

	joint_cfg_goal.setAccepted();

	m_joint_space_ctrl_op();
	m_use_arm_only_op();

    m_desired_joint_cfg = joint_cfg_goal.getGoal()->goal;

	setJointPositions(joint_cfg_goal.getGoal()->goal);

    // start to move to the joint position
	m_execute_op();

	m_joint_cfg_goal_handle = joint_cfg_goal;
	m_check_joint_cfg_reached = true;
}

void ArmBridgeRosOrocos::armCartesianPoseWithImpedanceCtrlGoalCallback(actionlib::ActionServer<raw_arm_navigation::MoveToCartesianPoseAction>::GoalHandle cartesian_pose_goal)
{
    std::vector<double> homog_matrix(16, 0.0);
    std::vector<double> cartesian_stiffness(9, 0.0);
    std::vector<double> cartesian_damping(6, 2.0);  
    std::vector<double> h_tip_cc(16, 0.0);


	ROS_INFO("MoveToCartesianPoseDirect action called");

	geometry_msgs::PoseStamped goal_pose = cartesian_pose_goal.getGoal()->goal;
    geometry_msgs::PoseStamped transformed_pose;
	
    try
    {
	    m_tf_listener->transformPose("/map", goal_pose, transformed_pose);
    }
    catch(const std::exception& ex)
    {
        log(Error) << "could not transform pose: " << transformed_pose << endlog();
        return;
    }
    
    goal_pose = transformed_pose;

	cartesian_pose_goal.setAccepted();

	m_cartesian_ctrl_op();

	double qw, qx, qy, qz;
	qx = goal_pose.pose.orientation.x;
	qy = goal_pose.pose.orientation.y;
	qz = goal_pose.pose.orientation.z;
	qw = goal_pose.pose.orientation.w;

	homog_matrix[0] = pow(qw,2) + pow(qx,2) - pow(qy,2) - pow(qz,2);
	homog_matrix[1] = 2*qx*qy - 2*qz*qw;
	homog_matrix[2] = 2*qx*qz + 2*qy*qw;
	homog_matrix[3] = goal_pose.pose.position.x;
	homog_matrix[4] = 2*qx*qy + 2*qz*qw;
	homog_matrix[5] = pow(qw,2) - pow(qx,2) + pow(qy,2) - pow(qz,2);
	homog_matrix[6] = 2*qy*qz - 2*qx*qw;
	homog_matrix[7] = goal_pose.pose.position.y;
	homog_matrix[8] = 2*qx*qz - 2*qy*qw;
	homog_matrix[9] = 2*qy*qz + 2*qx*qw;
	homog_matrix[10] = pow(qw,2) - pow(qx,2) - pow(qy,2) + pow(qz,2);
	homog_matrix[11] = goal_pose.pose.position.z;
	homog_matrix[12] = 0.0;
	homog_matrix[13] = 0.0;
	homog_matrix[14] = 0.0;
	homog_matrix[15] = 1.0;

	cartesian_stiffness[0] = 100.0;
	cartesian_stiffness[1] = 100.0;
	cartesian_stiffness[2] = 100.0;
	cartesian_stiffness[3] = 5.0;
	cartesian_stiffness[4] = 5.0;
	cartesian_stiffness[5] = 5.0;
	cartesian_stiffness[6] = 0.0;
	cartesian_stiffness[7] = 0.0;
	cartesian_stiffness[8] = 0.0;

	// if the stiffness is set in the message, overwrite the standard parameters set above
	for(unsigned int i=0; i < cartesian_pose_goal.getGoal()->cartesian_stiffness.size(); ++i)
		if(cartesian_pose_goal.getGoal()->cartesian_stiffness[i] > 0.0)
			cartesian_stiffness[i] = cartesian_pose_goal.getGoal()->cartesian_stiffness[i];

	// if the damping is set in the message, overwrite the standard parameters from the initialization
	for(unsigned int j=0; j < cartesian_pose_goal.getGoal()->cartesian_damping.size(); ++j)
		if(cartesian_pose_goal.getGoal()->cartesian_damping[j] > 0.0)
			cartesian_damping[j] = cartesian_pose_goal.getGoal()->cartesian_damping[j];

	// identity
	h_tip_cc[0] = h_tip_cc[5] = h_tip_cc[10] = h_tip_cc[15] = 1.0;

	m_set_cartesian_damping_op(cartesian_damping);
	m_set_cartesian_stiffness_op(cartesian_stiffness);
	m_set_h_tip_cc_op(h_tip_cc);
	m_set_h_vp_0_op(homog_matrix);

	m_execute_op();

	m_sum_force_diff = 1.0;
	m_cart_pose_goal_handle = cartesian_pose_goal;
	m_check_cart_pose_reached = true;
}

bool ArmBridgeRosOrocos::enableNavigationCtrlModeCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	m_navigation_ctrl_op();
	m_execute_op();

	return true;
}

void ArmBridgeRosOrocos::gripperCallback(actionlib::ActionServer<raw_arm_navigation::MoveToJointConfigurationAction>::GoalHandle gripper_goal)
{
	ROS_INFO("move gripper action called");

    brics_actuator::JointPositions joint_pos = gripper_goal.getGoal()->goal;

	gripper_goal.setAccepted();

    double opening_distance = 0;
    for (size_t i = 0; i < joint_pos.positions.size(); i++)
        opening_distance += joint_pos.positions[i].value;

    if(opening_distance < 0.0115)
        m_close_gripper_op();
    
    else
        m_open_gripper_op();

    sleep(3);

	gripper_goal.setSucceeded();
}

bool ArmBridgeRosOrocos::isJointCfgReached()
{
    //log(Info) << "desired size: " <<  m_desired_joint_cfg.positions.size() << " current size: " << m_current_joint_states.position.size() << endlog();

    for(unsigned int i=0; i < m_desired_joint_cfg.positions.size(); ++i)
    {
        for(unsigned int j=0; j < m_current_joint_states.position.size(); ++j)
        {
	    //log(Info) << "desired joint name: " << m_desired_joint_cfg.positions[i].joint_uri << " current joint name: " <<  m_desired_joint_cfg.positions[i].joint_uri << endlog();
            if(m_desired_joint_cfg.positions[i].joint_uri == m_current_joint_states.name[j])
            {
            	//log(Info) << "diff of joint <<" << m_desired_joint_cfg.positions[i].joint_uri << " : " << fabs(m_desired_joint_cfg.positions[i].value - m_current_joint_states.position[j]) << endlog();
            	if(fabs(m_desired_joint_cfg.positions[i].value - m_current_joint_states.position[j]) > 0.1)
            	{
            		return false;
            	}
            }
        }
    }

    return true;
}



bool ArmBridgeRosOrocos::isCartPoseReached()
{
	if(ip_current_force.read(m_current_force) == NoData)
		return false;

    double norm_force_diff = 0;
    for(unsigned int i=0; i < m_current_force.data.size(); ++i)
        for(unsigned int j=0; j < m_previous_force.data.size(); ++j)
            norm_force_diff += fabs(m_current_force.data[i] - m_previous_force.data[i]);

    m_sum_force_diff = (norm_force_diff * 0.1) + (0.9 * m_sum_force_diff);

    m_previous_force = m_current_force;

    if(m_sum_force_diff < 0.01)
        return true;

    return false;
}

ORO_CREATE_COMPONENT( ArmBridgeRosOrocos )
