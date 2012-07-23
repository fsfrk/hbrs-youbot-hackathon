#!/usr/bin/env python
import roslib; roslib.load_manifest('raw_arm_navigation')

import rospy
import sensor_msgs.msg
import actionlib
import brics_actuator.msg
import raw_arm_navigation.msg
import arm_navigation_msgs.msg
import tf
import control_msgs.msg
import arm_navigation_msgs.msg
import math
import random

from simple_ik_solver_wrapper import SimpleIkSolver


class ArmActionServer:
    def __init__(self):
        self.received_state = False
        if (not rospy.has_param("joints")):
            rospy.logerr("No arm joints given.")
            exit(0)
        else:
            self.joint_names = sorted(rospy.get_param("joints"))
            rospy.loginfo("arm joints: %s", self.joint_names)
        # read joint limits
        self.joint_limits = []
        for joint in self.joint_names:
            if ((not rospy.has_param("limits/" + joint + "/min")) or (not rospy.has_param("limits/" + joint + "/min"))):
                rospy.logerr("No arm joint limits given.")
                exit(0)
            else:
                limit = arm_navigation_msgs.msg.JointLimits()
                limit.joint_name = joint 
                limit.min_position = rospy.get_param("limits/" + joint + "/min")
                limit.max_position = rospy.get_param("limits/" + joint + "/max")
                self.joint_limits.append(limit)
        self.current_joint_configuration = [0 for i in range(len(self.joint_names))]
        self.unit = "rad"
        
        # subscriptions
        rospy.Subscriber("joint_states", sensor_msgs.msg.JointState, self.joint_states_callback)
        # publications
        self.pub_joint_positions = rospy.Publisher("position_command", brics_actuator.msg.JointPositions)
        
        # action server
        self.as_move_joint_direct = actionlib.SimpleActionServer("MoveToJointConfigurationDirect", raw_arm_navigation.msg.MoveToJointConfigurationAction, execute_cb = self.execute_cb_move_joint_config_direct)
        self.as_move_cart_direct = actionlib.SimpleActionServer("MoveToCartesianPoseDirect", raw_arm_navigation.msg.MoveToCartesianPoseAction, execute_cb = self.execute_cb_move_cartesian_direct)
        self.as_move_cart_rpy_sampled_direct = actionlib.SimpleActionServer("MoveToCartesianRPYSampledDirect", raw_arm_navigation.msg.MoveToCartesianPoseAction, execute_cb = self.execute_cb_move_cartesian_rpy_sampled_direct)
        self.as_move_joint_trajectory_direct = actionlib.SimpleActionServer("MoveJointTrajectoryDirect", control_msgs.msg.FollowJointTrajectoryAction, execute_cb = self.execute_cb_move_joint_trajectory_direct)
        
        self.as_move_joint_planned = actionlib.SimpleActionServer("MoveToJointConfigurationDirectPlanned", raw_arm_navigation.msg.MoveToJointConfigurationAction, execute_cb = self.execute_cb_move_joint_config_planned)
        self.as_move_cart_planned = actionlib.SimpleActionServer("MoveToCartesianPoseDirectPlanned", raw_arm_navigation.msg.MoveToCartesianPoseAction, execute_cb = self.execute_cb_move_cartesian_planned)
        self.as_move_cart_rpy_sampled_planned = actionlib.SimpleActionServer("MoveToCartesianPoseDirectPlanned", raw_arm_navigation.msg.MoveToCartesianPoseAction, execute_cb = self.execute_cb_move_cartesian_planned)
        self.as_move_joint_trajectory_palnned = actionlib.SimpleActionServer("MoveJointTrajectoryPlanned", control_msgs.msg.FollowJointTrajectoryAction, execute_cb = self.execute_cb_move_joint_trajectory_planned)
    
        # additional classes
        self.iks = SimpleIkSolver()
        
    def joint_states_callback(self, msg):
        for k in range(len(self.joint_names)):
            for i in range(len(msg.name)):
                if (msg.name[i] == self.joint_names[k]):
                    #rospy.loginfo("%s: %f", msg.name[i], msg.position[i])
                    self.current_joint_configuration[k] = msg.position[i]
        #print 'joint states received'
        self.received_state = True
        
    def is_joint_configuration_not_in_limits(self, goal_configuration):
        for goal_joint in goal_configuration.positions:
            for joint_limit in self.joint_limits:
                if ((goal_joint.joint_uri == joint_limit.joint_name) and ((goal_joint.value < joint_limit.min_position) or (goal_joint.value > joint_limit.max_position))):
                    rospy.logerr("goal configuration has <<%s>> in joint limit: %lf", goal_joint.joint_uri, goal_joint.value)
                    return False
        return True
    
    def is_goal_reached(self, goal_pose):
        for i in range(len(self.joint_names)):
            #rospy.loginfo("joint: %d -> curr_val: %f --- goal_val: %f", i, goal_pose.positions[i].value, self.current_joint_configuration[i])
            if (abs(goal_pose.positions[i].value - self.current_joint_configuration[i]) > 0.05):
                #ToDo: threshold via parameter
                return False
        rospy.loginfo("arm goal pose reached")
        return True
       
       
    #################################################
    ##### WITHOUT PLANNING
    #################################################
            
    def execute_cb_move_joint_config_direct(self, action_msgs):
        rospy.loginfo("move arm to joint configuration DIRECT")
        
        if not self.is_joint_configuration_not_in_limits(action_msgs.goal):
            result = raw_arm_navigation.msg.MoveToJointConfigurationResult()
            result.result.val = arm_navigation_msgs.msg.ArmNavigationErrorCodes.JOINT_LIMITS_VIOLATED
            self.as_move_joint_direct.set_aborted(result)
            return

        self.pub_joint_positions.publish(action_msgs.goal)
        #wait to reach the goal position

        while (not rospy.is_shutdown()):
            if (self.is_goal_reached(action_msgs.goal)):
                break

        result = raw_arm_navigation.msg.MoveToJointConfigurationResult()
        result.result.val = arm_navigation_msgs.msg.ArmNavigationErrorCodes.SUCCESS
        self.as_move_joint_direct.set_succeeded(result)
        

    def execute_cb_move_cartesian_direct(self, action_msgs):
        rospy.loginfo("move arm to cartesian pose DIRECT")
        
        joint_config = self.iks.call_constraint_aware_ik_solver(action_msgs.goal)
        
        result = raw_arm_navigation.msg.MoveToCartesianPoseResult()
                
        if (joint_config):
            rospy.loginfo("IK solution found")
            jp = brics_actuator.msg.JointPositions()
            
            for i in range(5):
                jv = brics_actuator.msg.JointValue()
                jv.joint_uri = self.iks.joint_names[i]
                jv.value = joint_config[i]
                jv.unit = self.unit
                jp.positions.append(jv)
                
        
            if not self.is_joint_configuration_not_in_limits(jp):
                result.result.val = arm_navigation_msgs.msg.ArmNavigationErrorCodes.JOINT_LIMITS_VIOLATED
                self.as_move_cart_direct.set_aborted(result)
                return
        
            
            self.pub_joint_positions.publish(jp)
            
            #wait to reach the goal position
            while (not rospy.is_shutdown()):
                if (self.is_goal_reached(jp)):
                    break
            
            result.result.val = arm_navigation_msgs.msg.ArmNavigationErrorCodes.SUCCESS
            self.as_move_cart_direct.set_succeeded(result)
        
        else:
            rospy.logerr("NO IK solution found")
            result.result.val = arm_navigation_msgs.msg.ArmNavigationErrorCodes.NO_IK_SOLUTION
            self.as_move_cart_direct.set_aborted(result)

            
    def execute_cb_move_cartesian_rpy_sampled_direct(self, action_msgs):
        rospy.loginfo("move to cartesian pose sampling for valid rpy DIRECT")
        
        pose = raw_arm_navigation.msg.MoveToCartesianPoseGoal()
        pose.goal.header.frame_id = action_msgs.goal.header.frame_id
        pose.goal.pose.position.x = action_msgs.goal.pose.position.x
        pose.goal.pose.position.y = action_msgs.goal.pose.position.y
        pose.goal.pose.position.z = action_msgs.goal.pose.position.z
        
        for i in range(100):
            rand_value = random.randint(int(-math.pi/0.01), int(math.pi/0.01))/100.0

            (qx, qy, qz, qw) = tf.transformations.quaternion_from_euler(0, rand_value, 0)
            pose.goal.header.stamp = rospy.Time.now()
            pose.goal.pose.orientation.x = qx
            pose.goal.pose.orientation.y = qy
            pose.goal.pose.orientation.z = qz
            pose.goal.pose.orientation.w = qw
                    
            joint_config = self.iks.call_constraint_aware_ik_solver(pose.goal)
                                
            if (joint_config):
                rospy.loginfo("IK solution found")
                break

        result = raw_arm_navigation.msg.MoveToJointConfigurationResult()
        
        if(joint_config):
            jp = brics_actuator.msg.JointPositions()
            for i in range(5):
                jv = brics_actuator.msg.JointValue()
                jv.joint_uri = self.iks.joint_names[i]
                jv.value = joint_config[i]
                jv.unit = self.unit
                jp.positions.append(jv)
            if not self.is_joint_configuration_not_in_limits(jp):
                result.result.val = arm_navigation_msgs.msg.ArmNavigationErrorCodes.JOINT_LIMITS_VIOLATED
                self.as_move_cart_rpy_sampled_direct.set_aborted(result)
                return

            self.pub_joint_positions.publish(jp)
            
            #wait to reach the goal position
            while (not rospy.is_shutdown()):
                if (self.is_goal_reached(jp)):
                    break

            result.result.val = arm_navigation_msgs.msg.ArmNavigationErrorCodes.SUCCESS
            self.as_move_cart_rpy_sampled_direct.set_succeeded(result)

        else:
            rospy.logerr("NO IK solution found")
            result.result.val = arm_navigation_msgs.msg.ArmNavigationErrorCodes.NO_IK_SOLUTION
            self.as_move_cart_rpy_sampled_direct.set_aborted(result)
            
    
    def execute_cb_move_joint_trajectory_direct(self, goal):
        rospy.loginfo("move joint trajectory DIRECT")
        
        is_timed_out = False
        start = rospy.Time.now()
        duration = rospy.Duration(5.0)
        
        for i in range(len(goal.trajectory.points)):
            joint_positions = brics_actuator.msg.JointPositions()
            conf = goal.trajectory.points[i].positions
            
        # transform from ROS to BRICS message
        for i in range(len(self.joint_names)):
            joint_value = brics_actuator.msg.JointValue()
            joint_value.joint_uri = self.joint_names[i]
            joint_value.value = conf[i]
            joint_value.unit = self.unit
            joint_positions.positions.append(joint_value)
        self.pub_joint_positions.publish(joint_positions)
        
        # wait to reach the goal position
        while (not rospy.is_shutdown()):
            if (self.is_goal_reached(conf)):
                break
            if (rospy.Time.now() - start > duration):
                is_timed_out = True
                break
            if (is_timed_out):
                break
        
        result = control_msgs.msg.FollowJointTrajectoryResult()
        if (is_timed_out):
            result.error_code = control_msgs.msg.FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED
            self.as_move_joint_trajectory_direct.set_aborted(result)
        else:
            result.error_code = control_msgs.msg.FollowJointTrajectoryResult.SUCCESSFUL
            self.as_move_joint_trajectory_direct.set_succeeded(result)
            
            
            
    #################################################
    ##### WITH PLANNING
    #################################################        
            
    def execute_cb_move_joint_config_planned(self, action_msgs):
        
        rospy.loginfo("move arm to joint configuration PLANNED")
        '''
        if not self.is_joint_configuration_not_in_limits(action_msgs.goal):
            result = raw_arm_navigation.msg.MoveToJointConfigurationResult()
            result.result.val = arm_navigation_msgs.msg.ArmNavigationErrorCodes.JOINT_LIMITS_VIOLATED
            self.as_move_joint_direct.set_aborted(result)
            return

        self.pub_joint_positions.publish(action_msgs.goal)
        #wait to reach the goal position

        while (not rospy.is_shutdown()):
            if (self.is_goal_reached(action_msgs.goal)):
                break

        result = raw_arm_navigation.msg.MoveToJointConfigurationResult()
        result.result.val = arm_navigation_msgs.msg.ArmNavigationErrorCodes.SUCCESS
        self.as_move_joint_direct.set_succeeded(result)
        '''
        

    def execute_cb_move_cartesian_planned(self, action_msgs):
        
        rospy.loginfo("move arm to cartesian pose PLANNED")
        '''
        joint_config = self.iks.call_constraint_aware_ik_solver(action_msgs.goal)
        
        result = raw_arm_navigation.msg.MoveToCartesianPoseResult()
                
        if (joint_config):
            rospy.loginfo("IK solution found")
            jp = brics_actuator.msg.JointPositions()
            
            for i in range(5):
                jv = brics_actuator.msg.JointValue()
                jv.joint_uri = self.iks.joint_names[i]
                jv.value = joint_config[i]
                jv.unit = self.unit
                jp.positions.append(jv)
                
        
            if not self.is_joint_configuration_not_in_limits(jp):
                result.result.val = arm_navigation_msgs.msg.ArmNavigationErrorCodes.JOINT_LIMITS_VIOLATED
                self.as_move_cart_direct.set_aborted(result)
                return
        
            
            self.pub_joint_positions.publish(jp)
            
            #wait to reach the goal position
            while (not rospy.is_shutdown()):
                if (self.is_goal_reached(jp)):
                    break
            
            result.result.val = arm_navigation_msgs.msg.ArmNavigationErrorCodes.SUCCESS
            self.as_move_cart_direct.set_succeeded(result)
        
        else:
            rospy.logerr("NO IK solution found")
            result.result.val = arm_navigation_msgs.msg.ArmNavigationErrorCodes.NO_IK_SOLUTION
            self.as_move_cart_direct.set_aborted(result)
        '''

            
    def execute_cb_move_cartesian_rpy_sampled_planned(self, action_msgs):
        
        rospy.loginfo("move to cartesian pose sampling for valid rpy PLANNED")
        '''
        pose = raw_arm_navigation.msg.MoveToCartesianPoseGoal()
        pose.goal.header.frame_id = action_msgs.goal.header.frame_id
        pose.goal.pose.position.x = action_msgs.goal.pose.position.x
        pose.goal.pose.position.y = action_msgs.goal.pose.position.y
        pose.goal.pose.position.z = action_msgs.goal.pose.position.z
        
        for i in range(100):
            rand_value = random.randint(int(-math.pi/0.01), int(math.pi/0.01))/100.0

            (qx, qy, qz, qw) = tf.transformations.quaternion_from_euler(0, rand_value, 0)
            pose.goal.header.stamp = rospy.Time.now()
            pose.goal.pose.orientation.x = qx
            pose.goal.pose.orientation.y = qy
            pose.goal.pose.orientation.z = qz
            pose.goal.pose.orientation.w = qw
                    
            joint_config = self.iks.call_constraint_aware_ik_solver(pose.goal)
                                
            if (joint_config):
                rospy.loginfo("IK solution found")
                break

        result = raw_arm_navigation.msg.MoveToJointConfigurationResult()
        
        if(joint_config):
            jp = brics_actuator.msg.JointPositions()
            for i in range(5):
                jv = brics_actuator.msg.JointValue()
                jv.joint_uri = self.iks.joint_names[i]
                jv.value = joint_config[i]
                jv.unit = self.unit
                jp.positions.append(jv)
            if not self.is_joint_configuration_not_in_limits(jp):
                result.result.val = arm_navigation_msgs.msg.ArmNavigationErrorCodes.JOINT_LIMITS_VIOLATED
                self.as_move_cart_rpy_sampled_direct.set_aborted(result)
                return

            self.pub_joint_positions.publish(jp)
            
            #wait to reach the goal position
            while (not rospy.is_shutdown()):
                if (self.is_goal_reached(jp)):
                    break

            result.result.val = arm_navigation_msgs.msg.ArmNavigationErrorCodes.SUCCESS
            self.as_move_cart_rpy_sampled_direct.set_succeeded(result)

        else:
            rospy.logerr("NO IK solution found")
            result.result.val = arm_navigation_msgs.msg.ArmNavigationErrorCodes.NO_IK_SOLUTION
            self.as_move_cart_rpy_sampled_direct.set_aborted(result)
        '''
            
    
    def execute_cb_move_joint_trajectory_planned(self, goal):
        
        rospy.loginfo("move joint trajectory PLANNED")
        '''
        is_timed_out = False
        start = rospy.Time.now()
        duration = rospy.Duration(5.0)
        
        for i in range(len(goal.trajectory.points)):
            joint_positions = brics_actuator.msg.JointPositions()
            conf = goal.trajectory.points[i].positions
            
        # transform from ROS to BRICS message
        for i in range(len(self.joint_names)):
            joint_value = brics_actuator.msg.JointValue()
            joint_value.joint_uri = self.joint_names[i]
            joint_value.value = conf[i]
            joint_value.unit = self.unit
            joint_positions.positions.append(joint_value)
        self.pub_joint_positions.publish(joint_positions)
        
        # wait to reach the goal position
        while (not rospy.is_shutdown()):
            if (self.is_goal_reached(conf)):
                break
            if (rospy.Time.now() - start > duration):
                is_timed_out = True
                break
            if (is_timed_out):
                break
        
        result = control_msgs.msg.FollowJointTrajectoryResult()
        if (is_timed_out):
            result.error_code = control_msgs.msg.FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED
            self.as_move_joint_trajectory_direct.set_aborted(result)
        else:
            result.error_code = control_msgs.msg.FollowJointTrajectoryResult.SUCCESSFUL
            self.as_move_joint_trajectory_direct.set_succeeded(result)
        '''
                

if __name__ == "__main__":
    rospy.init_node("arm_action_server")
    
    action = ArmActionServer()
    
    rospy.loginfo("arm action server started")
    
    rospy.spin()
