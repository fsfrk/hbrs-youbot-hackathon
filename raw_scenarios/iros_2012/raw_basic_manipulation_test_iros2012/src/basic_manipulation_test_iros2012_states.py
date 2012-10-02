#!/usr/bin/python
import roslib; roslib.load_manifest('raw_basic_manipulation_test_iros2012')
import rospy

import smach
import smach_ros

from simple_script_server import *
sss = simple_script_server()

class select_base_pose(smach.State):
    def __init__(self, pose):
        smach.State.__init__(self, 
            outcomes=['succeeded'],
            input_keys=['task_spec', 'base_pose_to_approach'],
            output_keys=['base_pose_to_approach'])
        
        self.pose = pose

    def execute(self, userdata):
        if(self.pose == 'source_pose'):
            userdata.base_pose_to_approach = userdata.task_spec.source_pose
        elif(self.pose == 'destination_pose'):
            userdata.base_pose_to_approach = userdata.task_spec.destination_pose
        elif(self.pose == 'final_pose'):
            userdata.base_pose_to_approach = userdata.task_spec.final_pose
        
        rospy.loginfo("selected pose: %s", userdata.base_pose_to_approach)
                
        return 'succeeded'


class select_recognized_object(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded','no_more_objects'],
            input_keys=['recognized_objects'],
            output_keys=['recognized_objects', 'object_to_grasp'])
        
    def execute(self, userdata):
        
        if(len(userdata.recognized_objects) == 0):
            return 'no_more_objects'
        
        userdata.object_to_grasp = userdata.recognized_objects.pop() 
                
        return 'succeeded'


class get_obj_poses_for_goal_configuration(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded', 'configuration_poses_not_available'],
            input_keys=['task_spec','obj_goal_configuration_poses'],
            output_keys=['obj_goal_configuration_poses'])
        
    def execute(self, userdata):
        
        sss.move("gripper","open")
        print userdata.task_spec.object_config 
        
        if (not rospy.has_param("/script_server/arm/" + userdata.task_spec.object_config)):
            rospy.logerr("configuration <<" + userdata.task_spec.object_config + ">> NOT available on parameter server")
            return 'configuration_poses_not_available'
            
        pose_names = rospy.get_param("/script_server/arm/" + userdata.task_spec.object_config)

        print pose_names
        
        for pose_name in pose_names:
            print "cfg pose: ", pose_name
            userdata.obj_goal_configuration_poses.append((userdata.task_spec.object_config + "/" + pose_name))
    

        userdata.obj_goal_configuration_poses.sort()
        
                
        return 'succeeded'

class demo_grasp_random_object_vertical(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['object_list'])
        
    def execute(self, userdata):
        sss.move("gripper", "open", blocking=False)
       # sss.move("arm", "zeroposition")
        
        for object in userdata.object_list:         
            
            # ToDo: need to be adjusted to correct stuff           
            if object.pose.pose.position.z <= 0.0 or object.pose.pose.position.z >= 0.10:
                continue
    
            sss.move("arm", "zeroposition", mode="planned")                             

            #object.pose.pose.position.z = object.pose.pose.position.z + 0.02
            object.pose.pose.position.x = object.pose.pose.position.x + 0.01
            object.pose.pose.position.y = object.pose.pose.position.y - 0.005

            handle_arm = sss.move("arm", [object.pose.pose.position.x, object.pose.pose.position.y, object.pose.pose.position.z, "/base_link"], mode="planned")

            if handle_arm.get_state() == 3:
                sss.move("gripper", "close", blocking=False)
                rospy.sleep(3.0)
                sss.move("arm", "zeroposition", mode="planned")        
                return 'succeeded'    
            else:
                rospy.logerr('could not find IK for current object')

        return 'failed'
class demo_grasp_vertical_object(smach.StateMachine):
    def __init__(self):    
        smach.StateMachine.__init__(self, 
            outcomes=['object_grasped', 'failed'])
        
        with self:
            smach.StateMachine.add('MOVE_ARM_OUT_OF_VIEW', move_arm_out_of_view(),
                transitions={'succeeded':'FIND_OBJECT'})
                      
            smach.StateMachine.add('FIND_OBJECT', detect_object(),
                transitions={'succeeded':'GRASP_OBJECT',  
                             'failed':'failed'})
                
            smach.StateMachine.add('GRASP_OBJECT', demo_grasp_random_object_vertical(),
                transitions={'succeeded':'object_grasped', 
                            'failed':'failed'})




