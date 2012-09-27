#!/usr/bin/python
import roslib; roslib.load_manifest('raw_basic_transportation_test_iros2012')
import rospy

import smach
import smach_ros

from simple_script_server import *
sss = simple_script_server()

class Bunch:
    def __init__(self, **kwds):
         self.__dict__.update(kwds)

class select_object_to_be_grasped(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['obj_selected', 'no_obj_selected'],
            input_keys=['recognized_objects', 'objects_to_be_grasped'],
            output_keys=['object_to_be_grasped'])
        
    def execute(self, userdata):
        
        for rec_obj in userdata.recognized_objects:
            for obj_grasp in userdata.objects_to_be_grasped:
                if rec_obj.name == obj_grasp:
                    userdata.object_to_be_grasped = rec_obj
                    return 'obj_selected'
                
        return 'no_obj_selected'
    
    
class delete_grasped_obj_from_task_list(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded'],
            input_keys=['object_to_be_grasped', 'base_pose_to_approach', 'task_list'],
            output_keys=['task_list'])
        
    def execute(self, userdata):
        
        userdata.objects_to_be_grasped.remove(userdata.object_to_be_grasped)
        
        for i in range(len(userdata.task_list)):
          if userdata.task_list[i].type == "source" and userdata.task_list[i].location == userdata.base_pose_to_approach:
            rospy.loginfo("task.location: %s  base_pose_to_approach: %s", userdata.task_list[i].location, userdata.base_pose_to_approach)
            userdata.task_list[i].object_names.remove(userdata.object_to_be_grasped)
            return 'success'    


class select_btt_subtask(smach.State):
    def __init__(self, type=""):
        smach.State.__init__(self, 
            outcomes=['task_selected','no_more_task_for_given_type'],
            input_keys=['task_list'],
            output_keys=['base_pose_to_approach', 'objects_to_be_grasped'])
        
        self.type = type
        
    def execute(self, userdata):
        
        for task in task_list:
            if task.type == self.type: 
                userdata.base_pose_to_approach = task.location
                userdata.objects_to_be_grasped = task.object_names
                return 'task_selected'
            
        return 'no_more_task_for_given_type'
                     

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


class select_delivery_workstation(smach.State):
    def __init__(self, type=""):
        smach.State.__init__(self, 
            outcomes=['success', 'no_more_dest_tasks'],
            input_keys=['rear_platform_occupied_poses', 'task_list'],
            output_keys=['base_pose_to_approach', 'objects_to_be_grasped'])
        
        
    def execute(self, userdata):
        
        for obj in userdata.rear_platform_occupied_poses:
            for task in userdata.task_list:
                if task.type == 'destination':
                    for dest in task.object_names:
                        if dest == obj.obj_name:
                            userdata.base_pose_to_approach = task.location
                            userdata.objects_goal_configuration = task.object_config
                            return 'success'
                        
        return 'no_more_dest_tasks'
                        

class setup_btt(smach.State):
    def __init__(self, type=""):
        smach.State.__init__(self, 
            outcomes=['success'],
            input_keys=['task_list'],
            output_keys=['destinaton_free_poses'])
        
        
    def execute(self, userdata):
        
        for task in userdata.task_list:
            if task.type == 'destination':
                poses = ['1', '2', '3', '4', '5']
                loc_free_poses = Bunch(location = task.location, free_poses = poses)
                userdata.destinaton_free_poses.append(loc_free_poses)
                
        return 'success'
        
        
class grasp_obj_from_pltf_btt(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['object_grasped', 'no_more_obj_for_this_workspace'], 
                             input_keys=['rear_platform_occupied_poses', 'base_pose_to_approach'],
                             output_keys=['rear_platform_occupied_poses', 'last_grasped_obj_name'])

    def execute(self, userdata):   
        global planning_mode
        
        pltf_obj_pose = 0
        for i in range(len(userdata.rear_platform_occupied_poses)):
            if userdata.rear_platform_occupied_poses.location == base_pose_to_approach:
                pltf_obj_pose = userdata.rear_platform_occupied_poses.free_poses.pop()
                userdata.last_grasped_obj_name = platform_pose.obj_name
                
        if pltf_obj_pose == 0:
            return 'no_more_obj_for_this_pltf'
        
        
        
        #sss.move("arm", "platform_intermediate")
        # untested
        #sss.move("arm", pltf_obj_pose+"_pre")
        #
        sss.move("arm", pltf_obj_pose.platform_pose.pose, mode=planning_mode)
        
        sss.move("gripper", "close")
        rospy.sleep(3)
        
        # untested
        #sss.move("arm", pltf_obj_pose+"_pre")
        #
        #sss.move("arm", "platform_intermediate")
        sss.move("arm", "zeroposition", mode=planning_mode)
           
        return 'object_grasped'


class place_object_in_configuration_btt(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded', 'no_more_cfg_poses'],
            input_keys=['base_pose_to_approach', 'objects_goal_configuration', 'rear_platform_occupied_poses', 'last_grasped_obj_name', 'task_list'],
            output_keys=['destinaton_free_poses', 'rear_platform_occupied_poses', 'task_list'])
        
    def execute(self, userdata):
        global planning_mode
        
        if len(userdata.destinaton_free_poses) == 0:
            rospy.logerr("no more configuration poses")
            return 'no_more_cfg_poses'
        
        cfg_goal_pose = "/script_server/arm/" + userdata.objects_goal_configuration + "/" + userdata.objects_goal_configuration + "_" + userdata.destinaton_free_poses.pop()
        print "goal pose taken: ",cfg_goal_pose
        print "rest poses: ", userdata.obj_goal_configuration_poses
                
        sss.move("arm", cfg_goal_pose, mode=planning_mode)
        
        sss.move("gripper","open")
        rospy.sleep(2)
        
        #delete placed obj from occupied poses of platform
        for i in range(len(userdata.rear_platform_occupied_poses)):            
           if userdata.rear_platform_occupied_poses[i].location == userdata.base_pose_to_approach and userdata.rear_platform_occupied_poses[i].obj_name == userdata.last_grasped_obj_name:
               userdata.rear_platform_occupied_poses.remove(userdata.last_grasped_obj_name)

        #delete placed obj from task list
        for j in range(len(userdata.task_list)):
            if userdata.task_list[j].type == 'destination' and task_list.location == userdata.base_pose_to_approach:
                userdata.task_list[j].object_names.remove(userdata.last_grasped_obj_name)
                
                if len(userdata.task_list[j].object_names) == 0:
                    userdata.task_list.pop(j)
        
    
        return 'succeeded'


class place_obj_on_rear_platform_btt(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'no_more_free_poses'], 
                                   input_keys=['object_to_be_grasped', 'rear_platform_free_poses', 'rear_platform_occupied_poses', 'task_list'], 
                                   output_keys=['rear_platform_free_poses', 'rear_platform_occupied_poses', 'task_list'])

    def execute(self, userdata):   
        global planning_mode
        #sss.move("arm", "zeroposition", mode=planning_mode)
        #sss.move("arm", "platform_intermediate", mode=planning_mode)

        
        if(len(userdata.rear_platform_free_poses) == 0):
            rospy.logerr("NO more free poses on platform")
            return 'no_more_free_poses'
            
        pltf_pose = userdata.rear_platform_free_poses.pop()
        # untested
        #sss.move("arm", pltf_pose+"_pre")
        #
        sss.move("arm", pltf_pose, mode=planning_mode)
        
        
        sss.move("gripper", "open")
        rospy.sleep(2)


        #delete from task list
        for i in range(len(userdata.task_list)):
            if userdata.task_list[i].type == 'source' and userdata.task_list[i].location == userdata.base_pose_to_approach:
                userdata.task_list[i].object_names.remove(object_to_be_grasped)

        # remember what is on the platform
        obj_on_platform = Bunch(obj_name=object_to_be_grasped, platform_pose=pltf_pose)
        userdata.rear_platform_occupied_poses.append(obj_on_platform)
        
        
        #sss.move("arm", pltf_pose+"_pre")
        sss.move("arm", "platform_intermediate", mode=planning_mode)

        return 'succeeded'



class check_if_platform_has_still_objects(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['still_objs_on_robot_pltf', 'no_more_objs_on_robot_pltf'], 
                                   input_keys=['rear_platform_occupied_poses'])

    def execute(self, userdata):   

        if len(userdata.rear_platform_occupied_poses) == 0:
            return 'no_more_objs_on_robot_pltf'


        return 'still_objs_on_robot_pltf'
