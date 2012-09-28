#!/usr/bin/python
import roslib; roslib.load_manifest('raw_basic_transportation_test_iros2012')
import rospy

import smach
import smach_ros

from simple_script_server import *
sss = simple_script_server()

planning_mode=""


def print_task_spec(task_list):
    
    rospy.loginfo("task spec")
    for task in task_list:
        rospy.loginfo("      %s %s %s ", task.location, task.object_names, task.type)
    
    return

def print_occupied_platf_poses(poses):
    
    rospy.loginfo("occupied_platform poses: ")
    for item in poses:
        rospy.loginfo("     %s", item.obj_name)
    
    return



class Bunch:
    def __init__(self, **kwds):
         self.__dict__.update(kwds)

class select_object_to_be_grasped(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['obj_selected', 'no_obj_selected','no_more_free_poses_at_robot_platf'],
            input_keys=['recognized_objects', 'objects_to_be_grasped', 'object_to_be_grasped', 'rear_platform_free_poses'],
            output_keys=['object_to_be_grasped'])
        
    def execute(self, userdata):
        
        if(len(userdata.rear_platform_free_poses) == 0):
            rospy.logerr("NO more free poses on robot rear platform")
            return 'no_more_free_poses_at_robot_platf'

        
        for rec_obj in userdata.recognized_objects:
            for obj_grasp in userdata.objects_to_be_grasped:
                if rec_obj.name == obj_grasp:
                    userdata.object_to_be_grasped = rec_obj
                    print "selected obj: ", userdata.object_to_be_grasped.name
                    return 'obj_selected'
                
        return 'no_obj_selected'


class select_btt_subtask(smach.State):
    def __init__(self, type=""):
        smach.State.__init__(self, 
            outcomes=['task_selected','no_more_task_for_given_type'],
            input_keys=['task_list'],
            output_keys=['base_pose_to_approach', 'objects_to_be_grasped'])
        
        self.type = type
        
    def execute(self, userdata):
        
        print_task_spec(userdata.task_list)
        
        # check if there is a empty obj_names list for a given locaction and remove it
        for i in range(len(userdata.task_list)):
            if len(userdata.task_list[i].object_names) == 0:
                userdata.task_list.pop(i)
                break;
            
        print_task_spec(userdata.task_list)
                
        # select next task
        for i in range(len(userdata.task_list)):
            if userdata.task_list[i].type == self.type:     
                userdata.base_pose_to_approach = userdata.task_list[i].location
                userdata.objects_to_be_grasped = userdata.task_list[i].object_names
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
            output_keys=['base_pose_to_approach', 'objects_to_be_grasped', 'objects_goal_configuration'])
        
        
    def execute(self, userdata):
        print_task_spec(userdata.task_list)
        print_occupied_platf_poses(userdata.rear_platform_occupied_poses)
        
        for task in userdata.task_list:
            for obj in userdata.rear_platform_occupied_poses:
                if task.type == 'destination':
                    for dest in task.object_names:
                        if dest == obj.obj_name:
                            userdata.base_pose_to_approach = task.location
                            userdata.objects_goal_configuration = task.object_config
                            return 'success'
                        
        if len(userdata.task_list) > 0:
            userdata.base_pose_to_approach = task.location
            userdata.objects_goal_configuration = task.object_config
            return 'success'
        
        
        return 'no_more_dest_tasks'
                        

class setup_btt(smach.State):
    def __init__(self, type=""):
        smach.State.__init__(self, 
            outcomes=['success'],
            input_keys=['task_list', 'destinaton_free_poses'],
            output_keys=['destinaton_free_poses'])
        
        
    def execute(self, userdata):
        print_task_spec(userdata.task_list)
        
        for task in userdata.task_list:
            if task.type == 'destination':
                poses = ['1', '2', '3', '4', '5']
                loc_free_poses = Bunch(location = task.location, free_poses = poses)
                userdata.destinaton_free_poses.append(loc_free_poses)
                
        print_task_spec(userdata.task_list)
        return 'success'
        
        
class grasp_obj_from_pltf_btt(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['object_grasped', 'no_more_obj_for_this_workspace'], 
                             input_keys=['rear_platform_occupied_poses', 'rear_platform_free_poses', 'base_pose_to_approach', 'task_list', 'last_grasped_obj'],
                             output_keys=['rear_platform_occupied_poses', 'rear_platform_free_poses', 'last_grasped_obj'])

    def execute(self, userdata):   
        global planning_mode
        print_occupied_platf_poses(userdata.rear_platform_occupied_poses)
        print_task_spec(userdata.task_list)
        
        #get objects to be placed at current workstation
        objs_for_this_ws = []
        for task in userdata.task_list:
            if task.type == 'destination' and task.location == userdata.base_pose_to_approach:
                objs_for_this_ws = task.object_names
        
        pltf_obj_pose = 0
        stop = False
        for i in range(len(userdata.rear_platform_occupied_poses)):
            for obj_name in objs_for_this_ws:
                rospy.loginfo("userdata.rear_platform_occupied_poses[i].obj_name: %s     obj_name: %s", userdata.rear_platform_occupied_poses[i].obj_name, obj_name)
                if userdata.rear_platform_occupied_poses[i].obj_name == obj_name:
                    pltf_obj_pose =  userdata.rear_platform_occupied_poses.pop(i)
                    userdata.rear_platform_free_poses.append(pltf_obj_pose)
                    userdata.last_grasped_obj = pltf_obj_pose.obj_name
                    print "LAST OBJ: ", userdata.last_grasped_obj
                    stop = True
                    break;
            if stop:
                break
                
        print_occupied_platf_poses(userdata.rear_platform_occupied_poses)        
        
        if pltf_obj_pose == 0:
            return 'no_more_obj_for_this_workspace'
        
       
       
	print "plat_pose: ", pltf_obj_pose.platform_pose
	print "plat_name: ", pltf_obj_pose.obj_name
        if planning_mode != "planned":
            sss.move("arm", "platform_intermediate")
            sss.move("arm", str(pltf_obj_pose.platform_pose)+"_pre")

        sss.move("arm", str(pltf_obj_pose.platform_pose), mode=planning_mode)
        
        sss.move("gripper", "close")
        rospy.sleep(3)
       
        if planning_mode != "planned":
            sss.move("arm", str(pltf_obj_pose.platform_pose)+"_pre")
            sss.move("arm", "platform_intermediate")

        sss.move("arm", "zeroposition", mode=planning_mode)
           
        return 'object_grasped'


class place_object_in_configuration_btt(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded', 'no_more_cfg_poses'],
            input_keys=['base_pose_to_approach', 'objects_goal_configuration', 'last_grasped_obj', 'task_list', 'destinaton_free_poses', 'obj_goal_configuration_poses'],
            output_keys=['destinaton_free_poses', 'task_list'])
        
    def execute(self, userdata):
        global planning_mode
        
        print_task_spec(userdata.task_list)
        
        if len(userdata.destinaton_free_poses) == 0:
            rospy.logerr("no more configuration poses")
            return 'no_more_cfg_poses'
        
        pose = 0
        for i in range(len(userdata.destinaton_free_poses)):
            if userdata.destinaton_free_poses[i].location == userdata.base_pose_to_approach:
                pose = userdata.destinaton_free_poses[i].free_poses.pop()
        
        
        cfg_goal_pose = userdata.objects_goal_configuration + "/" + userdata.objects_goal_configuration + "_" + pose
        print "goal pose taken: ",cfg_goal_pose
        print "rest poses: ", userdata.destinaton_free_poses[i].free_poses
                
        sss.move("arm", cfg_goal_pose, mode=planning_mode)
        
        sss.move("gripper","open")
        rospy.sleep(2)
        
        
        #delete placed obj from task list
        for j in range(len(userdata.task_list)):
            if userdata.task_list[j].type == 'destination' and userdata.task_list[j].location == userdata.base_pose_to_approach:
                print "lllll: ", userdata.last_grasped_obj
                print "list: ", userdata.task_list[j].object_names 
                userdata.task_list[j].object_names.remove(userdata.last_grasped_obj)
                
                if len(userdata.task_list[j].object_names) == 0:
                    userdata.task_list.pop(j)
                
                break
            
        print_task_spec(userdata.task_list)
        
    
        return 'succeeded'


class place_obj_on_rear_platform_btt(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'no_more_free_poses'], 
                                   input_keys=['object_to_be_grasped', 'rear_platform_free_poses', 'rear_platform_occupied_poses', 'task_list', 'base_pose_to_approach'], 
                                   output_keys=['rear_platform_free_poses', 'rear_platform_occupied_poses', 'task_list'])

    def execute(self, userdata):   
        global planning_mode
        print_occupied_platf_poses(userdata.rear_platform_occupied_poses)
        
        if planning_mode != "planned":
            sss.move("arm", "zeroposition", mode=planning_mode)
            sss.move("arm", "platform_intermediate", mode=planning_mode)

        print_task_spec(userdata.task_list)
        
        if(len(userdata.rear_platform_free_poses) == 0):
            rospy.logerr("NO more free poses on platform")
            return 'no_more_free_poses'

      # Removing free poses          
        pltf_pose = userdata.rear_platform_free_poses.pop();

        if planning_mode != "planned":
            sss.move("arm", pltf_pose.platform_pose+"_pre")
        
        sss.move("arm", pltf_pose.platform_pose, mode=planning_mode)
        
        
        sss.move("gripper", "open")
        rospy.sleep(2)

        print "object_to_be_grasped: ", userdata.object_to_be_grasped.name
        #delete from task list
        for i in range(len(userdata.task_list)):
            if userdata.task_list[i].type == 'source' and userdata.task_list[i].location == userdata.base_pose_to_approach:
                print "obj_names:", userdata.task_list[i].object_names
                userdata.task_list[i].object_names.remove(userdata.object_to_be_grasped.name)
                print "obj_names after remove from task list: ", userdata.task_list[i].object_names
                break
        
        print_task_spec(userdata.task_list)
        
        # remember what is on the platform
        pltf_pose.obj_name = userdata.object_to_be_grasped.name
        userdata.rear_platform_occupied_poses.append(pltf_pose)
        
        print_occupied_platf_poses(userdata.rear_platform_occupied_poses)
        
        if planning_mode != "planned":
            sss.move("arm", pltf_pose.platform_pose+"_pre")
        
        sss.move("arm", "platform_intermediate", mode=planning_mode)

        return 'succeeded'



class check_if_platform_has_still_objects(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['still_objs_on_robot_pltf', 'no_more_objs_on_robot_pltf'], 
                                   input_keys=['rear_platform_occupied_poses'])

    def execute(self, userdata):   
        print_occupied_platf_poses(userdata.rear_platform_occupied_poses)
        
        if len(userdata.rear_platform_occupied_poses) == 0:
            return 'no_more_objs_on_robot_pltf'


        return 'still_objs_on_robot_pltf'


class skip_pose(smach.State):

    def __init__(self, type=""):
        smach.State.__init__(self, outcomes=['pose_skipped'], 
                                   input_keys=['task_list', 'base_pose_to_approach'],
                                   output_keys=['task_list'])
        self.type = type

    def execute(self, userdata):   
        
        print_task_spec(userdata.task_list)
        
        for i in range(len(userdata.task_list)):
            if userdata.task_list[i].type == self.type and userdata.task_list[i].location == userdata.base_pose_to_approach:
                temp = userdata.task_list.pop(i)
                userdata.task_list.append(temp)
                return 'pose_skipped'
            
        print_task_spec(userdata.task_list)
                
        return 'pose_skipped'


