#/******************************************************************************
#-Author: Siyi
#-SLIDER project, Robot Intelligence Lab, Imperial College London, 2023
#******************************************************************************/
 #!/usr/bin/env python
 
import rospy
from gazebo_msgs.srv import DeleteModel,SpawnModel, SetModelConfiguration
from geometry_msgs.msg import Pose , Point , Quaternion
from controller_manager_msgs.srv import LoadController,SwitchController
from std_srvs.srv import Empty
import time
class reset_sim():
    def __init__(self):
        self.delete_model = rospy.ServiceProxy('/gazebo/delete_model',DeleteModel)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.spawner = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
        self.load_controller = rospy.ServiceProxy("/slider_gazebo/controller_manager/load_controller", LoadController)    
        self.switch_controller = rospy.ServiceProxy("/slider_gazebo/controller_manager/switch_controller", SwitchController)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.set_configuration = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
        self.reset_counter = rospy.ServiceProxy('/slider_gazebo/reset_counter', Empty)
    def reset(self):

        rospy.wait_for_service("/gazebo/delete_model")
        try:
            self.delete_model("slider_gazebo")
        except (rospy.ServiceException) as e:
            print ("/gazebo/delete_model service call failed")
        
        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")
        time.sleep(0.1)
        rospy.wait_for_service("/gazebo/spawn_urdf_model")
        try:    
            self.spawner("slider_gazebo", open("/home/siyi/SLIDER/src/slider_gazebo/urdf/SLIDER_ROTOGAIT_FOOT_pin.urdf", "r").read(), "/",Pose(position= Point(0,0,0),orientation=Quaternion(0,0,0,1)),"world")
        except (rospy.ServiceException) as e:
            print ("/gazebo/spawn_urdf_model service call failed")

        rospy.wait_for_service("/gazebo/set_model_configuration")
        try:
            self.set_configuration("slider_gazebo","robot",["left_hip_pitch_joint","left_hip_slide_joint","left_hip_roll_joint","right_hip_pitch_joint","right_hip_slide_joint","right_hip_roll_joint","left_ankle_pitch_joint","left_ankle_roll_joint","right_ankle_pitch_joint","right_ankle_roll_joint"],[0,0,0,0,0,0,0,0,0,0])
        except (rospy.ServiceException) as e:
            print ("/gazebo/set_model_configuration service call failed")
            
        rospy.wait_for_service("/slider_gazebo/controller_manager/load_controller")
        try:    
            self.load_controller("joint_state_controller")
        except (rospy.ServiceException) as e:
            print ("joint_state_controller service call failed")


        rospy.wait_for_service("/slider_gazebo/controller_manager/load_controller")
        try:    
            self.load_controller("left_hip_pitch_torque_controller")
        except (rospy.ServiceException) as e:
            print (" left_hip_pitch_torque_controller service call failed")

        rospy.wait_for_service("slider_gazebo//controller_manager/load_controller")
        try:
            self.load_controller("left_hip_slide_torque_controller")
        except (rospy.ServiceException) as e:
            print (" left_hip_slide_torque_controller service call failed")


        rospy.wait_for_service("/slider_gazebo/controller_manager/load_controller")
        try:
            self.load_controller("left_hip_roll_torque_controller")
        except (rospy.ServiceException) as e:
            print (" left_hip_roll_torque_controller service call failed")


        rospy.wait_for_service("/slider_gazebo/controller_manager/load_controller")
        try:
            self.load_controller("right_hip_pitch_torque_controller")
        except (rospy.ServiceException) as e:
            print (" right_hip_pitch_torque_controller service call failed")
        
        rospy.wait_for_service("/slider_gazebo/controller_manager/load_controller")
        try:
            self.load_controller("right_hip_slide_torque_controller")
        except (rospy.ServiceException) as e:
            print (" right_hip_slide_torque_controller service call failed")


        rospy.wait_for_service("/slider_gazebo/controller_manager/load_controller")
        try:
            self.load_controller("right_hip_roll_torque_controller")
        except (rospy.ServiceException) as e:
            print (" right_hip_roll_torque_controller service call failed")

        rospy.wait_for_service("/slider_gazebo/controller_manager/load_controller")
        try:
            self.load_controller("left_ankle_pitch_torque_controller")
        except (rospy.ServiceException) as e:
            print (" right_hip_roll_torque_controller service call failed")

        rospy.wait_for_service("/slider_gazebo/controller_manager/load_controller")
        try:
            self.load_controller("left_ankle_roll_torque_controller")
        except (rospy.ServiceException) as e:
            print (" right_hip_roll_torque_controller service call failed")

        rospy.wait_for_service("/slider_gazebo/controller_manager/load_controller")
        try:
            self.load_controller("right_ankle_pitch_torque_controller")
        except (rospy.ServiceException) as e:
            print (" right_hip_roll_torque_controller service call failed")

        rospy.wait_for_service("/slider_gazebo/controller_manager/load_controller")
        try:
            self.load_controller("right_ankle_roll_torque_controller")
        except (rospy.ServiceException) as e:
            print (" right_hip_roll_torque_controller service call failed")

        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause service call failed")

        rospy.wait_for_service("/slider_gazebo/controller_manager/switch_controller")
        try:
            self.switch_controller(start_controllers=["joint_state_controller"],strictness=2)
            self.switch_controller(start_controllers=["left_hip_pitch_torque_controller"],strictness=2)
            self.switch_controller(start_controllers=["left_hip_slide_torque_controller"],strictness=2)
            self.switch_controller(start_controllers=["left_hip_roll_torque_controller"],strictness=2)
            self.switch_controller(start_controllers=["right_hip_pitch_torque_controller"],strictness=2)
            self.switch_controller(start_controllers=["right_hip_slide_torque_controller"],strictness=2)
            self.switch_controller(start_controllers=["right_hip_roll_torque_controller"],strictness=2)
            self.switch_controller(start_controllers=["left_ankle_pitch_torque_controller"],strictness=2)
            self.switch_controller(start_controllers=["left_ankle_roll_torque_controller"],strictness=2)
            self.switch_controller(start_controllers=["right_ankle_pitch_torque_controller"],strictness=2)
            self.switch_controller(start_controllers=["right_ankle_roll_torque_controller"],strictness=2)
        except (rospy.ServiceException) as e:
            print ("/slider_gazebo/controller_manager/switch_controller service call failed")


        rospy.wait_for_service('/slider_gazebo/reset_counter')
        try:
                self.reset_counter()
        except (rospy.ServiceException) as e:
                print ("/slider_gazebo/reset_counter service call failed")
   
   




    
    
    


    

     