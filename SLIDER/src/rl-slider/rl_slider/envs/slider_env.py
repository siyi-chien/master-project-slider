#!/usr/bin/env python2
#/******************************************************************************
#-Author: Siyi
#-SLIDER project, Robot Intelligence Lab, Imperial College London, 2023
#******************************************************************************/
import roslaunch
import rospy
import time
import numpy as np
from restart_sim import reset_sim
import sys
sys.path.append('/home/siyi/SLIDER/devel/lib/python2.7/dist-packages')

from gym import spaces
from gym_gazebo.envs import gazebo_env
from std_srvs.srv import Empty
from  slider_controller.srv import Weight_Ref
from std_msgs.msg import Float64MultiArray
import random


class SliderEnv(gazebo_env.GazeboEnv):

    def __init__(self):
        #gazebo_env.GazeboEnv.__init__(self, "/home/siyi/SLIDER/src/slider_gazebo/launch/slider_controller.launch")
        # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        # roslaunch.configure_logging(uuid)
        # tracking_launch = roslaunch.parent.ROSLaunchParent(
        # uuid, ["/home/siyi/SLIDER/src/slider_gazebo/launch/slider_controller.launch"])
        # tracking_launch.start()
     
        rospy.init_node('training')
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.send = rospy.ServiceProxy('/slider_gazebo/setWeightRef', Weight_Ref)

        self.observation_space = spaces.Box(low=np.array([0,0,800,800]), high=np.array([30,30,30,30]), dtype=np.float64)

        self.newstate = np.array([2,2,8,12])
        self.action_space =  spaces.Box(low=np.array([-2, -2,-2,-2]), high=np.array([2, 2,2,2]), dtype=np.float64)
        self.reward_range = (-np.inf, np.inf)
        self.reset_simulation = reset_sim()
        self.state = np.array([2,2,12,12])
        self.forward_velocity = 0.2
        self.lateral_velocity = 0.0
        self.last_score = 0
    def step(self,action):


        new_forward_velocity = 0
        new_lateral_velocity = 0
      
        print("orignal_state  ",self.state)

        self.forward_velocity = new_forward_velocity
        self.lateral_velocity = new_lateral_velocity

        delta = self.state*0.05
        print("action  ",self.to5(action))
        self.newstate= self.state+self.to5(action)*delta
        print("new_state  ",self.newstate)



        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")


        rospy.wait_for_service('/slider_gazebo/setWeightRef')
        try:
            
            self.send(1e-4,1e-4,48.9035,2.9026,1243.6533,300,608.4622,300,new_forward_velocity,new_lateral_velocity)
        except (rospy.ServiceException) as e:
            print ("/slider_gazebo/setWeightRef service call failed")

            
        score = 0
        tbegin = time.time() 
        data = None
        done = False
        
        while not done:
            data = None
            try:
                data = rospy.wait_for_message("/slider_gazebo/planner_input", Float64MultiArray, timeout=0.1)
            except:
                pass

            if data is not None:
                data = data.data
                if  (data[-1]>0.08 and data[-2]>0.08) or time.time()-tbegin >=50:
                    #done = True
                    done = False
                score += -0.4*(data[1]-self.forward_velocity)**2 -1.6* (data[3]-self.lateral_velocity)**2- 0.4* (data[4]-0.7)**2
                if  (data[-1]>0.08 and data[-2]>0.08):
                    score += -200
            data = None
        if self.last_score == 0:
            reward = 0
        else:   
            reward = 10*(score - self.last_score)
        self.last_score = score
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        info = {}
        print("score  ",score)
        print("state  ",self.state)
        print("reward  ",reward)
        self.state = self.newstate 
        return self.state, reward, done,info

    def reset(self):

        self.reset_simulation.reset()
        time.sleep(0.1)
        # Unpause simulation to make observation
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")
        info = {}
        return self.state, info

    def to5(self, n):
    
        b=[]
        while True:
            s = n//5
            y = n%5
            b=b+[y-2]
            if s == 0:
                break   
            n = s

        if len(b)<4:
            b = b+[0]*(4-len(b))
        b.reverse()
        b = np.array(b)

        return b
        