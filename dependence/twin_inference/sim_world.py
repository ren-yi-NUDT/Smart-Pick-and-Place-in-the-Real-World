#!/usr/bin/env python3
import os, sys

import rospy
import numpy as np
import json
import time
import pybullet as p
import pybullet_data
import threading
from robot import ErdaijiRobot
import tqdm


SIMULATION_STEP_DELAY = 1 / 240.
ROUND_NUMBER = 3

class World:

    def __init__(self, vis = True, blocking_mode = False):
        
        self.vis = vis
        self.camera_refresh_freq = 10
        self.blocking_mode = blocking_mode
        
        urdf_dir = os.path.join(os.path.dirname(__file__), "../smart_pick_and_place_ws/src/rm_description/urdf/SingleArm")
        self.robot_path = os.path.join(urdf_dir, "rm65.urdf")
        self.robot_config_path = os.path.join(urdf_dir, "robot_config.json")
        self.robot = ErdaijiRobot((0.0, 0.0, 0.0), (0, 0, 0), robot_path = self.robot_path, config_path = self.robot_config_path, blocking_mode=self.blocking_mode, vis=self.vis)
        self.setup()
    
    def setup(self):
        self.num_step = 0
        # <<< setup bullet physics
        self.physicsClient = p.connect(p.GUI if self.vis else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        # p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
        # p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW,0)
        # p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW,0)
        # p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW,0)
        p.setGravity(0, 0, -10)
        p.resetDebugVisualizerCamera( cameraDistance=2.7, cameraYaw=63, cameraPitch=-44, cameraTargetPosition=[0,0,0])
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
        
        # load objects
        self.planeID = p.loadURDF("plane.urdf")
        self.robot.load_robot()
    
    def reset(self):
        self.num_step = 0
        self.robot.reset_robot()
    
    def step(self):
        self.num_step += 1
        self.robot.apply_actions()
        p.stepSimulation()
        rospy.sleep(SIMULATION_STEP_DELAY)
        
        # if self.num_step % self.camera_refresh_freq ==0:
        #     self.robot.update_camera()
        
        robot_state = self.robot.get_state()
        return robot_state
    
    def __del__(self):
        # 保持连接，不主动断开
        # p.disconnect(self.physicsClient)
        pass
        



if __name__ == '__main__':
    rospy.init_node('sim_world', anonymous=True)
    sim_world = World(blocking_mode=True)
    sim_world.reset()

    while not rospy.is_shutdown():
        state = sim_world.step()
        rospy.loginfo(state["ee_self"]["rm65"])

    #         print("#################3")