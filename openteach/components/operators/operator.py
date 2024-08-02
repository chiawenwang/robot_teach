from abc import ABC, abstractmethod
from openteach.components import Component
import numpy as np
import time
from matplotlib import pyplot as plt
import json
from matplotlib.animation import FuncAnimation
import asyncio
import  threading

class Operator(Component, ABC):
    @property
    @abstractmethod
    def timer(self):
        return self._timer

    # This function is used to create the robot
    @property
    @abstractmethod
    def robot(self):
        return self._robot

    # This function is the subscriber for the hand keypoints
    @property
    @abstractmethod
    def transformed_hand_keypoint_subscriber(self):
        return self._transformed_hand_keypoint_subscriber
    
    #This function is the subscriber for the arm keypoints
    @property
    @abstractmethod
    def transformed_arm_keypoint_subscriber(self):
        return self._transformed_arm_keypoint_subscriber

    # # #This function has the majority of retargeting code happening
    # @abstractmethod
    # def _apply_retargeted_angles(self):
    #     pass

    #This function applies the retargeted angles to the robot
    def stream(self):
        self.notify_component_start('{} control'.format(self.robot))
        print("Start controlling the robot hand using the Oculus Headset.\n")

        if self.return_real() is True:
            # print(self.robot)
            if self.robot.get_joint_position() is not None:
                # asyncio.run(self._apply_retargeted_angles())
                thread1 = threading.Thread(target = self._calculate_final_pose)
                thread1.start()
                print('Starting the thread:calculate_final_pose')

                thread2 = threading.Thread(target = self._process_commands)
                thread2.daemon = True
                thread2.start()
                print('Starting the thread:process_commands')

                thread3 = threading.Thread(target = self._apply_hand_joint)
                thread3.start()
                print('Starting the thread:apply_hand_joint')
