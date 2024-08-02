#! /usr/bin/python3
import rospy
import numpy as np
import time

import sys
sys.path.append('/home/gavin/wangjw/realman75/src/rm_robot')
# print(sys.path)

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from rm_control.rm_controller import RealmanController
# from rm_control.rm_controller import RealmanController
from copy import deepcopy as copy

REALMAN_CARTESIAN_STATE_TOPIC ='rm_driver/Pose_State'
REALMAN_JOINT_STATE_TOPIC = '/joint_states'

class DexArmControl():
    def __init__(self, record_type=None,prefix='right_', robot_type="realman"):
        
        try:
            rospy.init_node('Dexarm', disable_signals = True, anonymous=True)
        except:
            pass

        self.prefix = prefix

        if robot_type == 'realman_inspire':
            self._init_realman_control()
            self._init_inspire_control()
        elif robot_type == 'realman':
            self._init_realman_control()
        elif robot_type == 'inspire':
            self._init_inspire_control()

    # Controller initializers
    def _init_realman_control(self):
        self.realman = RealmanController(self.prefix)
        
        self.realman_joint_state = None
        rospy.Subscriber(
            REALMAN_JOINT_STATE_TOPIC,
            JointState,
            self._callback_realman_joint_state,
            queue_size = 1
        )

        state_topic = "/" + self.prefix + REALMAN_CARTESIAN_STATE_TOPIC
        self.realman_cartesian_state = None
        print(state_topic)
        rospy.Subscriber(
            state_topic,
            Pose,
            self._callback_realman_cartesian_state,
            queue_size = 1
        )

        print("Entering Realman Arm Control")

    # Rostopic callback functions
    def _callback_realman_joint_state(self, joint_state):
        self.realman_joint_state = self.realman.current_joint_state
        # print(self.realman_joint_state)

    def _callback_realman_cartesian_state(self, cartesian_state):
        self.realman_cartesian_state = cartesian_state
        # print(cartesian_state)

    def _callback_allegro_commanded_joint_state(self, joint_state):
        self.allegro_commanded_joint_state = joint_state


    # # State information functions
    # def get_hand_state(self):
    # Hidden for now


    def get_arm_cartesian_state(self):
        if self.realman_cartesian_state is None:
            print('No realman cartesian state')
            return None

        raw_cartesian_state = copy(self.realman_cartesian_state)

        cartesian_state = dict(
            position = np.array([
                raw_cartesian_state.position.x, raw_cartesian_state.position.y, raw_cartesian_state.position.z
            ], dtype = np.float32),
            orientation = np.array([
                raw_cartesian_state.orientation.x, raw_cartesian_state.orientation.y, raw_cartesian_state.orientation.z, raw_cartesian_state.orientation.w
            ], dtype = np.float32),
            timestamp = time.time()
        )
        return cartesian_state
    ##########################2024.05.30################################

    def get_arm_joint_state(self):
        if self.realman_joint_state is None:
            return None

        raw_joint_state = copy(self.realman_joint_state)

        joint_state = dict(
            position = np.array(raw_joint_state.position[:7], dtype = np.float32),
            velocity = np.array(raw_joint_state.velocity[:6], dtype = np.float32),
            effort = np.array(raw_joint_state.effort[:6], dtype = np.float32),
            timestamp = time.time()
        )
        return joint_state

    def get_arm_position(self):
        if self.realman_joint_state.position is None:
            raise ValueError('get_arm_position() is being called - Arm Position cannot be collected in Realman arms, this method should not be called')

        return np.array(self.realman_joint_state.position, dtype = np.float32)

    def get_arm_velocity(self):
        if not self.realman_joint_state.velocity:
            raise ValueError('get_arm_velocity() is being called - Arm Velocity cannot be collected in Realman arms, this method should not be called')
        
        return np.array(self.realman_joint_state.velocity, dtype = np.float32)

    def get_arm_torque(self):
        if not self.realman_joint_state.effort:
            raise ValueError('get_arm_effort() is being called - Arm Effort cannot be collected in Realman arms, this method should not be called')

        return np.array(self.realman_joint_state.effort, dtype = np.float32)

    def get_arm_cartesian_coords(self):
        if self.realman_cartesian_state is None:
            return None

        cartesian_state  =[
            self.realman_cartesian_state.position.x,
            self.realman_cartesian_state.position.y,
            self.realman_cartesian_state.position.z,
            self.realman_cartesian_state.orientation.x,
            self.realman_cartesian_state.orientation.y,
            self.realman_cartesian_state.orientation.z,
            self.realman_cartesian_state.orientation.w
        ]
        return np.array(cartesian_state)


    # Movement functions
    # def move_hand(self, allegro_angles):
    #     self.allegro.hand_pose(allegro_angles)

    # def home_hand(self):
    #     self.allegro.hand_pose(ALLEGRO_HOME_VALUES)

    # def reset_hand(self):
        # self.home_hand()

    def move_arm_joint(self, kinova_angles):
        self.realman.joint_movement(kinova_angles, False)

    def move_arm_cartesian(self, kinova_cartesian_values):
        self.kinova.cartesian_movement(kinova_cartesian_values, False, is_quaternion=True)

    def move_arm_cartesian_velocity(self, cartesian_velocity_values, duration):
        self.kinova.publish_cartesian_velocity(cartesian_velocity_values, duration)

    def home_arm(self):
        self.kinova.joint_movement(KINOVA_HOME_VALUES, False)

    def reset_arm(self):
        self.home_arm()


    # Full robot commands
    def move_robot(self, allegro_angles, kinova_angles):
        self.kinova.joint_movement(kinova_angles, False)
        self.allegro.hand_pose(allegro_angles)

    def home_robot(self):
        self.home_arm()
        self.home_hand()

if __name__ == '__main__':
    robot = DexArmControl(prefix='right_')
    time.sleep(0.1)
    # print(robot.get_arm_joint_state())
    print(robot.get_arm_cartesian_state())
    # print(robot.get_arm_position())
    # print(robot.get_arm_velocity())
    # print(get_arm_cartesian_coords())
    rospy.spin()