import sys
sys.path.append('/home/gavin/wangjw/realman75/src/rm_robot')
sys.path.append('/home/gavin/wangjw/realman75/src/rm_robot/rm_control')

import numpy as np
import transforms3d as t3d
import time
import copy
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from rm_control.rm_controller_python import RealmanController
import roboticstoolbox as rtb
from spatialmath import SE3
import queue
import threading

HOME_LEFT =[-15, 50, 40, 80, -20, 30, -50]
HOME_RIGHT = [15, 50, -25, 80, -20, 30, 150]

INI_LEFT = [-80 , 90, 0, -90, 170, 90, -90]

INI_RIGHT = [90,90, 0,  -90, 0, -90,150]

HOME_HAND = [1, 1, 1, 1, 1]


class DexArmControl():
    def __init__(self, prefix, record_type=None):
        self.robot =RealmanController(prefix)
        self.rtb_robot = rtb.models.Realman()
        print("Initializing:" + prefix) 

        if prefix == 'right_':
            self.robot.set_up_angle(90, 0, 0)
            pass
        elif prefix == 'left_':
            self.robot.set_up_angle(-90, 0, 0)
            pass
            # print(self.rtb_robot)

        else:
            pass

        self.prefix = prefix

        self.command_queue = queue.Queue()  # 创建一个队列来存储指令
        self.is_moving = False  # 用于跟踪机器人臂是否正在移动
        self.lock = threading.Lock()  # 创建一个锁，以确保在多线程环墨中不会发生冲突
        self.flag = 0
    # Controller initializers
    def _init_realman_control(self):
        print("Initiate Realman Arm Control")
        self.reset()
        
        # status, home_pose = self.robot.get_position_aa()
        # assert status == 0, "Failed to get robot position"
        # home_affine = self.robot_pose_aa_to_affine(home_pose)
        # Initialize timestamp; used to send messages to the robot at a fixed frequency.
        last_sent_msg_ts = time.time()

        # Initialize the environment state action tuple.
    
    # ee 6D pose
    def get_arm_cartesian_coords_euler(self):
        arm_pose = self.robot.get_current_pose()
        return arm_pose
    
    def get_arm_cartesian_coords_quat(self):
        # xyzw
        arm_pose = self.get_arm_cartesian_coords_euler()
        arm_pose_sci = copy.deepcopy(arm_pose)
        arm_pose_sci[3:7] = self.euler_to_quat(arm_pose[3:6])
        return arm_pose_sci

    def get_arm_pose_affine(self):
        arm_pose = self.robot.get_current_pose()
        arm_pose_affine = self.robot_pose_aa_to_affine(arm_pose)
        return arm_pose_affine

    # arm joint position
    def get_arm_position(self):
        # joint_position: (°) to rad
        joint_state = self.robot.get_current_joint_position()
        joint_state_rad = [x * (np.pi / 180) for x in joint_state]
        return joint_state_rad
    
    def get_arm_position_degree(self):
        # joint_position: (°) 
        joint_state = self.robot.get_current_joint_position()
        return joint_state

    def get_arm_velocity(self):
        raise ValueError('get_arm_velocity() is being called - Arm Velocity cannot be collected in Franka arms, this method should not be called')

    def get_arm_torque(self):
        raise ValueError('get_arm_torque() is being called - Arm Torques cannot be collected in Franka arms, this method should not be called')

    # def get_gripper_state(self):
    #     gripper_position=self.robot.get_gripper_position()
    #     gripper_pose= dict(
    #         position = np.array(gripper_position[1], dtype=np.float32).flatten(),
    #         timestamp = time.time()
    #     )
    #     return gripper_pose

    def get_arm_err(self):
        return self.robot.get_arm_err()
    
    def clear_arm_err(self):
        self.robot.clear_arm_err()
    
    def move_arm_joint(self, input_joint, velocity):
        self.robot.joint_movement(input_joint, velocity, trajectory_connect = 0, r = 0, block = False)

    # def receive_command(self, q):
    #     # 模拟机械臂接收指令并执行
    #     with self.lock:
    #         if not self.is_moving:
    #             self.is_moving = True
    #             # self.move_joint(q)
    #             print( q)
    #             self.flag += 1
    #             # 模拟机械臂执行指令所需的时间
    #             time.sleep(0.01)
    #             self.is_moving = False

    # def process_commands(self):
    #     # 从队列中获取指令并执行
    #     while True:
    #         try:
    #             q = self.command_queue.get(timeout=3)  # 设置超时避免无限阻塞
    #             self.receive_command(q)
    #         except queue.Empty:
    #             continue
    
    # def send_command(self, qt):
    #     for q in qt:
    #         self.command_queue.put(q)
    
    def move_joint(self, input_joint):
        # 需要设置一个等待队列
        self.robot.joint_move_touchuan(input_joint)

    def move_arm_cartesian(self, cartesian_pos, velocity=8, trajectory_connect = 0):
        self.robot.cartesian_movement(cartesian_pos, velocity=8, trajectory_connect = 0)

    def arm_control(self, cartesian_pose):
        self.robot.cartesian_control(cartesian_pose)

    def move_finger(self, finger_angle):
        if max(finger_angle) <= 1 and min(finger_angle) >= 0:
            tag = self.robot.move_finger(finger_angle)
            print('Set hand angle:',tag)
        else:
            print('Invalid finger angle')
            pass
    
    def home_hand(self):
        self.move_finger(HOME_HAND)
        
    def get_arm_joint_state(self):
        joint_positions =self.get_arm_position_degree()
        joint_state = dict(
            position = np.array(joint_positions, dtype=np.float32),
            timestamp = time.time()
        )
        return joint_state
        
    def get_cartesian_state_euler(self):
        current_pos=self.get_arm_cartesian_coords_euler()
        cartesian_state = dict(
            position = np.array(current_pos[0:3], dtype=np.float32).flatten(),
            orientation = np.array(current_pos[3:], dtype=np.float32).flatten(),
            timestamp = time.time()
        )
        return cartesian_state
    
    def get_cartesian_state(self):
        # quat_cartesian
        current_pos=self.get_arm_cartesian_coords_quat()
        cartesian_state = dict(
            position = np.array(current_pos[0:3], dtype=np.float32).flatten(),
            orientation = np.array(current_pos[3:], dtype=np.float32).flatten(),
            timestamp = time.time()
        )
        return cartesian_state


    def home_arm(self):
        if self.prefix == 'right_':
            self.move_arm_joint(HOME_RIGHT, velocity=8)
        elif self.prefix == 'left_':
            self.move_arm_joint(HOME_LEFT, velocity=8)
        else:
            pass

        return True
    
    def init_arm(self):
        if self.prefix == 'right_':
            self.move_arm_joint(INI_RIGHT, velocity=8)
        elif self.prefix == 'left_':
            self.move_arm_joint(INI_LEFT, velocity=8)
        else:
            pass

        return True

    def reset_arm(self):
        self.home_arm()

    def reset(self):
        # 根据实际情况reset
        self.home_arm()
        # self.home_hand()

    # 辅助函数
    def euler_to_quat(self, euler, order='xyzw'):
        r = R.from_euler('xyz', euler)
        if order == 'sxyz':
            pose_quat =  r.as_quat(canonical=True, scalar_first = True)
        elif order == 'xyzw':
            pose_quat =  r.as_quat(canonical=True, scalar_first = False)
        else:
            print('Invalid order')
        return pose_quat
    
    def quat_to_euler(self, quat, order = 'xyzw'):
        
        if order == 'sxyz':
            r = R.from_quat(quat, scalar_first=True)
        elif order == 'xyzw':
            r = R.from_quat(quat, scalar_first=False)
        else:
            print('Invalid order')
        return r.as_euler('xyz')

    def degree_to_rad(self, degree):
        return degree * (np.pi / 180)

    def rad_to_degree(self, rad):
        return rad * (180 / np.pi)  
    
    def quat_to_SE3(self, quat):
        trans = np.array(quat[0:3])
        rot = quat[3:7]
        rot_mat = R.from_quat(rot).as_matrix()
        homo_mat = np.zeros((4, 4))
        homo_mat[:3, :3] = rot_mat
        homo_mat[:3, 3] = trans.T
        homo_mat[3,3] = 1
        # print(homo_mat)
        se3_transform = SE3(homo_mat)

        return se3_transform
    
    def SE3_to_quat(self, homo_mat):
        rot_mat = homo_mat[:3, :3]
        trans = homo_mat[:3, 3].T
        # print('trans:', trans)
        # print(trans)
        quat = R.from_matrix(rot_mat).as_quat(canonical=True)
        # print(quat)
        res = np.concatenate((trans, quat))
        # print(res)
        return res
    
    def solve_ik_rm(self, joint, goal_pose, flag):
        tag, res = self.robot.solve_ik(joint, goal_pose, flag)
        return tag, res

    # roboticspythontool ik
    def better_ik(self,init, Tep_goal):
        # current_joint = self.get_arm_position()
        sol = self.rtb_robot.ik_LM(Tep_goal, q0 = init)  
        if sol[1] == 1:
            q_goal = sol[0]
            return q_goal
        elif sol[1] == 0:
            print('Solve IK Failed! Pass!')
            return None
        else:
            print('Strange IK')
            return None

    def gen_traj(self,current_joint, q_goal, steps = 10):
        qt = rtb.jtraj(current_joint, q_goal, steps)
        return qt.q

    def robot_pose_aa_to_affine(self,pose_aa: np.ndarray) -> np.ndarray:
        """Converts a robot pose in axis-angle format to an affine matrix.
        Args:
            pose_aa (list): [x, y, z, ax, ay, az] where (x, y, z) is the position and (ax, ay, az) is the axis-angle rotation.
            x, y, z are in mm and ax, ay, az are in radians.
        Returns:
            np.ndarray: 4x4 affine matrix [[R, t],[0, 1]]
        """

        rotation = R.from_rotvec(pose_aa[3:]).as_matrix()
        translation = np.array(pose_aa[:3])

        return np.block([[rotation, translation[:, np.newaxis]],
                        [0, 0, 0, 1]])
    
    
    def plot_data_rt(self):
        # 画图
        time_data = []
        pose_data = self.get_arm_cartesian_coords_quat()[3:6]
        num = len(pose_data)
        plot_data = [[] for _ in range(num)]
        while True:
            try:
                current_time = time.time()
                time_data.append(current_time)
                pose_data = self.get_arm_cartesian_coords_quat()[3:6]
                for i in range(num):
                    plot_data[i].append(pose_data[i])
                    plt.plot(time_data, plot_data[i], label=f'id {i+1}')
                plt.pause(0.001)  # 更新图形
                plt.ioff()
            except KeyboardInterrupt:
                break

if __name__ == '__main__':
    l_robot = DexArmControl(prefix='left_')
    r_robot = DexArmControl(prefix='right_')
    # l_robot.home_hand()
    # l_robot.clear_arm_err()

    # print('left:',  l_robot.get_arm_cartesian_coords_quat())
    # print('right:',  r_robot.get_arm_position_degree())

    r_robot.home_arm()
    # l_robot.init_arm()
    # r_robot.home_arm()
    # l_robot.init_arm()
    # r_robot.init_arm()
    # print(l_robot.get_arm_err())
