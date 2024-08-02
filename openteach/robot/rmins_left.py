from openteach.ros_links.bimanual_rmins import DexArmControl
from openteach.ros_links.bimanual_rmins import HOME_LEFT, HOME_RIGHT
from openteach.robot.robot import RobotWrapper
from openteach.utils.network import ZMQKeypointSubscriber
import numpy as np
import transforms3d as t3d
import time
from matplotlib import pyplot as plt
import json
from matplotlib.animation import FuncAnimation

from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation
from scipy.spatial.transform import Slerp

class RMINSLeft(RobotWrapper):
    def __init__(self, prefix ,record_type=None):
        self._controller = DexArmControl(prefix=prefix,record_type=record_type)
        self._data_frequency = 90


    @property
    def recorder_functions(self):
        return {
            'joint_states': self.get_joint_state_from_socket,
            'cartesian_states': self.get_cartesian_state_from_socket,
            # 'gripper_states': self.get_gripper_state_from_socket,
            'actual_cartesian_states': self.get_robot_actual_cartesian_position,
            'actual_joint_states': self.get_robot_actual_joint_position,
            # 'actual_gripper_states': self.get_gripper_state,
            'commanded_cartesian_state': self.get_cartesian_commanded_position

        }

    @property
    def name(self):
        return 'left_arm'

    @property
    def data_frequency(self):
        return self._data_frequency

    # State information functions
    def get_arm_err(self):
        return self._controller.get_arm_err()
    
    def clear_arm_err(self):
        self._controller.clear_arm_err()
        print('Cleared error')
    
    def get_joint_state(self):
        print("Getting joint state")
        return self._controller.get_arm_joint_state()
    
    def get_joint_velocity(self):
        pass

    def get_joint_torque(self):
        pass

    def get_cartesian_state(self):
        print("Getting cartesian state")
        return self._controller.get_cartesian_state()

    def get_joint_position(self):
        # joint_position: (rad)
        # print("Getting joint position")
        return self._controller.get_arm_position()
    
    def get_joint_position_degree(self):
        # joint_position: (°)
        return self._controller.get_arm_position_degree()
    
    def get_cartesian_position_euler(self):
        # x,y,z,dx,dy,dz
        # print("Getting cartesian position")
        return self._controller.get_arm_cartesian_coords_euler()
    
    def get_cartesian_position(self):
        # quat
        # print("Getting cartesian position")
        return self._controller.get_arm_cartesian_coords_quat()

    def reset_home(self):
        self._controller._init_realman_control()
        return True
    
    # Movement functions
    def home(self):
        print(' Doing home')
        return self._controller.home_arm()
    
    def home_total(self):
        print(' Doing home')
        self._controller.home_arm()
        flag = True
        while flag:
            current = self.get_joint_position_degree()
            if np.linalg.norm(np.array(current) - np.array(HOME_LEFT)) < 0.1:
                flag = False
                time.sleep(0.5)
                continue
            print('Doing home')
        self._controller.home_hand()

    def move(self, input_angles,vel):
        print('Doing move')
        self._controller.move_arm_joint(input_angles, vel)

    def joint_move(self, input_angles):
        # print(' Doing joint_move')
        self._controller.move_joint(input_angles)

    def hand_joint_move(self, input_angles):
        # print(' Doing joint_move')
        self._controller.move_finger(input_angles)

    def move_coords(self, cartesian_coords, vel=3):
        print('Doing move_coords')
        self._controller.move_arm_cartesian(cartesian_coords, vel)

    def arm_control(self, cartesian_coords):
        print(' Doing arm_control')
        self._controller.arm_control(cartesian_coords)


    def control_arm_joint(self, q_goal, steps=20):
        qt = self._controller.gen_traj( q_goal, steps)
        qt_degree = [self._controller.rad_to_degree(q) for q in qt]
        time_start = time.time()
        for q in qt_degree:
            self._controller.receive_command(q)
        print("send_Time: ", time.time() - time_start)

    def solve_ik(self, init, fina_pose):
        fina_pose[2] += 0.5
        goal_Tep = self._controller.quat_to_SE3(fina_pose)
        q_goal = self._controller.better_ik(init,goal_Tep)
        return q_goal
    
    def solve_ik_rm(self, joint, goal,flag):
        # goal[2] += 0.5
        # print('joints now:',joint, flush=True)
        # print('goal:',goal[:3], flush=True)
        tag, res = self._controller.solve_ik_rm(joint,goal,flag)
        return tag, res
    
    # def solve_ik_rm(self, joint, goal_pose):s

    
    def joint_distance(self, joint1, joint2):
        return np.linalg.norm(np.array(joint1) - np.array(joint2))


    # def get_gripper_state_from_socket(self):
    #     self._gripper_state_subscriber = ZMQKeypointSubscriber(
    #             host = '10.19.216.156', 
    #             port = 8115,
    #             topic = 'gripper_left'
    #         )
    #     gripper_state = self._gripper_state_subscriber.recv_keypoints()
    #     gripper_state_dict= dict(
    #         gripper_position = np.array(gripper_state, dtype=np.float32),
    #         timestamp = time.time()
    #     )
    #     return gripper_state_dict
    

    def get_cartesian_state_from_socket(self):
        self.cartesian_state_subscriber = ZMQKeypointSubscriber(
                host = '192.168.31.15', 
                port = 8116,
                topic = 'cartesian'
            )
        cartesian_state = self.cartesian_state_subscriber.recv_keypoints()
        cartesian_state_dict= dict(
            cartesian_position = np.array(cartesian_state, dtype=np.float32),
            timestamp = time.time()
        )
        return cartesian_state_dict
    
    def get_joint_state_from_socket(self):
        self._joint_state_subscriber = ZMQKeypointSubscriber(
                host = '192.168.31.15', 
                port = 8117,
                topic = 'joint'
            )
        joint_state = self._joint_state_subscriber.recv_keypoints()
        # gripper_state = self._controller.robot.get_gripper_position()[1]
        joint_state_dict= dict(
            joint_position = np.array(joint_state, dtype=np.float32),
            timestamp = time.time()
        )
        #self._controller.set_gripper_status(gripper_state)
        return joint_state_dict
    
    def get_cartesian_commanded_position(self):
        self.cartesian_state_subscriber = ZMQKeypointSubscriber(
                host = '192.168.31.15', 
                port = 8121,
                topic = 'cartesian'
            )
        cartesian_state = self.cartesian_state_subscriber.recv_keypoints()
        # gripper_state = self._controller.robot.get_gripper_position()[1]
        cartesian_state_dict= dict(
            commanded_cartesian_position = np.array(cartesian_state, dtype=np.float32),
            timestamp = time.time()
        )
        #self._controller.set_gripper_status(gripper_state)
        return cartesian_state_dict
    
    def get_robot_actual_cartesian_position(self):
        cartesian_state=self.get_cartesian_position()
        cartesian_dict = dict(
            cartesian_position = np.array(cartesian_state, dtype=np.float32),
            timestamp = time.time()
        )
        
        return cartesian_dict
    
    def get_robot_actual_joint_position(self):
        joint_state_dict=self._controller.get_arm_joint_state()
        return joint_state_dict
    
    # def get_gripper_state(self):
    #     gripper_state_dict= self._controller.get_gripper_state()
    #     return gripper_state_dict

    def real_time_plot(self,data_num):
        # 初始化Matplotlib图形
        fig, ax = plt.subplots()
        lines = [ax.plot([], [], lw=2)[0] for _ in range(4)]
        ax.set_xlim(0, 300)  # 设置x轴范围，根据需要调整
        ax.set_ylim(-3, 3)  # 设置y轴范围，根据需要调整
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('queternion_ee')

        # 初始化数据
        time_data = []
        y_data = [[] for _ in range(data_num)]

        def update(frame):
            try:
                current_time = time.time() - start_time
                time_data.append(current_time)
                joint_angles = self.get_cartesian_position()[3:]  # 假设left_arm有一个获取关节角度的方法
                for i in range(data_num):
                    y_data[i].append(joint_angles[i])
                    lines[i].set_data(time_data, y_data[i])
                return lines
            except KeyboardInterrupt:
                print("程序终止")

        # 使用FuncAnimation实时更新图形
        start_time = time.time()
        ani = FuncAnimation(fig, update, frames=None, blit=True)
        plt.show()


    # def arm_control_insert(self, start, cartesian_coords,steps = 10):
    #     pose = self._controller.interpolate_poses(start, cartesian_coords, steps)
    #     for p in pose:       
    #         self._controller.arm_control(p)
    #         time.sleep(0.005)

    # # def calculate_insert(self, start, cartesian_coords,steps = 10):
    # #         start_pose = np.array(start)
    # #         end_pose = np.array(cartesian_coords)
    # #         pos_interp = np.zeros((steps, 3))
    # #         for i in range(3):
    # #             pos_interp[:, i] = np.linspace(start_pose[i], end_pose[i], steps)

    # #         rot_start = Rotation.from_quat(start_pose[3:])
    # #         rot_end = Rotation.from_quat(end_pose[3:])

    # #         # interp_rotations = Rotation.from_quat([rot_start.as_quat(), rot_end.as_quat()]).slerp(np.linspace(0, 1, steps)).as_quat()

    # #         # 计算插值后的四元数
    # #         # interp_quats = interp_rotations

    # #         print(interp_quats)


    # def control_arm(self, cartesian_coords, threshold=0.05):
    #     # 先判断距离远近，再决定用什么控制
    #     # if self.get_arm_err() == 4099:
    #     #     print('error!!!!!!!')
    #     #     self.clear_arm_err()
    #     current = self.get_cartesian_position()
    #     if self.pose_distance(current, cartesian_coords) > threshold:
    #         print('Doing move_coords')
    #         arm_pose = []
    #         arm_pose[:3] = cartesian_coords[:3]
    #         arm_pose[3:6] = t3d.euler.quat2euler(cartesian_coords[3:])
    #         # print(arm_pose)
    #         self.move_coords(arm_pose, vel=30)
    #         time.sleep(0.1)
    #         # self.arm_control_insert(current, cartesian_coords, steps=60)
    #         # pass

    #     else:
    #         print('Doing arm_control_insert_short')
    #         self.arm_control_insert(current, cartesian_coords, steps=15)


    # def pose_distance(self, pose1, pose2):
    #     # 提取位置和位移
    #     pos1 = np.array(pose1[:3])
    #     pos2 = np.array(pose2[:3])
    #     ori1 = np.array(pose1[3:])
    #     ori2 = np.array(pose2[3:])
        
    #     # 计算位置变化量之间的欧氏距离
    #     pos_distance = np.linalg.norm(pos1 - pos2)
    #     print('pos_distance:', pos_distance)
        
    #     # 计算位移之间的欧氏距离
    #     disp_distance = np.linalg.norm(ori1 - ori2)
    #     print('disp_distance:', disp_distance)
        
    #     # 返回总距离
    #     distance = np.sqrt(pos_distance ** 2 + disp_distance ** 2)
    #     # distance = np.sqrt(0.8 * pos_distance ** 2 + 0.2 * disp_distance ** 2)
        
    #     return distance

    
if __name__ == '__main__':
    left_arm = RMINSLeft(prefix='left_')
    # joint_angles = left_arm.get_joint_position_degree()
    # print(joint_angles)
    # target = joint_angles.copy()

    # for i in range(10):
    #     target[0] += 1
    #     left_arm.joint_move(target)
    #     time.sleep(0.005)
    # left_arm.clear_arm_err()

    # time_start = time.time()
    # current = left_arm.get_cartesian_position()
    # print('check_Time:', time.time() - time_start)

    # current[2] += 0.1
    # time_start = time.time()
    # left_arm.control_arm(current, threshold=1)
    # print('control_Time:', time.time() - time_start)
    # left_arm.calculate_insert(HOME_LEFT, [0.54, 0.14, 0.02, 0.25, 0.79, 0.06, 0.55], steps=10)

    # left_arm.reset_home()
    # try:
    #     while True:
    #         print(left_arm.get_cartesian_position()[3:])
    #         time.sleep(0.01)
            
    # except KeyboardInterrupt:
    #     print("程序终止")

    # left_arm.real_time_plot(4)


    # left_arm.clear_arm_err()

    # cart_position = left_arm.get_cartesian_position()
    # print(cart_position)
    # print(left_arm.get_joint_position_degree())
    # cart_position[0] += 0.05
    # cart_position[1] -= 0.02
    # cart_position[2] -= 0.02
    # cart_position[3] -= 0.01
    # left_arm.control_arm(cart_position, threshold=0.01)
    


    # # 记录数据
    # current = left_arm.get_joint_position_degree()
    # current = left_arm.get_cartesian_position()
    # print(current)
    # position = []
    # time.sleep(2)
    # try:
    #     while True:
    #         position.append(left_arm.get_joint_position_degree())
    #         time.sleep(0.002)
            
    # except KeyboardInterrupt:
    #     print("程序终止")

    # position_file = "position_data.txt"

    # with open(position_file, "w") as f:
    #     json.dump(position, f)



    # # 还原数据
    # position = []
    # position_file = "position_data.txt"
    # with open(position_file, "r") as f:
    #     position = json.load(f)

    # # run the recorded data
    # # print(position[0])
    # # left_arm.move(position[0], vel = 20)
    # # time.sleep(5)

    # # for pos in position:
    # #     left_arm.joint_move(pos)
    # #     time.sleep(0.005)

    # distances = []
    # for i in range(len(position)-1):
    #     distances.append(left_arm.joint_distance(position[i], position[i+1]))
    #     if distances[-1] > 1:
    #         print(i)
    #         print(distances[-1])
    #         print(position[i])
    #         print(position[i+1])

    # plt.plot(distances)
    # plt.show()


