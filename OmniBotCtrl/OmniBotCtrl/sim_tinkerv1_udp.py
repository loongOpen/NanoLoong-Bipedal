# 添加global_config的搜索路径
import sys  
import os  
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))  
sys.path.append(parent_dir)  

import math
import numpy as np
import mujoco, mujoco_viewer
from collections import deque
from scipy.spatial.transform import Rotation as R
from global_config import ROOT_DIR
from configs.tinker_constraint_him_trot import TinkerConstraintHimRoughCfg 
import time
import threading
import socket
import struct
import select
import termios
import tty

action_count = 0
last_print_time = time.time()

default_dof_pos = [0.0,-0.0,0.56,-1.12,0.57,  0.0,0.0,0.56,-1.12,0.57] # tinkerv1
# default_dof_pos = [0.0,0.08,0.56,-1.12,-0.57,  0.0,-0.08,-0.56,1.12,0.57] # tinkerv2
action_rl = default_dof_pos

current_gait = 0  # 0: 初始化站立

class cmd:
    vx = 0.0
    vy = 0.0
    dyaw = 0.0

def quaternion_to_euler_array(quat):
    # Ensure quaternion is in the correct format [x, y, z, w]
    x, y, z, w = quat
    
    # Roll (x-axis rotation)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = np.arctan2(t0, t1)
    
    # Pitch (y-axis rotation)
    t2 = +2.0 * (w * y - z * x)
    t2 = np.clip(t2, -1.0, 1.0)
    pitch_y = np.arcsin(t2)
    
    # Yaw (z-axis rotation)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = np.arctan2(t3, t4)
    
    # Returns roll, pitch, yaw in a NumPy array in radians
    return np.array([roll_x, pitch_y, yaw_z])

def get_obs(data):
    '''Extracts an observation from the mujoco data structure
    '''
    q = data.qpos.astype(np.double)
    dq = data.qvel.astype(np.double)
    quat = data.sensor('orientation').data[[1, 2, 3, 0]].astype(np.double) # 姿态eu_ang
    r = R.from_quat(quat)
    v = r.apply(data.qvel[:3], inverse=True).astype(np.double)  # In the base frame
    omega = data.sensor('angular-velocity').data.astype(np.double) #角速度
    gvec = r.apply(np.array([0., 0., -1.]), inverse=True).astype(np.double)
    
    # 将 q, dq, quat, r, v, omega, gvec 按照 State_Rl.cpp需要的方式组装起来
    return (q, dq, quat, v, omega, gvec)

def pd_control(target_q, q, kp, target_dq, dq, kd):
    '''Calculates torques from position commands
    '''
    return (target_q - q) * kp + (target_dq - dq) * kd

def _low_pass_action_filter(actions,last_actions):
    flt =0.1
    actons_filtered = last_actions * flt + actions * (1-flt)
    return actons_filtered

# UDP数据结构
class MsgRequest:
    def __init__(self):
        self.omega = [0.0] * 3
        self.eu_ang = [0.0] * 3
        self.command = [0.0] * 3
        self.q = [0.0] * 10
        self.dq = [0.0] * 10
        self.gait_type = [0]

class MsgResponse:
    def __init__(self):
        self.q_exp = [0.0] * 10

# UDP消息编码解码函数
def encode_msg_request(msg):
    """编码请求消息到字节流"""
    data = bytearray()
    # omega[3]
    for val in msg.omega:
        data.extend(struct.pack('f', val))
    # eu_ang[3]
    for val in msg.eu_ang:
        data.extend(struct.pack('f', val))
    # command[3]
    for val in msg.command:
        data.extend(struct.pack('f', val))
    # q[10]
    for val in msg.q:
        data.extend(struct.pack('f', val))
    # dq[10]
    for val in msg.dq:
        data.extend(struct.pack('f', val))
    # gait_type[1]
    for val in msg.gait_type:
        data.extend(struct.pack('i', val))
    return data # 120字节

def decode_msg_response(data):
    """从字节流解码响应消息"""
    if len(data) < 40:  # 10个float * 4字节
        return None
    
    msg = MsgResponse()
    for i in range(10):
        msg.q_exp[i] = struct.unpack('f', data[i*4:(i+1)*4])[0]
    return msg

def check_keyboard():
    """检查键盘输入，非阻塞方式"""
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        key = sys.stdin.read(1)
        return key # w a s d
    return None

def run_mujoco(cfg):
    global action_rl, default_dof_pos, current_gait
    """
    通过UDP作为通用接口传输传感器原始数据，接受网络直接输出
    """
    # UDP服务器初始化
    # udp_addr = ('127.0.0.1', 8888)  # 本地
    udp_addr = ('0.0.0.0', 8848)  # k1
    
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # 创建UDP socket
    udp_socket.bind(udp_addr) # 绑定到指定地址和端口
    # udp_socket.settimeout(0.001)  # 设置短超时，避免阻塞仿真循环
    udp_socket.setblocking(False) # 非阻塞模式
    print(f"UDP服务器启动在 {udp_addr}")

    # 非阻塞输入
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())

        model = mujoco.MjModel.from_xml_path(cfg.sim_config.mujoco_model_path)#载入初始化位置由XML决定
        model.opt.timestep = cfg.sim_config.dt
        data = mujoco.MjData(model)
        mujoco.mj_step(model, data)

        viewer = mujoco_viewer.MujocoViewer(
            model, 
            data,
            width=1280,
            height=720
        )

        target_q = np.zeros((cfg.env.num_actions), dtype=np.double)     # 10
        action = np.zeros((cfg.env.num_actions), dtype=np.double)       # 10
        action_flt = np.zeros((cfg.env.num_actions), dtype=np.double)   # 10
        last_actions = np.zeros((cfg.env.num_actions), dtype=np.double) # 10
        hist_obs = deque()
        for _ in range(cfg.env.history_len):
            hist_obs.append(np.zeros([1, cfg.env.n_proprio], dtype=np.double)) # 39

        count_lowlevel = 0

        render_interval = int(1 / (cfg.sim_config.dt * 60))  # 1000Hz / 60Hz ≈ 16次step渲染1次
        render_count = 0  # 渲染计数器

        total_steps = int(cfg.sim_config.sim_duration / cfg.sim_config.dt)
        print(f"Simulating {cfg.sim_config.sim_duration}s, total steps: {total_steps}, render interval: {render_interval}")

        print("键盘控制：wasd")

        # C++ UDP客户端地址
        cpp_client_addr = None

        for step in range(total_steps):
            try:
                data_udp, addr = udp_socket.recvfrom(1024) # 接收UDP数据
                cpp_client_addr = addr  # 保存C++客户端地址
                response_msg = decode_msg_response(data_udp) # 解码响应消息
                if response_msg:
                    action_rl = response_msg.q_exp.copy()
                    # print(f"收到动作: {action_rl}")
            except socket.timeout:
                # 继续仿真
                pass
            except Exception as e:
                # print(f"接收UDP数据错误: {e}")
                pass

            # 更新步态
            key_gait = check_keyboard()
            if key_gait is not None:
                if key_gait == 's':
                    print("rl-tort：踏步")
                    current_gait = 1
                    cmd.vx = 0.0
                    cmd.vy = 0.0
                    cmd.dyaw = 0.0
                if key_gait == 'w':
                    print("rl-tort：前进")
                    current_gait = 2
                    cmd.vx = 0.3
                    cmd.vy = 0.0
                    cmd.dyaw = 0.0
                elif key_gait == 'a':
                    print("rl-tort：左转")
                    current_gait = 3
                    cmd.vx = 0.1
                    cmd.vy = 0.0
                    cmd.dyaw = 0.3
                elif key_gait == 'd':
                    print("rl-tort：右转")
                    current_gait = 4
                    cmd.vx = 0.1
                    cmd.vy = 0.0
                    cmd.dyaw = -0.3
                elif key_gait == 'x':
                    print("rl-tort：后退")
                    current_gait = 5
                    cmd.vx = -0.3
                    cmd.vy = 0.0
                    cmd.dyaw = 0

            # Obtain an observation
            q, dq, quat, v, omega, gvec = get_obs(data)#从mujoco获取仿真数据
            q = q[-cfg.env.num_actions:]
            dq = dq[-cfg.env.num_actions:]

            if count_lowlevel % cfg.sim_config.decimation == 0:
                # 只在50Hz时发送观测数据
                eu_ang = quaternion_to_euler_array(quat)
                eu_ang[eu_ang > math.pi] -= 2 * math.pi

                # 创建并发送请求消息
                request_msg = MsgRequest()
                request_msg.omega = omega
                request_msg.eu_ang = eu_ang
                request_msg.command = [cmd.vx, cmd.vy, cmd.dyaw]
                request_msg.q = q
                request_msg.dq = dq
                request_msg.gait_type = [current_gait]

                # 编码并发送观测UDP消息
                tx_data = encode_msg_request(request_msg)
                
                if cpp_client_addr:  # 只有知道C++地址时才发送
                    try:
                        udp_socket.sendto(tx_data, cpp_client_addr)
                        # print(f"发送观测数据到 {cpp_client_addr}")
                    except Exception as e:
                        print(f"发送UDP数据错误: {e}")
                else:
                    # 如果还没有收到C++的连接，发送到默认地址
                    try:
                        udp_socket.sendto(tx_data, ('127.0.0.1', 10000))
                    except:
                        pass

                # 处理RL动作
                action = np.clip(action_rl, -cfg.normalization.clip_actions, cfg.normalization.clip_actions)
                target_q = action * cfg.control.action_scale + default_dof_pos
            
            # PD控制计算力矩
            target_dq = np.zeros((cfg.env.num_actions), dtype=np.double)
            # Generate PD control
            tau = pd_control(target_q, q, cfg.robot_config.kps,
                                target_dq, dq, cfg.robot_config.kds)  # Calc torques
            tau = np.clip(tau, -cfg.robot_config.tau_limit, cfg.robot_config.tau_limit)  # Clamp torques
            data.ctrl = tau

            render_count += 1
            if render_count >= render_interval:
                viewer.render()
                render_count = 0

            mujoco.mj_step(model, data)
            count_lowlevel += 1

        viewer.close()
        udp_socket.close()

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Deployment script.')
    parser.add_argument('--terrain', action='store_true', default=False)
    args = parser.parse_args()
    
    class Sim2simCfg(TinkerConstraintHimRoughCfg):

        class sim_config:
            if args.terrain:
                mujoco_model_path = f'{ROOT_DIR}/resources/tinker/xml/world_terrain.xml' # tinkerv1
                # mujoco_model_path = f'{ROOT_DIR}/resources/TinkerV2_URDF/xml/world_terrain.xml' # tinkerv2
            else:
                mujoco_model_path = f'{ROOT_DIR}/resources/tinker/xml/world.xml' # tinkerv1
                # mujoco_model_path = f'{ROOT_DIR}/resources/TinkerV2_URDF/xml/world.xml' # tinkerv2
            sim_duration = 600.0
            dt = 0.001 #1Khz底层
            decimation = 20 # 50Hz

        class robot_config:
            # kp_all = 15.0 # 10.0
            # kd_all = 0.5 # 0.4
            # kps = np.array([kp_all, kp_all, kp_all, kp_all, kp_all, kp_all, kp_all, kp_all, kp_all, kp_all], dtype=np.double)#PD和isacc内部一致
            # kds = np.array([kd_all, kd_all, kd_all, kd_all, kd_all, kd_all, kd_all, kd_all, kd_all, kd_all], dtype=np.double)
            kps = np.array([13, 15, 15, 15, 13, 13, 15, 15, 15, 13], dtype=np.double)
            kds = np.array([0.3, 0.65, 0.65, 0.65, 0.3, 0.3, 0.65, 0.65, 0.65, 0.3], dtype=np.double)
            tau_limit = 20. * np.ones(10, dtype=np.double)#nm
    
    run_mujoco(Sim2simCfg())