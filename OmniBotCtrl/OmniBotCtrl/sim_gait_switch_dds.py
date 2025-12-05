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
import numpy as np
import time
import threading
import select
import termios
import tty

sys.path.append(os.path.join(os.path.dirname(__file__), "sim2sim_dds","Request", "build"))
sys.path.append(os.path.join(os.path.dirname(__file__), "sim2sim_dds","Response", "build"))
import fastdds
import Request
import Response

last_print_time = time.time()

default_dof_pos = [0.0,0.08,0.56,-1.12,-0.57,  0.0,-0.08,-0.56,1.12,0.57]
action_rl=default_dof_pos
current_gait = 0

class ResponseListener(fastdds.DataReaderListener):
    def __init__(self):
        super().__init__()
    def on_data_available(self, reader):
        global action_rl, last_print_time
        info = fastdds.SampleInfo()
        response = Response.Response()
        reader.take_next_sample(response, info)
        action_rl = list(response.q_exp())

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
    quat = data.sensor('orientation').data[[1, 2, 3, 0]].astype(np.double)
    r = R.from_quat(quat)
    v = r.apply(data.qvel[:3], inverse=True).astype(np.double)  # In the base frame
    omega = data.sensor('angular-velocity').data.astype(np.double)
    gvec = r.apply(np.array([0., 0., -1.]), inverse=True).astype(np.double)
    
    return (q, dq, quat, v, omega, gvec)

def pd_control(target_q, q, kp, target_dq, dq, kd):
    '''Calculates torques from position commands
    '''
    return (target_q - q) * kp + (target_dq - dq) * kd

def _low_pass_action_filter(actions,last_actions):
    flt =0.1
    actons_filtered = last_actions * flt + actions * (1-flt)
    return actons_filtered
    
def check_keyboard():
    """检查键盘输入，非阻塞方式"""
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        key = sys.stdin.read(1)
        return key # w a s d
    return None

def run_mujoco(cfg):
    global action_rl,default_dof_pos,current_gait
    """
    通过DDS作为通用接口传输传感器原始数据，接受网络直接输出
    """
    # DDS config
    # domain
    participant_qos = fastdds.DomainParticipantQos()
    participant = fastdds.DomainParticipantFactory.get_instance().create_participant(0, participant_qos)

    #sub
    response_type = fastdds.TypeSupport(Response.ResponsePubSubType())
    participant.register_type(response_type, "Response")
    subscriber = participant.create_subscriber(fastdds.SUBSCRIBER_QOS_DEFAULT)
    response_topic = participant.create_topic("ResponseTopic", "Response", fastdds.TOPIC_QOS_DEFAULT)
    reader_qos = fastdds.DataReaderQos()
    reader_qos.reliability().kind = fastdds.BEST_EFFORT_RELIABILITY_QOS
    listener = ResponseListener()
    response_reader = subscriber.create_datareader(response_topic, reader_qos, listener)

    # pub
    request_type = fastdds.TypeSupport(Request.RequestPubSubType())
    participant.register_type(request_type, "Request")
    publisher = participant.create_publisher(fastdds.PUBLISHER_QOS_DEFAULT)
    request_topic = participant.create_topic("RequestTopic", "Request", fastdds.TOPIC_QOS_DEFAULT)
    writer_qos = fastdds.DataWriterQos()
    writer_qos.reliability().kind = fastdds.RELIABLE_RELIABILITY_QOS
    request_writer = publisher.create_datawriter(request_topic, writer_qos)

    # 非阻塞输入
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())

        model = mujoco.MjModel.from_xml_path(cfg.sim_config.mujoco_model_path)
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
        render_count = 0  # 渲染计数

        total_steps = int(cfg.sim_config.sim_duration / cfg.sim_config.dt)
        print(f"Simulating {cfg.sim_config.sim_duration}s, total steps: {total_steps}, render interval: {render_interval}")

        print("键盘控制：Q:踏步 w:前进 a：左转 d：右转 s:站立")

        for step in range(total_steps):
            # 更新步态
            key_gait = check_keyboard()
            if key_gait is not None:
                if key_gait == 'q':
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
                elif key_gait == 's':
                    print("rl-stand：站立")
                    current_gait = 6
                    cmd.vx = 0.0
                    cmd.vy = 0.0
                    cmd.dyaw = 0.0

            # Obtain an observation
            q, dq, quat, v, omega, gvec = get_obs(data)#从mujoco获取仿真数据
            q = q[-cfg.env.num_actions:]
            dq = dq[-cfg.env.num_actions:]

            if count_lowlevel % cfg.sim_config.decimation == 0:
                obs = np.zeros([1, cfg.env.n_proprio], dtype=np.float32) #1,39

                eu_ang = quaternion_to_euler_array(quat)
                eu_ang[eu_ang > math.pi] -= 2 * math.pi

                # Request publish
                request = Request.Request()
                request.omega(omega)
                request.eu_ang(eu_ang)
                cmd_list = [cmd.vx, cmd.vy, cmd.dyaw]
                request.command(cmd_list)
                request.q(q)
                request.dq(dq)
                request.gait_type([current_gait])

                request_writer.write(request)

                #--DDS传输RL滤波后的输出 
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

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    
    participant.delete_contained_entities()
    fastdds.DomainParticipantFactory.get_instance().delete_participant(participant)

def main():
    import argparse

    parser = argparse.ArgumentParser(description='Deployment script.')
    parser.add_argument('--terrain', action='store_true', default=False)
    args = parser.parse_args()
    
    class Sim2simCfg(TinkerConstraintHimRoughCfg):

        class sim_config:
            if args.terrain:
                mujoco_model_path = f'{ROOT_DIR}/resources/TinkerV2_URDF/xml/world_terrain.xml'
            else:
                mujoco_model_path = f'{ROOT_DIR}/resources/TinkerV2_URDF/xml/world.xml'
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

if __name__ == "__main__":
    main()
