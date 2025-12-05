#include "rl_tinkerv1_udp.h"
#include <iostream>
#include <valarray>
#include <iomanip>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <pthread.h>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/time.h>
#include <math.h>
#include <sys/shm.h>
#include <arpa/inet.h>
#include <time.h>
#include <cstring>

using namespace std;
using namespace chrono;

//RL
_msg_request msg_request;
_msg_response msg_response;
float limit(float input,float min,float max){
    if(input>max)
        return max;
    if(input<min)
        return min;
    return input;
} 

void RL_Tinker::handleMessage(_msg_request request)
{    
    if (!rl_gait) {
        if (request.gait_type[0] !=0){
            rl_gait = true;
        }else
            return;
    }

    // RL控制
    if (rl_gait) {
        if (request.gait_type[0]){
            onnx_model = &onnx_model_tort;
        }
    }

    // 检查模型指针
    if (onnx_model == nullptr) {
        std::cerr << "错误: onnx_model 指针为空!" << std::endl;
        return;
    }

    std::vector<float> obs;
    //---------------Push data into obsbuf--------------------
    obs.push_back(request.omega[0]*omega_scale);
    obs.push_back(request.omega[1]*omega_scale);
    obs.push_back(request.omega[2]*omega_scale);

    obs.push_back(request.eu_ang[0]*eu_ang_scale);
    obs.push_back(request.eu_ang[1]*eu_ang_scale);
    obs.push_back(request.eu_ang[2]*eu_ang_scale);

    // cmd
    // float max = 1.0;
    // float min = -1.0;

    cmd_x = cmd_x * (1 - smooth) + (std::fabs(request.command[0]) < dead_zone ? 0.0 : request.command[0]) * smooth;
    cmd_y = cmd_y * (1 - smooth) + (std::fabs(request.command[1]) < dead_zone ? 0.0 : request.command[1]) * smooth;
    cmd_rate = cmd_rate * (1 - smooth) + (std::fabs(request.command[2]) < dead_zone ? 0.0 : request.command[2]) * smooth;

    obs.push_back(cmd_x*lin_vel);//控制指令x
    obs.push_back(cmd_y*lin_vel);//控制指令y
    obs.push_back(cmd_rate*ang_vel);//控制指令yaw rate

    
    // pos q joint
    for (int i = 0; i < 10; ++i)
    {
        float pos = (request.q[i]  - init_pos[i])* pos_scale;
        obs.push_back(pos);
    }
    // vel q joint
    for (int i = 0; i < 10; ++i)
    {
        float vel = request.dq[i] * vel_scale;
        obs.push_back(vel);
    }
    // last action
    for (int i = 0; i < 10; ++i)
    {
        obs.push_back(action_temp[i]);// 
    }
    
    // // normalization:clip_observations = 100. clip_actions = 100 #控制限制
    // float clip_obs = 100.0f;
    // for (int i = 0; i < 39; i++) {
    //     if (obs[i] > clip_obs) obs[i] = clip_obs;
    //     if (obs[i] < -clip_obs) obs[i] = -clip_obs;
    // }

    // 第一个输入: obs [1, 39] - 当前观测
    for (int i = 0; i < 39; ++i) {
        onnx_model->in1[i] = obs[i];
    }
    
    // 第二个输入: obs_buf [1, 10, 39] - 历史观测
    if (obs_buf.size() == 390) {
        for (int i = 0; i < 390; ++i) {
            onnx_model->in2[i] = obs_buf[i];
        }
    } else {
        std::cerr << "错误: obs_buf大小不正确，期望390，实际" << obs_buf.size() << std::endl;
        return;
    }

    // 运行模型推理 - 使用我们的接口
    onnx_model->run();

    // 获取输出 - 输出应该是 [1, 10] 或 [10] 形状
    std::vector<float> action_vec(10);
    for (int i = 0; i < 10; ++i) {
        action_vec[i] = onnx_model->out[i];
    }

    // 更新action_buf (移位操作)
    if (action_buf.size() == history_length * 10) {
        // 移除最旧的动作（第一行）
        action_buf.erase(action_buf.begin(), action_buf.begin() + 10);
        // 添加新动作到末尾
        action_buf.insert(action_buf.end(), action_vec.begin(), action_vec.end());
    }

    // 检查NaN
    bool has_nan = false;
    for (float val : obs) {
        if (std::isnan(val)) {
            has_nan = true;
            break;
        }
    }
    if (has_nan) {
        cout << "NaN detected in obs. Press any key to continue..." << endl;
        getchar(); // 等待键盘输入
    }

    // 网络输出滤波
    std::vector<float> action_blend(10); 
    for (int i = 0; i < 10; i++) { 
        action_blend[i] = 0.8f * action_vec[i] + 0.2f * last_action[i];
    }

    last_action = action_vec; // 更新last_action
    
    // 更新obs_buf (移位操作)
    if (obs_buf.size() == history_length * 39) {
        // 移除最旧的观测（第一行）
        obs_buf.erase(obs_buf.begin(), obs_buf.begin() + 39);
        // 添加新观测到末尾
        obs_buf.insert(obs_buf.end(), obs.begin(), obs.end());
    }

    // 输出动作
    for (int j = 0; j < 10; j++) { 
        action[j] = action_blend[j]; 
        action_temp[j] = action_blend[j]; // 原始值
    }


    // action_refresh = 1;

    // ---------- 接收频率统计 ----------
    obs_count++;
    auto now = chrono::steady_clock::now();
    double elapsed = chrono::duration<double>(now - last_print).count();

    if (elapsed >= 1.0) {
        double freq = obs_count / elapsed;
        cout << "UDP接收频率: " << fixed << setprecision(2) << freq << " Hz" << endl;
        obs_count = 0;
        last_print = now;
    }
}

int RL_Tinker::load_policy()
{   
    printf("开始初始化ONNX_TROT模型...\n");
    bool success = onnx_model_tort.init("/home/bianbu/bipedal-robot-sim/riscv/bipedal-sim-udp/Model/Trot/modelv1.onnx", 39, 390, 10);
    
    if (success) {
        std::cout << "ONNX模型加载成功!" << std::endl;
        // 设置默认模型
        onnx_model = &onnx_model_tort;
        return 0;
    } else {
        std::cerr << "ONNX模型加载失败!" << std::endl;
        return -1;
    }
}

int RL_Tinker::init_policy(){
    std::cout << "RL模型线程启动" << endl;

    printf("调用load_policy...\n");
    if (load_policy() != 0) {
        return -1;
    }

    // initialize record
    action_buf.resize(history_length * 10, 0.0f); // 形状: [history_length, 10]
    obs_buf.resize(history_length * 39, 0.0f); // 形状: [history_length, 39]
    last_action.resize(10, 0.0f); // 形状: [10]

    // 清空并重新初始化
    action_temp.clear();
    prev_action.clear();

    for (int j = 0; j < 10; j++)
    {
        action_temp.push_back(0.0);
	    action.push_back(init_pos[j]);
        prev_action.push_back(init_pos[j]);
    }

    //hot start - 初始化历史观测
    for (int i = 0; i < history_length; i++)
    {
        std::vector<float> obs;
        //---------------Push data into obsbuf--------------------
        obs.push_back(0);
        obs.push_back(0);
        obs.push_back(0);

        obs.push_back(0);
        obs.push_back(0);
        obs.push_back(0);

        // cmd
        obs.push_back(0);//控制指令x
        obs.push_back(0);//控制指令y
        obs.push_back(0);//控制指令yaw rate

        
        // pos q joint
        for (int j = 0; j < 10; ++j)
        {
            float pos = 0;
            obs.push_back(pos);
            action[i]=init_pos[i];
        }
        // vel q joint
        for (int j = 0; j < 10; ++j)
        {
            float vel = 0;
            obs.push_back(vel);
        }
        // last action
        for (int j = 0; j < 10; ++j)
        {
            obs.push_back(0);//历史
        }
    }
    
    printf("初始化完成\n");
    return 0;
}

int main(int argc, char** argv) {
    int sock_fd;
    sock_fd = socket(AF_INET, SOCK_DGRAM, 0); //创建UDP socket
    if(sock_fd < 0)
    {
        exit(1);
    }

    struct sockaddr_in addr_serv;
    int len;
    memset(&addr_serv, 0, sizeof(addr_serv)); // 清零地址结构
    addr_serv.sin_family = AF_INET; // 设置为IPv4地址

#if 0
    string UDP_IP="127.0.0.1";// local test
    int SERV_PORT= 8888 ;// 
#else
    string UDP_IP="10.0.90.6";// 
    int SERV_PORT= 8848 ;// 
#endif

    addr_serv.sin_addr.s_addr = inet_addr(UDP_IP.c_str()); // 目标IP
    addr_serv.sin_port = htons(SERV_PORT); // 目标端口
    len = sizeof(addr_serv);

    int recv_num=0,send_num=0;
    // int connect=0,loss_cnt=0;
    char send_buf[500]={0},recv_buf[500]={0}; // 发送和接收缓冲区

    RL_Tinker tinker_rl;
    
    // 初始化RL策略
    tinker_rl.init_policy();
    
    // 初始化响应消息
    for(int i=0;i<10;i++){
        msg_response.q_exp[i]=tinker_rl.action[i];
    }

    printf("UDP RL-Tinker 启动\n");
    // int cnt_p=0;

    // 设置socket超时
    struct timeval timeout;
    // timeout.tv_sec = 1;
    // timeout.tv_usec = 0;
    timeout.tv_sec = 0;
    timeout.tv_usec = 1000;
    setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    // 主通信循环
    while (1)
    {
        //send action
        for(int i=0;i<10;i++){
            msg_response.q_exp[i]=tinker_rl.action[i];
        }
        // std::cout.precision(2);

        // 调试输出
        #if 0
        if (cnt_p % 50 == 0) {  // 每50次打印一次
            cout << "act send: ";
            for(int i = 0; i < 10; i++) {
                cout << msg_response.q_exp[i] << " ";
            }
            cout << endl;
        }
        #endif
        // cnt_p++;

        // 编码
        memcpy(send_buf, &msg_response, sizeof(msg_response));
        send_num = sendto(sock_fd, send_buf, sizeof(msg_response), 0, 
                         (struct sockaddr *)&addr_serv, len);
 
        if(send_num < 0) {
            perror("sendto error");
            continue;
        }
        
        // UDP获取mujoco观测
        recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), 0, 
                           (struct sockaddr *)&addr_serv, (socklen_t *)&len);
        if(recv_num > 0) {
            // 解码
            memcpy(&msg_request, recv_buf, sizeof(msg_request));
            tinker_rl.handleMessage(msg_request);
        } else {
            // perror("recvfrom error or timeout");
        }
        
        usleep(1 * 1000);  // 2ms
    }
    
    close(sock_fd);
    return 0;
}