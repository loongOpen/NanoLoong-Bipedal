#include "rl_gait_switch_lcm.h"
#include <iostream>
#include <valarray>
#include <iomanip>

using namespace std;
using namespace chrono;

void RL_Tinker::handleMessage(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                       const  my_lcm::Request *request)//获取机器人反馈
{       
    if (!rl_gait) {
        if (request->gait_type[0] !=0){
            rl_gait = true;
        }else
            return;
    }

    // RL控制
    if (rl_gait) {
        if (request->gait_type[0] == 4){
            // std::cout << "切换到 rl-stand" << std::endl;
            onnx_model = &onnx_model_stand;
        }else {
            // std::cout << "切换到 rl-tort" << std::endl;
            onnx_model = &onnx_model_tort;
        }
    }

    std::vector<float> obs;
    //---------------Push data into obsbuf--------------------
    obs.push_back(request->omega[0]*omega_scale);
    obs.push_back(request->omega[1]*omega_scale);
    obs.push_back(request->omega[2]*omega_scale);

    obs.push_back(request->eu_ang[0]*eu_ang_scale);
    obs.push_back(request->eu_ang[1]*eu_ang_scale);
    obs.push_back(request->eu_ang[2]*eu_ang_scale);

    // cmd
    // float max = 1.0;
    // float min = -1.0;

    cmd_x = cmd_x * (1 - smooth) + (std::fabs(request->command[0]) < dead_zone ? 0.0 : request->command[0]) * smooth;
    cmd_y = cmd_y * (1 - smooth) + (std::fabs(request->command[1]) < dead_zone ? 0.0 : request->command[1]) * smooth;
    cmd_rate = cmd_rate * (1 - smooth) + (std::fabs(request->command[2]) < dead_zone ? 0.0 : request->command[2]) * smooth;

    obs.push_back(cmd_x*lin_vel);//控制指令x
    obs.push_back(cmd_y*lin_vel);//控制指令y
    obs.push_back(cmd_rate*ang_vel);//控制指令yaw rate

    
    // pos q joint
    for (int i = 0; i < 10; ++i)
    {
        float pos = (request->q[i]  - init_pos[i])* pos_scale;
        obs.push_back(pos);
    }
    // vel q joint
    for (int i = 0; i < 10; ++i)
    {
        float vel = request->dq[i] * vel_scale;
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

    // 运行模型推理
    onnx_model->run();

    // 获取输出
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

    action_refresh=1;

    // ---------- 接收频率统计 ----------
    obs_count++;
    auto now = chrono::steady_clock::now();
    double elapsed = chrono::duration<double>(now - last_print).count();

    if (elapsed >= 1.0) { // 每秒打印一次
        double freq = obs_count / elapsed;
        // cout << "LCM_OBS 接收频率: " << fixed << setprecision(2) << freq << " Hz" << endl;
        obs_count = 0;
        last_print = now;
    }
}

int RL_Tinker::load_policy()
{   
    // model_path = "/home/lcj/tinker_dev/Tinker-k1/Model/Tort/model.onnx";
    // std::cout << "加载模型: " << model_path << std::endl; 
    printf("开始初始化ONNX_TROT模型...\n");
    bool success1 = onnx_model_tort.init("/home/bianbu/bipedal-robot-sim/riscv/bipedal-sim-lcm/Model/Trot/model.onnx", 39, 390, 10);
    
    // model_path = "/home/lcj/tinker_dev/Tinker-k1/Model/Stand/model.onnx";
    printf("开始初始化ONNX_STAND模型...\n");
    bool success2 = onnx_model_stand.init("/home/bianbu/bipedal-robot-sim/riscv/bipedal-sim-lcm/Model/Stand/model.onnx", 39, 390, 10);
    
    if (success1 && success2) {
        std::cout << "ONNX模型加载成功!" << std::endl;
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
            obs.push_back(0);
        }
    }
    
    printf("初始化完成\n");
    return 0;
}

int main(int argc, char** argv) {
    lcm::LCM lc("udpm://239.255.76.67:8888?ttl=0");
    if (!lc.good()) {
        std::cerr << "LCM initialization failed" << std::endl;
        return 1;
    }
    
    RL_Tinker tinker_rl;

    tinker_rl.init_policy();
    lc.subscribe("LCM_OBS", &RL_Tinker::handleMessage, &tinker_rl);

    while (1){
        // 发布推理结果
        my_lcm::Response msg;
        for(int i = 0; i < 10; i++){
            msg.q_exp[i] = tinker_rl.action[i];
        }
        lc.publish("LCM_ACTION", &msg);
        lc.handle();
    }

    return 0;
}