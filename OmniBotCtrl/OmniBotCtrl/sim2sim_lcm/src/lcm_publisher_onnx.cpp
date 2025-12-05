#include "lcm_publish_onnx.h"
#include <iostream>
#include <valarray>

#include <iomanip>

using namespace std;
using namespace chrono;

static Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "RL_Model");
std::unique_ptr<Ort::Session> session;
static Ort::SessionOptions session_options;

RL_Tinker tinker_rl;

void RL_Tinker::handleMessage(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                       const  my_lcm::Request *request)//获取机器人反馈
{           
    // auto handle_start = high_resolution_clock::now();
    std::vector<float> obs;
    //---------------Push data into obsbuf--------------------
    obs.push_back(request->omega[0]*omega_scale);
    obs.push_back(request->omega[1]*omega_scale);
    obs.push_back(request->omega[2]*omega_scale);

    obs.push_back(request->eu_ang[0]*eu_ang_scale);
    obs.push_back(request->eu_ang[1]*eu_ang_scale);
    obs.push_back(request->eu_ang[2]*eu_ang_scale);

    // cmd
    float max = 1.0;
    float min = -1.0;

    cmd_x = cmd_x * (1 - smooth) + (std::fabs(request->command[0]) < dead_zone ? 0.0 : request->command[0]) * smooth;
    cmd_y = cmd_y * (1 - smooth) + (std::fabs(request->command[1]) < dead_zone ? 0.0 : request->command[1]) * smooth;
    cmd_rate = cmd_rate * (1 - smooth) + (std::fabs(request->command[2]) < dead_zone ? 0.0 : request->command[2]) * smooth;

    obs.push_back(cmd_x*lin_vel);
    obs.push_back(cmd_y*lin_vel);
    obs.push_back(cmd_rate*ang_vel);

    
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

    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    
    // obs [1, 39]
    std::vector<int64_t> obs_shape = {1, 39};
    Ort::Value input_obs = Ort::Value::CreateTensor<float>(
        memory_info, obs.data(), obs.size(), obs_shape.data(), obs_shape.size());
    
    // obs_hist [1, history_length, 39]
    std::vector<int64_t> obs_buf_shape  = {1, static_cast<int64_t>(history_length), 39};
    
    Ort::Value input_obs_buf = Ort::Value::CreateTensor<float>(
        memory_info, obs_buf.data(), obs_buf.size(), obs_buf_shape.data(), obs_buf_shape.size());
    
    std::vector<const char*> input_names = {"obs", "obs_hist"};
    std::vector<const char*> output_names = {"action_tensor"};
    std::vector<Ort::Value> inputs;
    inputs.push_back(std::move(input_obs));
    inputs.push_back(std::move(input_obs_buf ));
    
    auto outputs = session->Run(Ort::RunOptions{nullptr}, 
                               input_names.data(), 
                               inputs.data(), 
                               inputs.size(), 
                               output_names.data(), 
                               output_names.size());
    
    float* output_data = outputs[0].GetTensorMutableData<float>();
    std::vector<float> action_vec(output_data, output_data + 10); 
    
    // 更新action_buf (移位操作)
    if (action_buf.size() == history_length * 10) {
        // 移除最旧的动作（第一行）
        action_buf.erase(action_buf.begin(), action_buf.begin() + 10);
        // 添加新动作到末尾
        action_buf.insert(action_buf.end(), action_vec.begin(), action_vec.end());
    }

    bool has_nan = false;
    for (float val : obs) {
        if (std::isnan(val)) {
            has_nan = true;
            break;
        }
    }
    if (has_nan) {
        cout << "NaN detected in obs. Press any key to continue..." << endl;
        getchar();
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

    // ---------- 接收频率统计 ----------
    obs_count++;
    auto now = chrono::steady_clock::now();
    double elapsed = chrono::duration<double>(now - last_print).count();

    if (elapsed >= 1.0) { // 每秒打印一次
        double freq = obs_count / elapsed;
        cout << "LCM_OBS 接收频率: " << fixed << setprecision(2) << freq << " Hz" << endl;
        obs_count = 0;
        last_print = now;
    }

    // auto handle_end = high_resolution_clock::now();
    // double handle_time = duration_cast<microseconds>(handle_end - handle_start).count() / 1000.0;
    // // cout << "Handle time: " << handle_time << " ms" << endl;
    // cout << "RL频率: " << fixed << setprecision(2) << 1000.0 / handle_time << " Hz" << endl;
}


int RL_Tinker::load_policy()
{   
    std::cout << model_path << std::endl;
    // 加载ONNX模型
    try {
        session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        session = std::make_unique<Ort::Session>(env, model_path.c_str(), session_options);
        std::cout << "ONNX model loaded successfully!" << std::endl;
        return 0;
    } catch (const Ort::Exception& e) {
        std::cerr << "Error loading model: " << e.what() << std::endl;
        return -1;
    }
}
 
int RL_Tinker::init_policy(){
 // load policy
    std::cout << "RL model thread start"<<endl;
    model_path = "/home/bianbu/sim2sim_lcm/model.onnx"; 
    if (load_policy() != 0) {
        return -1;
    }

 // initialize record
    action_buf.resize(history_length * 10, 0.0f);
    obs_buf.resize(history_length * 39, 0.0f);
    last_action.resize(10, 0.0f);

    for (int j = 0; j < 10; j++)
    {
        action_temp.push_back(0.0);
	    action.push_back(init_pos[j]);
        prev_action.push_back(init_pos[j]);
    }
    //hot start
    for (int i = 0; i < history_length; i++)
    {
        std::vector<float> obs;
        //---------------Push data into obsbuf--------------------
        obs.push_back(0);//request->omega[0]*omega_scale);
        obs.push_back(0);//request->omega[1]*omega_scale);
        obs.push_back(0);//request->omega[2]*omega_scale);

        obs.push_back(0);//request->eu_ang[0]*eu_ang_scale);
        obs.push_back(0);//request->eu_ang[1]*eu_ang_scale);
        obs.push_back(0);//request->eu_ang[2]*eu_ang_scale);

        // cmd
        obs.push_back(0);
        obs.push_back(0);
        obs.push_back(0);

        
        // pos q joint
        for (int i = 0; i < 10; ++i)
        {
            float pos = 0;//(request->q[i]  - init_pos[i])* pos_scale;
            obs.push_back(pos);
            action[i]=init_pos[i];
        }
        // vel q joint
        for (int i = 0; i < 10; ++i)
        {
            float vel = 0;//request->dq[i] * vel_scale;
            obs.push_back(vel);
        }
        // last action
        for (int i = 0; i < 10; ++i)
        {
            obs.push_back(0);
        }
    }
    return 0;
}

int main(int argc, char** argv) {
    // auto start = high_resolution_clock::now();
    // int pub_count = 0;  // 统计处理的消息总数

    // lcm::LCM lcm;
    lcm::LCM lc("udpm://239.255.76.67:8888?ttl=1");
    if (!lc.good()) {
        std::cerr << "LCM initialization failed" << std::endl;
        return 1;
    }
    tinker_rl.init_policy();
 
    lc.subscribe("LCM_OBS", &RL_Tinker::handleMessage, &tinker_rl);
    while (1){
        my_lcm::Response msg;
        for(int i = 0; i < 10; i++) //修改12->10
            msg.q_exp[i] = tinker_rl.action[i];
        
        lc.publish("LCM_ACTION", &msg);
        lc.handle();

        // pub_count++;
        // if (pub_count % 10 == 0) {
        //     auto end = high_resolution_clock::now();
        //     double elapsed = duration<double>(end - start).count();
        //     double freq = pub_count / elapsed;
        //     cout << "LCM_ACTION发布频率: " << fixed << setprecision(2) << freq << " Hz" << endl;
        // }
    }
 
    return 0;
}

