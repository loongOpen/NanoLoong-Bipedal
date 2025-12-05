#include <iostream>
#include <mutex>
#include <thread>
#include <vector>
#include <memory>

#include <lcm/lcm-cpp.hpp>
#include "Request.hpp"
#include "Response.hpp"

#include "enumClass.h"
#include "mathTools.h"
#include "mathTypes.h" 
#include "stdio.h"
#include <chrono>
#include <iostream>

#include "rl_onnx.h"

struct _msg_request
{
    float omega[3];
    float eu_ang[3];
    float command[3];
    float q[10];
    float dq[10];
    int gait_type[1];
    // float acc[3];
    // float tau[10];
    // float init_pos[10];
};

struct _msg_response
{
    float q_exp[10];
    // float dq_exp[10];
    // float tau_exp[10];
};


class RL_Tinker {
public:
    std::string model_path;
    int init_policy();
    int load_policy();
    void handleMessage(_msg_request request);

    bool rl_gait = false;

    //gamepad
    float smooth = 0.03;
    float dead_zone = 0.01;

    float cmd_x = 0.;
    float cmd_y = 0.;
    float cmd_rate = 0.;

    std::vector<float> action;
    std::vector<float> action_temp;
    std::vector<float> prev_action;

    std::vector<float> action_buf;
    std::vector<float> obs_buf;
    std::vector<float> last_action;
    
    // default values
    // int action_refresh=0;
    int history_length = 10;
    float init_pos[10] = {0.0,-0.0,0.56,-1.12,0.57,  0.0,0.0,0.56,-1.12,0.57}; //v1 no head
    // float init_pos[10] = {0.0,0.08,0.56,-1.12,-0.57,  0.0,-0.08,-0.56,1.12,0.57};
    float eu_ang_scale= 1;
    float omega_scale=  0.25;
    float pos_scale =   1.0;
    float vel_scale =   0.05;
    float lin_vel = 2.0;
    float ang_vel = 0.25;
    // float action_scale[10] = {0.25,0.25,0.25,0.25,0.25, 0.25,0.25,0.25,0.25,0.25};

    int obs_count = 0;
    std::chrono::steady_clock::time_point last_print = std::chrono::steady_clock::now();

private:
    OnnxRuntime::OnnxRuntimeClass onnx_model_tort;
    OnnxRuntime::OnnxRuntimeClass onnx_model_stand;
    OnnxRuntime::OnnxRuntimeClass* onnx_model;
};