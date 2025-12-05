#include <iostream>
#include <mutex>
#include <thread>
#include <vector>
#include <memory>
#include <random>

#include "enumClass.h"
#include "mathTools.h"
#include "mathTypes.h" 
#include "stdio.h"
#include <chrono>

// DDS
#include "Request/Request.h"
#include "Request/RequestPubSubTypes.h"
#include "Response/Response.h"
#include "Response/ResponsePubSubTypes.h"

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/topic/Topic.hpp>

#include "rl_onnx.h"

using namespace eprosima::fastdds::dds;
using namespace std;
using namespace chrono;

class RL_Tinker
{ 
public:
    RL_Tinker();
    ~RL_Tinker();

    std::string model_path;

    bool rl_gait = false;
    std::chrono::steady_clock::time_point dds_start_time;
    bool dds_stable = false;
    
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
    int action_refresh=0;
    int history_length = 10;
    float init_pos[10] = {0.0,0.08,0.56,-1.12,-0.57,  0.0,-0.08,-0.56,1.12,0.57};
    float eu_ang_scale= 1;
    float omega_scale=  0.25;
    float pos_scale =   1.0;
    float vel_scale =   0.05;
    float lin_vel = 2.0;
    float ang_vel = 0.25;

    int obs_count = 0;
    std::chrono::steady_clock::time_point last_print = std::chrono::steady_clock::now();

    class RequestListener : public DataReaderListener
    {
    public:
        // RequestListener() : outer_(nullptr) {}
        RequestListener(RL_Tinker* outer) : outer_(outer) {}
        void on_data_available(DataReader* reader) override;
    private:
        RL_Tinker* outer_; 
    };

    // DDS init
    // domain
    DomainParticipant* participant;
    // sub
    Subscriber* subscriber;
    Topic* request_topic;
    DataReader* request_reader;
    RequestListener request_listener;
    // pub
    Publisher* publisher;
    Topic* response_topic;
    DataWriter* response_writer;

    OnnxRuntime::OnnxRuntimeClass onnx_model_tort;
    OnnxRuntime::OnnxRuntimeClass onnx_model_stand;
    OnnxRuntime::OnnxRuntimeClass* onnx_model;
};