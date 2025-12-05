#include "rl_gait_switch_dds.h"
#include <iostream>
#include <valarray>
#include <iomanip>
#include <unistd.h>
#include <sys/time.h>

RL_Tinker::RL_Tinker() : request_listener(this) 
{
    // DDS config
    // domain
    DomainParticipantQos participant_qos;
    participant = DomainParticipantFactory::get_instance()->create_participant(0, participant_qos);

    // sub
    TypeSupport request_type(new RequestPubSubType());
    participant->register_type(request_type, "Request");
    subscriber = participant->create_subscriber(SUBSCRIBER_QOS_DEFAULT);
    request_topic = participant->create_topic("RequestTopic", "Request", TOPIC_QOS_DEFAULT);
    DataReaderQos reader_qos;
    reader_qos.reliability().kind = BEST_EFFORT_RELIABILITY_QOS;
    request_reader = subscriber->create_datareader(request_topic, reader_qos, &request_listener);

    // pub
    TypeSupport response_type(new ResponsePubSubType());
    participant->register_type(response_type, "Response");
    publisher = participant->create_publisher(PUBLISHER_QOS_DEFAULT);
    response_topic = participant->create_topic("ResponseTopic", "Response", TOPIC_QOS_DEFAULT);
    DataWriterQos writer_qos;
    writer_qos.reliability().kind = RELIABLE_RELIABILITY_QOS;
    response_writer = publisher->create_datawriter(response_topic, writer_qos);

    bool success1 = onnx_model_tort.init("/home/bianbu/bipedal-robot-sim/riscv/bipedal-sim-dds/Model/Trot/model.onnx", 39, 390, 10);
    bool success2 = onnx_model_stand.init("/home/bianbu/bipedal-robot-sim/riscv/bipedal-sim-dds/Model/Stand/model.onnx", 39, 390, 10);
    if (success1 && success2) {
        std::cout << "ONNX Model loaded good!" << std::endl;
        onnx_model = &onnx_model_tort; // set default model
    } else {
        std::cerr << "ONNX Model loaded fail!" << std::endl;
    }

    // init rl record
    action_buf.resize(history_length * 10, 0.0f);
    obs_buf.resize(history_length * 39, 0.0f);
    last_action.resize(10, 0.0f);

    action_temp.clear();
    prev_action.clear();

    for (int j = 0; j < 10; j++){
        action_temp.push_back(0.0);
	    action.push_back(init_pos[j]);
        prev_action.push_back(init_pos[j]);
    }
    //hot start
    for (int i = 0; i < history_length; i++)
    {
        std::vector<float> obs;
        //---------------Push data into obsbuf--------------------
        obs.push_back(0);obs.push_back(0);obs.push_back(0);
        obs.push_back(0);obs.push_back(0);obs.push_back(0);
        obs.push_back(0);obs.push_back(0);obs.push_back(0);
        // pos q joint
        for (int j = 0; j < 10; ++j){
            float pos = 0;
            obs.push_back(pos);
            action[j]=init_pos[j];
        }
        // vel q joint
        for (int j = 0; j < 10; ++j){
            float vel = 0;
            obs.push_back(vel);
        }
        // last action
        for (int j = 0; j < 10; ++j){
            obs.push_back(0);
        }
    }
}

RL_Tinker::~RL_Tinker() {
    participant->delete_contained_entities();
    DomainParticipantFactory::get_instance()->delete_participant(participant);
}

void RL_Tinker::RequestListener::on_data_available(DataReader* reader)
{    
    Request request;
    SampleInfo info;
    
    reader->take_next_sample(&request, &info);
    if(!info.valid_data){
        return;
    }

    if (!outer_->rl_gait) {
        if (request.gait_type()[0] !=0){
            outer_->rl_gait = true;
            outer_->dds_start_time = std::chrono::steady_clock::now();
        }else
            return;
    }

    // load onnx model
    if (outer_->rl_gait) {
        if (request.gait_type()[0] == 6){
            outer_->onnx_model = &outer_->onnx_model_stand;
        }else {
            outer_->onnx_model = &outer_->onnx_model_tort;
        }
    }

    //---------------Push data into obsbuf--------------------
    std::vector<float> obs;
    obs.push_back(request.omega()[0]*outer_->omega_scale);
    obs.push_back(request.omega()[1]*outer_->omega_scale);
    obs.push_back(request.omega()[2]*outer_->omega_scale);

    obs.push_back(request.eu_ang()[0]*outer_->eu_ang_scale);
    obs.push_back(request.eu_ang()[1]*outer_->eu_ang_scale);
    obs.push_back(request.eu_ang()[2]*outer_->eu_ang_scale);

    outer_->cmd_x = outer_->cmd_x * (1 - outer_->smooth) + (std::fabs(request.command()[0]) < outer_->dead_zone ? 0.0 : request.command()[0]) * outer_->smooth;
    outer_->cmd_y = outer_->cmd_y * (1 - outer_->smooth) + (std::fabs(request.command()[1]) < outer_->dead_zone ? 0.0 : request.command()[1]) * outer_->smooth;
    outer_->cmd_rate = outer_->cmd_rate * (1 - outer_->smooth) + (std::fabs(request.command()[2]) < outer_->dead_zone ? 0.0 : request.command()[2]) * outer_->smooth;

    obs.push_back(outer_->cmd_x*outer_->lin_vel);
    obs.push_back(outer_->cmd_y*outer_->lin_vel);
    obs.push_back(outer_->cmd_rate*outer_->ang_vel);
 
    // pos q joint
    for (int i = 0; i < 10; ++i){
        float pos = (request.q()[i]  - outer_->init_pos[i])* outer_->pos_scale;
        obs.push_back(pos);
    }
    // vel q joint
    for (int i = 0; i < 10; ++i){
        float vel = request.dq()[i] * outer_->vel_scale;
        obs.push_back(vel);
    }
    // last action
    for (int i = 0; i < 10; ++i){
        obs.push_back(outer_->action_temp[i]);
    }
        
    // obs
    for (int i = 0; i < 39; ++i) {
        outer_->onnx_model->in1[i] = obs[i];
    }
        
    // obs_buf
    if (outer_->obs_buf.size() == 390) {
        for (int i = 0; i < 390; ++i) {
            outer_->onnx_model->in2[i] = outer_->obs_buf[i];
        }
    } else {
        std::cerr << "error: obs_buf len wrong, expect 390, actuall: " << outer_->obs_buf.size() << std::endl;
        return;
    }

    // run
    outer_->onnx_model->run();
    std::vector<float> action_vec(10);
    for (int i = 0; i < 10; ++i) {
        action_vec[i] = outer_->onnx_model->out[i];
    }

    // action_buf update
    if (outer_->action_buf.size() == outer_->history_length * 10) {
        outer_->action_buf.erase(outer_->action_buf.begin(), outer_->action_buf.begin() + 10);
        outer_->action_buf.insert(outer_->action_buf.end(), action_vec.begin(), action_vec.end());
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

    // action
    std::vector<float> action_blend(10); 
    for (int i = 0; i < 10; i++) { 
        action_blend[i] = 0.8f * action_vec[i] + 0.2f * outer_->last_action[i];
    }
    outer_->last_action = action_vec;
        
    // obs_buf update
    if (outer_->obs_buf.size() == outer_->history_length * 39) {
        outer_->obs_buf.erase(outer_->obs_buf.begin(), outer_->obs_buf.begin() + 39);
        outer_->obs_buf.insert(outer_->obs_buf.end(), obs.begin(), obs.end());
    }

    // dds send action
    Response response;
    for (int j = 0; j < 10; j++) { 
        outer_->action[j] = action_blend[j]; 
        outer_->action_temp[j] = action_blend[j];
        response.q_exp()[j] = action_blend[j];
    }
    outer_->response_writer->write(&response);

    // average freq
    outer_->obs_count++;
    auto now = chrono::steady_clock::now();
    double elapsed = chrono::duration<double>(now - outer_->last_print).count();

    if (elapsed >= 1.0) {
        double freq = outer_->obs_count / elapsed;
        cout << "DDS_OBS 平均接收频率: " << fixed << setprecision(2) << freq << " Hz" << endl;
        outer_->obs_count = 0;
        outer_->last_print = now;
    }
}

int main(int argc, char** argv) {
    RL_Tinker tinker_rl;

    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return 0;
}