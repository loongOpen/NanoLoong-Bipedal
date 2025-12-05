#include "gait_math.h"
#include "eso.h"
#include "locomotion_header.h"
#include "include.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "can.h"
#include <sys/time.h>
#include <iostream>
#include <chrono>
#if RL_USE_ONNX
    #include "rl_onnx.h"
    OnnxRuntime::OnnxRuntimeClass onnx_model;// 模型推理对象
    vecNf(39) obs1;
    vecNf(39 * 10) obs10;
    // matNf(39,10) obs10;

    // vecNf(41*15) obs_all;
    vecNf(10) action, actionOld, actionTmp, actionFilt, target_q;//添加目标力矩
    
    float smooth_rc = 0.03;
    float dead_zone_rc = 0.01;

    float cmd_x = 0.;
    float cmd_y = 0.;
    float cmd_rate = 0.;

    float cmd_x_temp = 0.;
    float cmd_y_temp = 0.;
    float cmd_rate_temp = 0.;

    float eu_ang_scale = 1;
    float omega_scale = 0.25;
    float pos_scale = 1.0;
    float vel_scale = 0.05;
    float lin_vel = 2.0;
    float ang_vel = 0.25;

    int id_list[10] = {0,1,2,3,4, 7,8,9,10,11};
    int rl_model_loaded=0;
    int first_loaded_model=0;

#endif

void  Gait_RL_Active(char rst)
{
    //重置voice_cmd
    voice_cmd.cmd_type = 0;
    voice_cmd.cmd_data = 0;
    int i = 0;
    //state estimateor
    printf("RL::Activate!\n");
    vmc_all.param.robot_mode=M_RL;vmc_all.gait_mode=G_RL;
#if RUN_PI
    robotwb.exp_att.yaw =robotwb.now_att.yaw;
    robot_param_read();//reset param

    vmc_all.rl_gain=1;//doghome
    leg_motor_all.stiff[0]=config_gait["imp_param"]["stiff_q00"].as<float>()*vmc_all.rl_gain;
    leg_motor_all.stiff[1]=config_gait["imp_param"]["stiff_q01"].as<float>()*vmc_all.rl_gain;
    leg_motor_all.stiff[2]=config_gait["imp_param"]["stiff_q02"].as<float>()*vmc_all.rl_gain;
    leg_motor_all.stiff[3]=config_gait["imp_param"]["stiff_q03"].as<float>()*vmc_all.rl_gain;
    leg_motor_all.stiff[4]=config_gait["imp_param"]["stiff_q04"].as<float>()*vmc_all.rl_gain;
    leg_motor_all.stiff[5]=config_gait["imp_param"]["stiff_q05"].as<float>()*vmc_all.rl_gain;
    leg_motor_all.stiff[6]=config_gait["imp_param"]["stiff_q06"].as<float>()*vmc_all.rl_gain;

    leg_motor_all.stiff[7]=config_gait["imp_param"]["stiff_q10"].as<float>()*vmc_all.rl_gain;
    leg_motor_all.stiff[8]=config_gait["imp_param"]["stiff_q11"].as<float>()*vmc_all.rl_gain;
    leg_motor_all.stiff[9]=config_gait["imp_param"]["stiff_q12"].as<float>()*vmc_all.rl_gain;
    leg_motor_all.stiff[10]=config_gait["imp_param"]["stiff_q13"].as<float>()*vmc_all.rl_gain;
    leg_motor_all.stiff[11]=config_gait["imp_param"]["stiff_q14"].as<float>()*vmc_all.rl_gain;
    leg_motor_all.stiff[12]=config_gait["imp_param"]["stiff_q15"].as<float>()*vmc_all.rl_gain;
    leg_motor_all.stiff[13]=config_gait["imp_param"]["stiff_q16"].as<float>()*vmc_all.rl_gain;

    //vmc_all.rl_commond_rl_rst[0]=0.3;//tinker
    vmc_all.net_run_dt=0.01;
#endif
    vmc_all.att_measure_bias[PITr]=config_gait["vmc_param"]["att_bias_pit"].as<float>();
    vmc_all.att_measure_bias[ROLr]=config_gait["vmc_param"]["att_bias_rol"].as<float>();

#if RL_USE_ONNX
    if(rst!=vmc_all.rl_mode_used&&rst!=0){
        printf("RL:loaded start=%d\n",rst);
        if(!first_loaded_model){
            first_loaded_model=1;//首次加载模型
            // action.setZero();
            action.setZero();
            actionOld.setZero();
            actionTmp.setZero();
            target_q.setZero();

            obs1.setZero();
            obs10.setZero();
            // obs_all.setZero();

        }
        vmc_all.net_run_dt=0.024;
        int loaded=0;

        switch(rst){
            case 1:loaded=onnx_model.init("/home/bianbu-tinker/bipedal-robot-real/Tinker/Model/Trot/model.onnx", 39, 390, 10);break;//trot
            case 2:loaded=onnx_model.init("/home/bianbu-tinker/bipedal-robot-real/Tinker/Model/Stand/model.onnx", 39, 390, 10);break;//stand
        }

        if(loaded){
            rl_model_loaded=1;
            vmc_all.rl_mode_used=rst;
            printf("RL:: Model policy=%d loaded good!\n",rst);
        }
        else{
            vmc_all.rl_mode_used=0;
            printf("RL:: Model loaded fail!\n");
            ocu.cmd_robot_state = 2;
            vmc_all.param.robot_mode = M_STAND_RC;	vmc_all.gait_mode = STAND_RC;
            Gait_Stand_Active();//
            for(i=0;i<14;i++)
                leg_motor_all.q_set_servo[i]=move_joint_to_pos_all(leg_motor_all.q_set_servo[i],leg_motor_all.q_set_servo_init[i],90,0.005);//doghome
        }
    }else{
        printf("RL:: used estited Model policy loaded good!\n");
        vmc_all.rl_commond_rl_rst[0]=0;
        vmc_all.net_run_dt=config_gait["rl_gait"]["net_run_dt"].as<float>();
        rl_model_loaded=1;
    }
#endif
}

void  Gait_RL_Update(float dt)
{
    // auto test_time1 = std::chrono::steady_clock::now();
    static float timer[10];
    timer[0]+=dt;
    // printf("推理时dt=%.4f\n",dt);
#if RL_USE_ONNX
    if(timer[0]>vmc_all.net_run_dt&&rl_model_loaded){//50Hz to ONNX

        #if 0//打印onnx实际推理频率
        static struct timeval last_time_onnx = {0};
        struct timeval now_onnx;
        gettimeofday(&now_onnx, NULL);
        if (last_time_onnx.tv_sec != 0) {
            long usec_onnx = (now_onnx.tv_sec - last_time_onnx.tv_sec) * 1000000L +(now_onnx.tv_usec - last_time_onnx.tv_usec);
            double fps_onnx = 1000000.0 / (double)usec_onnx;
            printf("RL推理循环调用 Frame Interval: %.1ld ms, Frame Rate: %.1f FPS\n", usec_onnx/1000, fps_onnx);
        }
        last_time_onnx = now_onnx;
        #endif

        timer[0]=0;
        // 命令处理
        float cmd_x_temp = 0;//LIMIT(vmc_all.tar_spd.x * 1.5, -0.15, 1.0) + vmc_all.rl_commond_off[4] * vmc_all.rl_commond_off[0];
        float cmd_y_temp = 0;//LIMIT(vmc_all.tar_spd.y * 1.5, -0.35, 0.35) + vmc_all.rl_commond_off[4] * vmc_all.rl_commond_off[1];
        //printf("%f %f %f\n",vmc_all.rl_commond_off[4],vmc_all.rl_commond_off[0],vmc_all.rl_commond_off[1]);
        cmd_x_temp=   LIMIT(vmc_all.tar_spd.x,-MAX_SPD_X*0.5,MAX_SPD_X)+vmc_all.rl_commond_rl_rst[0]+
                vmc_all.rl_commond_off[4]*vmc_all.rl_commond_off[0]*(vmc_all.gait_mode==G_RL);
        cmd_y_temp=   LIMIT(-vmc_all.tar_spd.y,-MAX_SPD_Y,MAX_SPD_Y)+
                vmc_all.rl_commond_off[4]*vmc_all.rl_commond_off[1]*(vmc_all.gait_mode==G_RL);
        //printf("MAX_SPD_X=%f\n",MAX_SPD_X);
        float cmd_rate_temp = 0;
        if (fabs(vmc_all.tar_spd.z) > 1)
        {
            cmd_rate_temp = vmc_all.tar_spd.z/57.3*1.5;
            robotwb.exp_att.yaw = robotwb.now_att.yaw;
        }
        else {
            cmd_rate_temp = -dead(limitw(To_180_degrees(robotwb.exp_att.yaw - robotwb.now_att.yaw), -25, 25), 0.25)*1.4/57.3;
        }
        cmd_rate_temp=   LIMIT(cmd_rate_temp,-1,1);        
        
        // 平滑处理
        cmd_x = cmd_x * (1 - smooth_rc) + (std::fabs(cmd_x_temp) < dead_zone_rc ? 0.0 : cmd_x_temp) * smooth_rc;
        cmd_y = cmd_y * (1 - smooth_rc) + (std::fabs(cmd_y_temp) < dead_zone_rc ? 0.0 : cmd_y_temp) * smooth_rc;
        cmd_rate = cmd_rate * (1 - smooth_rc) + (std::fabs(cmd_rate_temp) < dead_zone_rc ? 0.0 : cmd_rate_temp) * smooth_rc;
        
        //IMU 初始校准归零
        static int imu_print_cnt = 0;
        static float roll_temp = 0.0;
        static float pitch_temp = 0.0;
        static float yaw_temp = 0.0;
        if (imu_print_cnt++<1){
            printf("实际rl.cpp内频率控制 net_run_dt=%.4f\n",vmc_all.net_run_dt);
            printf("IMU初始:\n");
            // printf("vmc_all.att[ROLr]=%.2f\n",vmc_all.att[ROLr]);
            // printf("vmc_all.att[PITr]=%.2f\n",vmc_all.att[PITr]);
            // printf("vmc_all.att[YAWr]=%.2f\n",vmc_all.att[YAWr]);
            roll_temp = vmc_all.att[ROLr];
            pitch_temp = vmc_all.att[PITr];
            yaw_temp = vmc_all.att[YAWr];
        }
        vmc_all.att[ROLr] = vmc_all.att[ROLr]-roll_temp;
        vmc_all.att[PITr] = vmc_all.att[PITr]-pitch_temp;
        vmc_all.att[YAWr] = vmc_all.att[YAWr]-yaw_temp;

        #if 1 // use_usb_imu = 1 （x后 y左 z下）
        obs1 << -vmc_all.att_rate[ROLr] / 57.3 * omega_scale,
                vmc_all.att_rate[PITr] / 57.3 * omega_scale,
                -vmc_all.att_rate[YAWr] / 57.3 * omega_scale,
                -vmc_all.att[ROLr] / 57.3 * eu_ang_scale,
                vmc_all.att[PITr] / 57.3 * eu_ang_scale,
                -vmc_all.att[YAWr] / 57.3 * eu_ang_scale,
                 cmd_x * lin_vel,
                 cmd_y * lin_vel,
                 cmd_rate * ang_vel;
        #else // use_usb_imu = 0 （x前 y右 z下）
        obs1 <<  vmc_all.att_rate[ROLr] / 57.3 * omega_scale,
                -vmc_all.att_rate[PITr] / 57.3 * omega_scale,
                -vmc_all.att_rate[YAWr] / 57.3 * omega_scale,
                 vmc_all.att[ROLr] / 57.3 * eu_ang_scale,
                -vmc_all.att[PITr] / 57.3 * eu_ang_scale,
                -vmc_all.att[YAWr] / 57.3 * eu_ang_scale,
                 cmd_x * lin_vel,
                 cmd_y * lin_vel,
                 cmd_rate * ang_vel;
        #endif

        // printf("net_input:%f %f %f\n",cmd_x,cmd_y,cmd_rate);
        
        // 添加关节信息
        for (int i = 0; i < 10; i++)
        {
            #if 0 //初始位置
            static bool default_pos_cnt = false;
            if (!default_pos_cnt){
                for (int j=0; j<10; j++){
                    printf("打印初始default_action位置：%.4f\n", vmc_all.default_action[id_list[j]]);
                }
                default_pos_cnt = true;
            }
            #endif
        
            obs1[9 + i] = (leg_motor_all.q_now[id_list[i]] / 57.3 - vmc_all.default_action[id_list[i]]) * pos_scale;
            obs1[19 + i] = (leg_motor_all.qd_now[id_list[i]] / 57.3) * vel_scale;
            obs1[29 + i] = action[i];
        }

        #if 0
        // ======打印rl推理obs输入======
        static int obs_cnt = 0;
        if(obs_cnt++ < 5){
            printf("======第%d帧 RL推理输入OBS ======\n", obs_cnt);
            printf("**omega**\n");
            for(int i = 0; i < 3; i++ ){
                printf("obs1[%d]: %.2f\n", i, obs1[i]);
            }
            printf("**eu_ang**\n");
            for(int i = 3; i < 6; i++ ){
                printf("obs1[%d]: %.2f\n", i, obs1[i]);
            }
            printf("**cmd**\n");
            for(int i = 6; i < 9; i++ ){
                printf("obs1[%d]: %.2f\n", i, obs1[i]);
            }
            printf("**q**\n");
            for(int i = 0; i < 10; i++ ){
                printf("obs1[%d]: %.2f\n", 9+i, vmc_all.default_action[id_list[i]] + obs1[9+i]);
            }
            printf("**dq**\n");
            for(int i = 0; i < 10; i++ ){
                printf("obs1[%d]: %.2f\n", 19+i, obs1[19+i]);
            }
            printf("**actionOld**\n");
            for(int i = 0; i < 10; i++ ){
                printf("obs1[%d]: %.2f\n", 29+i, obs1[29+i]);
            }
        }
        #endif

        // 更新历史观测
        obs10.head<39 * 9>() = obs10.tail<39 * 9>();
        obs10.tail<39>() = obs1;

        // 设置ONNX模型输入并运行推理
        onnx_model.in1 = obs1;
        onnx_model.in2 = obs10;
        onnx_model.run();

        // 动作滤波
        action = actionOld * 0.2 + onnx_model.out * 0.8;
        actionOld = onnx_model.out;
    }

    // // 动作滤波
    // action = actionOld * 0.2 + onnx_model.out * 0.8;
    // actionOld = onnx_model.out;
        
    for (int i = 0; i < 10; i++)
    {
        action[i]=LIMIT(action[i], -5, 5);
    }
    actionTmp = action;

    #if 0//打印RL实际电机控制帧率
    static struct timeval last_time = {0};
    struct timeval now;
    gettimeofday(&now, NULL);
    if (last_time.tv_sec != 0) {
        long usec = (now.tv_sec - last_time.tv_sec) * 1000000L +(now.tv_usec - last_time.tv_usec);
        double fps = 1000000.0 / (double)usec;
        printf("RL推理循环调用 Frame Interval: %.1ld ms, Frame Rate: %.1f FPS\n", usec/1000, fps);
    }
    last_time = now;
    #endif

    // 输出action转化 ==> 电机目标角度值：rad
    for (int i = 0; i < 10; i++){
        target_q[i] = actionTmp[i] * vmc_all.action_scale + vmc_all.default_action[id_list[i]];
    }
    
    #if 0 //打印电机目标角度值：rad
        static int target_q_cnt = 0;
        if(target_q_cnt++ > 50){
        printf("******************** 每秒打印1、6电机目标位置输出********************\n");
        printf("target_q[1]=%.2f\n",target_q[1]);
        printf("target_q[6]=%.2f\n",target_q[6]);
        target_q_cnt=0;
        }
    #endif
    for (int i = 0; i < 10; i++){
        // float temp = actionTmp[i] * vmc_all.action_scale + vmc_all.default_action[id_list[i]];
        // leg_motor_all.q_set[id_list[i]] = temp * 57.3;
        leg_motor_all.q_set[id_list[i]] = target_q[i] * 57.3;//弧度转角度
    }
    
    // auto test_time2 = std::chrono::steady_clock::now();
    // std::chrono::duration<double, std::milli> test_time_rl = test_time2 - test_time1;
    // double fps_rl = 1000.0 / test_time_rl.count();
    // printf("RL推理过程实际耗时 Frame Interval: %.1fms, Frame Rate: %.1f FPS\n",test_time_rl.count(), fps_rl);

#endif
}
