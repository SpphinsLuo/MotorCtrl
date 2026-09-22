/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <cstdio>
#include <iostream>
#include "useful_math.h"
#include "GLFW_callbacks.h"
#include "MJ_interface.h"
#include "PVT_ctrl.h"
#include "pino_kin_dyn.h"
#include "data_logger.h"

#include <fstream>
#include <sstream>
#include <algorithm>
#include <string>
#include <vector>

// 轨迹数据结构
struct TrajectoryPoint {
    double time;
    std::vector<double> joint_angles;
};

// 加载轨迹数据函数
std::vector<TrajectoryPoint> loadTrajectory(const std::string& filename) {
    std::vector<TrajectoryPoint> trajectory;
    std::ifstream file(filename);
    
    if (!file.is_open()) {
        std::cerr << "Error opening trajectory file: " << filename << std::endl;
        return trajectory;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream ss(line);
        TrajectoryPoint point;
        point.joint_angles.resize(12); // 12个关节角度
        
        // 读取时间
        ss >> point.time;
        
        // 读取12个关节角度
        for (int i = 0; i < 12; ++i) {
            ss >> point.joint_angles[i];
        }
        
        trajectory.push_back(point);
    }
    
    file.close();
    return trajectory;
}




// MuJoCo load and compile model
char error[1000] = "Could not load binary model";
mjModel* mj_model = mj_loadXML("../models_flj/scene_ssj_V6.xml", 0, error, 1000);
mjData* mj_data = mj_makeData(mj_model);

//************************
// main function
int main(int argc, const char** argv)
{
    
    // 添加状态变量
    enum RobotStatePhase {
        SQUATTING,    // 下蹲阶段
        PAUSING,      // 暂停阶段
        CONTINUING    // 继续阶段
    };
    RobotStatePhase currentPhase = SQUATTING;
    double phaseStartTime = 0.0; // 当前阶段开始时间
    
    //load data
    std::vector<TrajectoryPoint> trajectory = loadTrajectory("../trajectory_data/trajectory_data_V3.txt");

    if (trajectory.empty())
    {
        std::cerr<<"Fail to load trajectory data!"<<std::endl;
        return -1;
    }
     
    
    // 关节索引映射
    const std::vector<int> joint_indices = {
        0, 1, 2, 3, 4, 5,  // 左腿关节
        6, 7, 8, 9, 10, 11  // 右腿关节
    };
    
    // ini classes
    UIctr uiController(mj_model,mj_data);   // UI control for Mujoco
    MJ_Interface mj_interface(mj_model, mj_data); // data interface for Mujoco
    Pin_KinDyn kinDynSolver("../models_flj/model_ssj_V6Fixed.urdf"); // kinematics and dynamics solver
    DataBus RobotState(kinDynSolver.model_nv); // data bus
    PVT_Ctr pvtCtr(mj_model->opt.timestep,"../common/joint_ctrl_config.json");// PVT joint control
    DataLogger logger("../record/datalog.log"); // data logger

    // variables ini
    double stand_legLength = 0.650; //-0.95; // desired baselink height
    double foot_height =0.065; // distance between the foot ankel joint and the bottom
    double  xv_des = 0.7;  // desired velocity in x direction

    RobotState.width_hips = 0.362;
    //mju_copy(mj_data->qpos, mj_model->key_qpos, mj_model->nq*1); // set ini pos in Mujoco
    int model_nv=kinDynSolver.model_nv;

    // ini position and posture for foot-end and hand
    std::vector<double> motors_pos_des(model_nv-6,0);//期望关节位置（弧度）
    std::vector<double> motors_pos_cur(model_nv-6,0);//当前关节位置
    std::vector<double> motors_vel_des(model_nv-6,0);//期望关节速度（弧度/秒）
    std::vector<double> motors_vel_cur(model_nv-6,0);//当前实际关节速度
    std::vector<double> motors_tau_des(model_nv-6,0);//期望关节扭矩（Nm）
    std::vector<double> motors_tau_cur(model_nv-6,0);//当前实际关节扭矩

    // register variable name for data logger
    logger.addIterm("simTime", 1);
    logger.addIterm("motors_pos_cur",model_nv-6);
    logger.addIterm("motors_pos_des",model_nv-6);
    logger.addIterm("motors_tau_cur",model_nv-6);
    logger.addIterm("motors_vel_des",model_nv-6);
    logger.addIterm("motors_vel_cur",model_nv-6);
    logger.finishItermAdding();

    /// ----------------- sim Loop ---------------
    double simEndTime=15;
    mjtNum simstart = mj_data->time;
    double simTime = mj_data->time;
    // double startSteppingTime=3;
    // double startWalkingTime=5;

    // 添加定时器变量
    double lastUpdateTime = 0.0;
    int currentTrajectoryIndex = 0;

    

    // init UI: GLFW
    uiController.iniGLFW();
    uiController.enableTracking(); // enable viewpoint tracking of the body 1 of the robot
    uiController.createWindow("Demo",false);

    while( !glfwWindowShouldClose(uiController.window))
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        simstart=mj_data->time;
        while( mj_data->time - simstart < 1.0/60.0 )
        {
            mj_step(mj_model, mj_data);

            simTime=mj_data->time;
            // printf("-------------%.3f s------------\n",simTime);
            mj_interface.updateSensorValues();
            mj_interface.dataBusWrite(RobotState);


            // 状态机逻辑
            switch (currentPhase) 
            {
                case SQUATTING:
                // 正常更新关节角度
                if (simTime - lastUpdateTime >= 0.001 && currentTrajectoryIndex < trajectory.size()) 
                {
                    TrajectoryPoint currentPoint = trajectory[currentTrajectoryIndex];
                    for (int i = 0; i < joint_indices.size(); ++i) 
                    {
                        motors_pos_des[joint_indices[i]] = currentPoint.joint_angles[i];
                    }
                lastUpdateTime = simTime;
                currentTrajectoryIndex++;
                std::cout<<"SQUATTING"<<"time is "<<simTime<<std::endl;
                }
            
                // 检查是否到达0.5秒
                if (simTime == 0.5) 
                {
                    currentPhase = PAUSING;
                    phaseStartTime = simTime;  // 记录暂停开始时间

                    std::cout<<"PAUSING"<<"time is "<<simTime<<std::endl;
                
                    // 释放基座关节（关键修改！）
                    int baseJointId = mj_name2id(mj_model, mjOBJ_JOINT, "base_link");
                    if (baseJointId != -1) 
                    {
                        mj_model->jnt_type[baseJointId] = mjJNT_FREE;  // 改为自由关节
                        mj_forward(mj_model, mj_data);  // 重新初始化模型
                        std::cout << "基座已释放为浮动状态！" << std::endl;
                    }
                }
                break;
            
                case PAUSING:
                // 暂停期间不更新关节角度
                if (simTime - phaseStartTime >= 2.0) 
                {
                    currentPhase = CONTINUING;
                    phaseStartTime = simTime;
                
                    // 定位当前时间对应的轨迹点
                    for (int i = 0; i < trajectory.size(); i++) 
                    {
                        if (trajectory[i].time >= simTime) 
                        {
                            currentTrajectoryIndex = i;
                            break;
                        }
                    }
                }
                break;
            
                case CONTINUING:
                // 继续更新关节角度
                if (simTime - lastUpdateTime >= 0.001 && currentTrajectoryIndex < trajectory.size()) 
                {
                    TrajectoryPoint currentPoint = trajectory[currentTrajectoryIndex];
                    for (int i = 0; i < joint_indices.size(); ++i) 
                    {
                        motors_pos_des[joint_indices[i]] = currentPoint.joint_angles[i];
                    }
                    lastUpdateTime = simTime;
                    currentTrajectoryIndex++;
                }
                break;
            }

        //     // 轨迹更新逻辑
        // if (currentPhase == SQUATTING || currentPhase == CONTINUING)
        //  {
        //     if (simTime - lastUpdateTime >= 0.001 && currentTrajectoryIndex < trajectory.size()) 
        //     {
        //         TrajectoryPoint currentPoint = trajectory[currentTrajectoryIndex];
                
        //         for(int i = 0; i < joint_indices.size(); ++i) 
        //         {
        //             motors_pos_des[joint_indices[i]] = currentPoint.joint_angles[i];
        //         }
                
        //         lastUpdateTime = simTime;
        //         currentTrajectoryIndex++;
                
        //         std::cout << "Time: " << simTime << "s, Index: " << currentTrajectoryIndex 
        //                   << ", Joint angles updated." << std::endl;
                
        //         // 检查是否完成下蹲
        //         if (currentPhase == SQUATTING && simTime >= 0.5) 
        //         {
        //             currentPhase = PAUSING;
        //             phaseStartTime = simTime;
        //             std::cout << "Entering pause phase at " << simTime << "s" << std::endl;
                    
        //             // 切换为浮动基座
        //             for (int i = 0; i < 6; i++) 
        //             {
        //                 mj_data->qvel[i] = 0.0;
        //             }
        //         }
        //     }
        // }

        // // 处理暂停阶段
        // if (currentPhase == PAUSING) 
        // {
        //     if (simTime - phaseStartTime >= 2.0) 
        //     {
        //         currentPhase = CONTINUING;
        //         phaseStartTime = simTime;
        //         std::cout << "Resuming trajectory at " << simTime << "s" << std::endl;
                
        //         // 调整轨迹索引
        //         double resumeTime = simTime;
        //         for (int i = 0; i < trajectory.size(); i++) 
        //         {
        //             if (trajectory[i].time >= resumeTime) 
        //             {
        //                 currentTrajectoryIndex = i;
        //                 break;
        //             }
        //         }
        //     }
        // }
            
            // // 每0.001秒更新一次关节角度
            // if (simTime - lastUpdateTime >= 0.001 && currentTrajectoryIndex < trajectory.size())
            // {
            //     // 获取当前时间对应的轨迹点
            //     TrajectoryPoint currentPoint = trajectory[currentTrajectoryIndex];
                
            //     // 设置关节角度
            //     for(int i = 0; i < joint_indices.size(); ++i) {
            //         motors_pos_des[joint_indices[i]] = currentPoint.joint_angles[i];
            //     }
                
            //     // 更新状态
            //     lastUpdateTime = simTime;
            //     currentTrajectoryIndex++;
                
            //     // 输出调试信息
            //     std::cout << "Time: " << simTime << "s, Index: " << currentTrajectoryIndex 
            //               << ", Joint angles updated." << std::endl;
            // }

          
            // Enter here functions to send actuator commands, like:
            // arm-l: 0-6, arm-r: 7-13, head: 14,15 waist: 16-18, leg-l: 19-24, leg-r: 25-30
            // get the final joint command
            
            //机器人状态更新
            // RobotState.motors_pos_des= eigen2std(zeroJointPos);
            RobotState.motors_pos_des= motors_pos_des;
            RobotState.motors_vel_des=motors_vel_des;
            RobotState.motors_tor_des=motors_tau_des;
//            Eigen::VectorXd tmp=resLeg.jointPosRes+resHand.jointPosRes;
//            std::cout<<tmp.transpose()<<std::endl;
//            std::cout<<resHand.itr<<std::endl;
//            std::cout<<resHand.err.transpose()<<std::endl;

            pvtCtr.dataBusRead(RobotState);// 读取机器人状态
            
            //限制前三秒力矩，执行关节控制
            if (simTime<=3)
            {
                pvtCtr.calMotorsPVT(100.0/1000.0/180.0*3.1415);  // limit velocity
            }
            else
            {
//                pvtCtr.setJointPD(100,10,"Joint-ankel-l-pitch");
//                pvtCtr.setJointPD(100,10,"Joint-ankel-l-roll");
//                pvtCtr.setJointPD(100,10,"Joint-ankel-r-pitch");
//                pvtCtr.setJointPD(100,10,"Joint-ankel-r-roll");
//                pvtCtr.setJointPD(1000,100,"Joint-knee-l-pitch");
//                pvtCtr.setJointPD(1000,100,"Joint-knee-r-pitch");
                pvtCtr.calMotorsPVT();
            }
            pvtCtr.dataBusWrite(RobotState);//写入机器人状态

            mj_interface.setMotorsTorque(RobotState.motors_tor_out);

            logger.startNewLine();
            logger.recItermData("simTime", simTime);
            logger.recItermData("motors_pos_cur",RobotState.motors_pos_cur);
            logger.recItermData("motors_pos_des",RobotState.motors_pos_des);
            logger.recItermData("motors_tau_cur",RobotState.motors_tor_out);
            logger.recItermData("motors_vel_cur",RobotState.motors_vel_cur);
            logger.recItermData("motors_vel_des",RobotState.motors_vel_des);
            logger.finishLine();
        }

        //到达结束时间结束循环
        if (mj_data->time>=simEndTime)
        {
            break;
        }

        uiController.updateScene();//更新UI界面
    }

//    // free visualization storage
    uiController.Close();

    // free MuJoCo model and data, deactivate
    mj_deleteData(mj_data);
    mj_deleteModel(mj_model);

    return 0;
}