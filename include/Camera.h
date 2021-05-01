#ifndef CAMERA_H_
#define CAMERA_H_

#include<iostream>
#include <Eigen/Core> 
#include <Eigen/Geometry> 

#include<vector>

#include"Frame.h"

class Camera
{
    public:
        Camera()//默认构造函数
        {
            cam_R << 1, 0, 0, 0, 1, 0, 0, 0, 1;
            cam_t << 0, 0, 0;
            cam_id = -1;
        }
        Camera(int cam_id_, Eigen::Matrix3d R_, Eigen::Vector3d t_,Eigen::Matrix3d K_,Eigen::Matrix3d invK_);//创建初始化函数
        ~Camera(){};//解析函数
        Eigen::Matrix3d K;//相机内参
        Eigen::Matrix3d invK;//内参的逆
        Eigen::Matrix3d cam_R;//相机间的旋转（与数据集中给的参数对应）
        Eigen::Vector3d cam_t;//相机间的位移
        std::vector<Frame> frames;//一个相机可以有很多帧，因为只用了两个相机的各一帧，本次没有使用到
        int cam_id;//相机ID
        void create_Camera(int cam_id_, Eigen::Matrix3d R_, Eigen::Vector3d t_, Eigen::Matrix3d K_,Eigen::Matrix3d invK_);//赋值函数
        
};

#endif