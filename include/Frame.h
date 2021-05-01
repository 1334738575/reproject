#ifndef FRAME_H_
#define FRAME_H_

#include<iostream>
#include"math.h"
#include<vector>

#include<opencv2/core/core.hpp>

#include <Eigen/Core> 

#include"Virtual_ViewPoint.h"

#include "configuration.h"

class Frame
{
    public:
        Frame()
        {
            img_id = -1;
            cam_id = -1;
            std::vector<Eigen::Vector3d> temp_3d_set(ROW * COL);
            p_3d_set = temp_3d_set;
        }
        Frame(cv::Mat img_, cv::Mat depth_img_, int img_id_, int cam_id_);//,std::vector<Eigen::Vector3d> p_3d_set_);
        ~Frame(){};
        cv::Mat img;//帧包含的rgb图
        cv::Mat depth_img;//深度图
        int img_id;//图片ID
        int cam_id;//所属相机ID
        std::vector<Eigen::Vector3d> p_3d_set;//映射到三维空间的坐标集
        std::vector<V_viewpoint> V_viewpoints;//每帧可以有多个虚拟视点
        void create_Frame(cv::Mat img_, cv::Mat depth_img_, int img_id_, int cam_id_);//,std::vector<Eigen::Vector3d> p_3d_set_);
        void Compute_Depth(Eigen::Matrix3d invK);//计算深度
        void Show_img();//显示rgb图
        void Show_depth_img();//显示深度图
};

#endif