#ifndef VIRTUAL_VIEWPOINT_H_
#define VIRTUAL_VIEWPOINT_H_

#include<iostream>

#include"math.h"
#include<vector>
#include<string>

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Core> 

#include "configuration.h"
#include"Save_point.h"

typedef Eigen::Vector3d local_3d;

class V_viewpoint
{
    public:
        V_viewpoint()
        {
            V_id = -1;
            cam_id = -1;
            img_id = -1;
            R << 1, 0, 0, 0, 1, 0, 0, 0, 1;
            t << 0, 0, 0;
            //V_img.create(ROW, COL, CV_8UC1);
            cv::Mat temp_img(ROW, COL, CV_8UC1, 255);
            V_img = temp_img;
            std::vector<Save_Point> temp_trans_set(ROW * COL);
            trans_set = temp_trans_set;
        }
        V_viewpoint(int V_id_, int belong_frame_, int belong_cam_,Eigen::Matrix3d R_,Eigen::Vector3d t_);
        ~V_viewpoint(){};
        int V_id;//虚拟视点ID
        //cv::Mat V_img=cv::Mat::zeros(ROW,COL,CV_8UC1);
        //cv::Mat V_img(ROW, COL, CV_8UC1, 255);
        cv::Mat V_img;//三维投影到二维之后的图像
        int cam_id;//所属相机ID
        int img_id;//所属帧ID
        Eigen::Matrix3d R;//所属帧的图像到虚拟视点的旋转
        Eigen::Vector3d t;//所属帧的图像到虚拟视点的位移
        std::vector<Save_Point> trans_set;//所属帧的图像旋转到虚拟视点后，所有三维点打位置信息
        std::vector<std::pair<int,int>> fill_id;//保存补丁（即用来填补空洞）的相机ID，图像ID，本次没有使用
        std::vector<int> holes;//空洞位置
        void create_V_viewpoint(int V_id_, int belong_frame_, int belong_cam_, Eigen::Matrix3d R_, Eigen::Vector3d t_);
        void Compute_Trans(Eigen::Matrix3d K,std::vector<local_3d> input,cv::Mat img);//位姿变换
        void Find_Hole();//寻找空洞
        void Fill_Hole();//填补空洞，本次没有使用，而是直接在主函数中写了个填补的函数，有待改进
        void Show_V_img();//显示位姿变换后的二维图片
        void Save_V_img(std::string img_name);//保存变换后的图片
};
#endif