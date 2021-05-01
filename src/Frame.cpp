#include"Frame.h"

#include<opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>

#include <Eigen/Core> 
#include <Eigen/Geometry> 
#include <Eigen/SVD> 
#include <Eigen/Dense>

//#include "configuration.h"

Frame::Frame(cv::Mat img_, cv::Mat depth_img_, int img_id_, int cam_id_)//,std::vector<Eigen::Vector3d> p_3d_set_)
{
    img = img_;
    depth_img = depth_img_;
    img_id = img_id_;
    cam_id = cam_id_;
    std::vector<Eigen::Vector3d> temp_3d_set(ROW * COL);
    p_3d_set = temp_3d_set;
}
void Frame::create_Frame(cv::Mat img_, cv::Mat depth_img_, int img_id_,int cam_id_)//,std::vector<Eigen::Vector3d> p_3d_set_)
{
    img = img_;
    depth_img = depth_img_;
    img_id = img_id_;
    cam_id = cam_id_;
    std::vector<Eigen::Vector3d> temp_3d_set(ROW * COL);
    p_3d_set = temp_3d_set;
}
void Frame::Compute_Depth(Eigen::Matrix3d invK)
{
    //Eigen::Matrix3d invK=K.inverse();
    //std::cout << "enter CD" << std::endl;
    //std::cout << p_3d_set.size() << std::endl;
    for (int y = 0; y < ROW; y++)
    {
        for (int x = 0; x < COL;x++)
        {
            //std::cout << "start" << std::endl;
            double num = (double)depth_img.at<uchar>(y, x);
            //std::cout << "mid" << std::endl;
            double true_depth = 1.0 / ((num / 255.0) * (1.0 / 44.0 - 1.0 / 120.0) + 1.0 / 120.0);//计算公式
            p_3d_set[y*COL+x] = invK * Eigen::Vector3d(x, y, 1) * true_depth;//保存三维点
            //std::cout << "end" << std::endl;
        }
    }
}
void Frame::Show_img()
{
    cv::imshow("rgb", img);
    cv::waitKey();
}
void Frame::Show_depth_img()
{
    cv::imshow("depth", depth_img);
    cv::waitKey();
}