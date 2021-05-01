#include<iostream>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include"Camera.h"
#include"Frame.h"
#include"Virtual_ViewPoint.h"
using namespace std;
//输入两个虚拟视点，以及补丁图像
void Compelement_Hole(V_viewpoint *v_vp1,V_viewpoint *v_vp2,cv::Mat input)
{
    vector<int>::iterator it;
    //int aim_size = v_vp1.holes.size();
    for (int i = 0; i < v_vp1->holes.size();)
    {
        int localtion = v_vp1->holes[i];
        if (v_vp2->trans_set[localtion].point_size != 0)
        {
            int x = localtion % COL;
            int y = localtion / COL;
            int ox = v_vp2->trans_set[localtion].front_point.origin_2d.x;
            int oy = v_vp2->trans_set[localtion].front_point.origin_2d.y;
            v_vp1->V_img.at<uchar>(y, x) = input.at<uchar>(oy, ox);
            P_connect temp = v_vp2->trans_set[localtion].front_point;
            v_vp1->trans_set[localtion].front_point = temp;
            v_vp1->trans_set[localtion].P_connects.push_back(temp);
            v_vp1->trans_set[localtion].point_size++;
            it = v_vp1->holes.begin() + i;
            v_vp1->holes.erase(it);
            continue;
        }
        i++;
    }
}

int main()
{
    //创建两个相机实例
    std::vector<Camera> cams;
    Camera cam0, cam1;
    //cam0
    cam0.cam_id = 0;
    Eigen::Matrix3d R;
    R <<
          0.962107, -0.005824, 0.272486,
          0.004023, 0.999964, 0.007166,
          -0.272519, -0.005795, 0.962095;
    R = R.inverse();
    cam0.cam_R = R;
    Eigen::Vector3d t;
    t << -14.832727, 0.093097, -0.005195;
    t = -1 * t;
    cam0.cam_t = t;
    
    Eigen::Matrix3d K;
    K <<
          1884.190000, -0.654998, 513.700000,
          0.0, 1887.490000, 395.609000,
          0.0, 0.0, 1.0;
    Eigen::Matrix3d invK = K.inverse();
    
    cam0.K = K;
    cam0.invK = invK;
    cams.push_back(cam0);
    //cam1
    cam1.cam_id = 1;
    Eigen::Matrix3d R2;
    R2 << 0.975810, -0.026010, 0.216939,
        0.022983, 0.999598, 0.016432,
        -0.217280, -0.011048, 0.976016;
    R2 = R2.inverse();
    cam1.cam_R = R2;
    Eigen::Vector3d t2;
    t2 << -11.315863, -0.167907, 0.701363;
    t2 = -1 * t2;
    cam1.cam_t = t2;
    
    Eigen::Matrix3d K2;
    K2 << 1898.030000, 0.282128, 517.910000,
        0.0, 1900.810000, 382.815000,
        0.0, 0.0, 1.0;
    Eigen::Matrix3d invK2 = K2.inverse();
    
    cam1.K = K2;
    cam1.invK = invK2;
    cams.push_back(cam1);
    //读取两个相机中对应的rgb和深度图
    cv::Mat img1 = cv::imread("/home/anoorb2/icp/build/repro_image/cam0/color-cam0-f000.jpg", 0);
    cv::Mat depth_img1 = cv::imread("/home/anoorb2/icp/build/repro_image/cam0/depth-cam0-f000.png", 0);
    cv::Mat img2 = cv::imread("/home/anoorb2/icp/build/repro_image/cam1/color-cam1-f000.jpg", 0);
    cv::Mat depth_img2 = cv::imread("/home/anoorb2/icp/build/repro_image/cam1/depth-cam1-f000.png", 0);
    //创建两个帧对象实例
    Frame *frame1 = new Frame(img1,depth_img1,0,0);
    Frame *frame2 = new Frame(img2,depth_img2,0,1);
    //frame1->Show_img();
    //frame1->Show_depth_img();
    frame1->Compute_Depth(invK);//计算实际深度
    frame2->Compute_Depth(invK2);
    //创建两个虚拟视点实例
    V_viewpoint *v_vp1 = new V_viewpoint(0, 0, 0, R, t);
    V_viewpoint *v_vp2 = new V_viewpoint(0, 0, 0, R2, t2);
    std::string img_name = "trans1.jpg";
    v_vp1->Compute_Trans(cam0.K, frame1->p_3d_set, frame1->img);//计算旋转
    v_vp1->Show_V_img();//显示旋转后的图像
    v_vp1->Save_V_img(img_name);//保存旋转后的图像
    img_name = "trans2.jpg";
    v_vp2->Compute_Trans(cam1.K, frame2->p_3d_set, frame2->img);
    v_vp2->Show_V_img();
    v_vp2->Save_V_img(img_name);
    img_name = "fill.jpg";
    Compelement_Hole(v_vp1, v_vp2, frame2->img);//融合虚拟视点，填补空洞
    v_vp1->Show_V_img();
    v_vp2->Save_V_img(img_name);
    return 0;
}