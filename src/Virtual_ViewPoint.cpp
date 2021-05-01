#include"Virtual_ViewPoint.h"

#include<fstream>

#include<opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>

//std::ofstream out_file("out_file.txt");
//int out_point = 0;

V_viewpoint::V_viewpoint(int V_id_, int belong_frame_, int belong_cam_,Eigen::Matrix3d R_,Eigen::Vector3d t_)
{
    V_id=V_id_;
    img_id = belong_frame_;
    cam_id = belong_cam_;
    R = R_;
    t = t_;
    std::vector<Save_Point> temp_trans_set(ROW * COL);
    trans_set = temp_trans_set;
    cv::Mat temp_img(ROW, COL, CV_8UC1, 255);
    V_img = temp_img;
}
void V_viewpoint::create_V_viewpoint(int V_id_, int belong_frame_, int belong_cam_,Eigen::Matrix3d R_,Eigen::Vector3d t_)
{
    V_id=V_id_;
    img_id = belong_frame_;
    cam_id = belong_cam_;
    R = R_;
    t = t_;
    std::vector<Save_Point> temp_trans_set(ROW * COL);
    trans_set = temp_trans_set;
    cv::Mat temp_img(ROW, COL, CV_8UC1, 255);
    V_img = temp_img;
}
void V_viewpoint::Compute_Trans(Eigen::Matrix3d K,std::vector<local_3d> input,cv::Mat img)
{
    //std::cout << "enter CT" << std::endl;
    //std::cout << trans_set.size() << std::endl;
    //int i = 0;
    for (int y = 0; y < ROW; y++)
    {
        for (int x = 0; x < COL;x++)
        {
            local_3d trans_3d = R * input[y*COL+x] + t;//进行R，t变换
            local_3d trans_2d = K * trans_3d / trans_3d(2, 0);//过一化，投影到z=1打平面上
            //std::cout << "trans" << std::endl;
            int tx = round(trans_2d(0, 0));
            int ty = round(trans_2d(1, 0));
            /*
            if((ty==0&&tx==826) || (ty==0&&tx==827) || (ty==1&&tx==0))
            {
                std::cout << tx << " " << ty << " " << trans_2d(2,0) << std::endl << std::endl;
                std::cout << K << std::endl << std::endl;
                std::cout << trans_3d << std::endl << std::endl;
                std::cout << R << std::endl << std::endl;
                std::cout << t << std::endl << std::endl;
                std::cout << input[y*COL+x] << std::endl << std::endl;
            }
            */
            //std::cout << tx << " " << ty << " " << trans_2d(2,0) << std::endl;
            if(tx<COL && tx>=0 && ty<ROW && ty>=0 )
            {
                cv::Point2d o2d;
                o2d.x = x;
                o2d.y = y;
                cv::Point2d l2d;
                l2d.x = tx;
                l2d.y = ty;
                cv::Point3d l3d;
                l3d.x = trans_3d(0, 0);
                l3d.y = trans_3d(1, 0);
                l3d.z = trans_3d(2, 0);
                P_connect temp;//保存二维与三维信息
                temp.now_2d = l2d;
                temp.now_3d = l3d;
                temp.origin_2d = o2d;
                trans_set[ty*COL+tx].P_connects.push_back(temp);//放在对应二维位置中
                /*
                if(ty==0&&tx==826)
                {
                    std::cout << "temp" << std::endl;
                }
                */
                if (trans_set[ty*COL+tx].point_size == 0)//判断该位置是否已有点
                {
                    trans_set[ty*COL+tx].front_point = temp;
                }
                else if(trans_set[ty*COL+tx].front_point.now_3d.z > l3d.z)//若已有点，判断深度
                {
                    trans_set[ty*COL+tx].front_point = temp;//更新显示点
                }
                /*
                if(ty==0&&tx==826)
                {
                    std::cout << "front" << std::endl;
                }
                */
                trans_set[ty*COL+tx].point_size++;//更新点的数量
                /*
                if(ty==0&&tx==826)
                {
                    std::cout << "size" << std::endl;
                    std::cout << V_img.at<uchar>(ty, tx) << std::endl;
                    std::cout << img.at<uchar>(0, 0) << std::endl;
                }
                */
                cv::Point2d front_2d = trans_set[ty*COL+tx].front_point.origin_2d;
                V_img.at<uchar>(ty, tx) = img.at<uchar>(front_2d.y, front_2d.x);//提取原图像素至新图中
                /*
                if(ty==0&&tx==826)
                {
                    std::cout << "img" << std::endl;
                }
                */
            }
            //i++;
            //std::cout << i << std::endl;
            /*
            else//保存外点
            {
                out_point++;
                out_file << x << " " << y << endl;
                out_file<< trans_3d(0, 0) << " " << trans_3d(1, 0) << " " << trans_3d(2, 0) << endl;
                out_file << input[y * COL + x](0, 0) << " " << input[y * COL + x](1, 0) << " " << input[y * COL + x](2, 0) << endl;
            }
            */
        }
    }
}
void V_viewpoint::Find_Hole()
{
    for (int i = 0; i < ROW * COL;i++)
    {
        if(trans_set[i].point_size==0)
        {
            int localtion = i;
            holes.push_back(localtion);
        }
    }
}
void V_viewpoint::Fill_Hole()//空
{
    int i = 0;
}
void V_viewpoint::Show_V_img()
{
    cv::imshow("V_img", V_img);
    cv::waitKey();
}
void V_viewpoint::Save_V_img(std::string img_name)
{
    cv::imwrite(img_name, V_img);
}