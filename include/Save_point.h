#ifndef SAVE_POINT_H_
#define SAVE_POINT_H_

#include<iostream>
#include<opencv2/core/core.hpp>
#include<vector>

struct P_connect//依次为原二维点，现二维点，现三维点
{
    cv::Point2d origin_2d;
    cv::Point2d now_2d;
    cv::Point3d now_3d;
};

class Save_Point//点信息集，点集数量，显示的点
{
    public:
        Save_Point ()
        {
            point_size = 0;
        }
        std::vector<P_connect> P_connects;
        int point_size;
        P_connect front_point;
};
#endif