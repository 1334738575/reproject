#include"Camera.h"
Camera::Camera(int cam_id_, Eigen::Matrix3d R_, Eigen::Vector3d t_,Eigen::Matrix3d K_,Eigen::Matrix3d invK_)
{
    cam_id = cam_id_;
    cam_R = R_;
    cam_t = t_;
    K = K_;
    invK = invK_;
}
void Camera::create_Camera(int cam_id_, Eigen::Matrix3d R_, Eigen::Vector3d t_,Eigen::Matrix3d K_,Eigen::Matrix3d invK_)
{
    cam_id = cam_id_;
    cam_R = R_;
    cam_t = t_;
    K = K_;
    invK = invK_;
}