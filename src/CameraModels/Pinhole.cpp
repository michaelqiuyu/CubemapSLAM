/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include "Pinhole.h"


long unsigned int GeometricCamera::nNextId=0;

cv::Point2f Pinhole::project(const cv::Point3f &p3D) {
    return cv::Point2f(mvParameters[0] * p3D.x / p3D.z + mvParameters[2],
                       mvParameters[1] * p3D.y / p3D.z + mvParameters[3]);
}

cv::Point2f Pinhole::project(const cv::Matx31f &m3D) {
    return this->project(cv::Point3f(m3D(0),m3D(1),m3D(2)));
}

cv::Point2f Pinhole::project(const cv::Mat &m3D) {
    const float* p3D = m3D.ptr<float>();

    return this->project(cv::Point3f(p3D[0],p3D[1],p3D[2]));
}

Eigen::Vector2d Pinhole::project(const Eigen::Vector3d &v3D) {
    Eigen::Vector2d res;
    res[0] = mvParameters[0] * v3D[0] / v3D[2] + mvParameters[2];
    res[1] = mvParameters[1] * v3D[1] / v3D[2] + mvParameters[3];

    return res;
}

cv::Mat Pinhole::projectMat(const cv::Point3f &p3D) {
    cv::Point2f point = this->project(p3D);
    return (cv::Mat_<float>(2,1) << point.x, point.y);
}

float Pinhole::uncertainty2(const Eigen::Matrix<double,2,1> &p2D)
{
    return 1.0;
}

cv::Point3f Pinhole::unproject(const cv::Point2f &p2D) {
    return cv::Point3f((p2D.x - mvParameters[2]) / mvParameters[0], (p2D.y - mvParameters[3]) / mvParameters[1],
                       1.f);
}

cv::Mat Pinhole::unprojectMat(const cv::Point2f &p2D){
    cv::Point3f ray = this->unproject(p2D);
    return (cv::Mat_<float>(3,1) << ray.x, ray.y, ray.z);
}

cv::Matx31f Pinhole::unprojectMat_(const cv::Point2f &p2D) {
    cv::Point3f ray = this->unproject(p2D);
    cv::Matx31f r{ray.x, ray.y, ray.z};
    return r;
}

cv::Mat Pinhole::projectJac(const cv::Point3f &p3D) {
    cv::Mat Jac(2, 3, CV_32F);
    Jac.at<float>(0, 0) = mvParameters[0] / p3D.z;
    Jac.at<float>(0, 1) = 0.f;
    Jac.at<float>(0, 2) = -mvParameters[0] * p3D.x / (p3D.z * p3D.z);
    Jac.at<float>(1, 0) = 0.f;
    Jac.at<float>(1, 1) = mvParameters[1] / p3D.z;
    Jac.at<float>(1, 2) = -mvParameters[1] * p3D.y / (p3D.z * p3D.z);

    return Jac;
}

Eigen::Matrix<double, 2, 3> Pinhole::projectJac(const Eigen::Vector3d &v3D) {
    Eigen::Matrix<double, 2, 3> Jac;
    Jac(0, 0) = mvParameters[0] / v3D[2];
    Jac(0, 1) = 0.f;
    Jac(0, 2) = -mvParameters[0] * v3D[0] / (v3D[2] * v3D[2]);
    Jac(1, 0) = 0.f;
    Jac(1, 1) = mvParameters[1] / v3D[2];
    Jac(1, 2) = -mvParameters[1] * v3D[1] / (v3D[2] * v3D[2]);

    return Jac;
}

cv::Mat Pinhole::unprojectJac(const cv::Point2f &p2D) {
    cv::Mat Jac(3, 2, CV_32F);
    Jac.at<float>(0, 0) = 1 / mvParameters[0];
    Jac.at<float>(0, 1) = 0.f;
    Jac.at<float>(1, 0) = 0.f;
    Jac.at<float>(1, 1) = 1 / mvParameters[1];
    Jac.at<float>(2, 0) = 0.f;
    Jac.at<float>(2, 1) = 0.f;

    return Jac;
}

cv::Mat Pinhole::toK() {
    cv::Mat K = (cv::Mat_<float>(3, 3)
            << mvParameters[0], 0.f, mvParameters[2], 0.f, mvParameters[1], mvParameters[3], 0.f, 0.f, 1.f);
    return K;
}

cv::Matx33f Pinhole::toK_() {
    cv::Matx33f K{mvParameters[0], 0.f, mvParameters[2], 0.f, mvParameters[1], mvParameters[3], 0.f, 0.f, 1.f};

    return K;
}

cv::Mat Pinhole::SkewSymmetricMatrix(const cv::Mat &v)
{
    return (cv::Mat_<float>(3,3) <<             0, -v.at<float>(2), v.at<float>(1),
            v.at<float>(2),               0,-v.at<float>(0),
            -v.at<float>(1),  v.at<float>(0),              0);
}

cv::Matx33f Pinhole::SkewSymmetricMatrix_(const cv::Matx31f &v)
{
    cv::Matx33f skew{0.f, -v(2), v(1),
                     v(2), 0.f, -v(0),
                     -v(1), v(0), 0.f};

    return skew;
}

