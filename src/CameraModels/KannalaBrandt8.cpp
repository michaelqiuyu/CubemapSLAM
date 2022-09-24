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

#include "KannalaBrandt8.h"
#include <iostream>

cv::Point2f KannalaBrandt8::project(const cv::Point3f &p3D) {
    const float x2_plus_y2 = p3D.x * p3D.x + p3D.y * p3D.y;
    const float theta = atan2f(sqrtf(x2_plus_y2), p3D.z);
    const float psi = atan2f(p3D.y, p3D.x);

    const float theta2 = theta * theta;
    const float theta3 = theta * theta2;
    const float theta5 = theta3 * theta2;
    const float theta7 = theta5 * theta2;
    const float theta9 = theta7 * theta2;
    const float r = theta + mvParameters[4] * theta3 + mvParameters[5] * theta5
                    + mvParameters[6] * theta7 + mvParameters[7] * theta9;

    return cv::Point2f(mvParameters[0] * r * cos(psi) + mvParameters[2],
                       mvParameters[1] * r * sin(psi) + mvParameters[3]);

}

cv::Point2f KannalaBrandt8::project(const cv::Matx31f &m3D) {
    return this->project(cv::Point3f(m3D(0),m3D(1),m3D(2)));
}

cv::Point2f KannalaBrandt8::project(const cv::Mat &m3D) {
    const float* p3D = m3D.ptr<float>();

    return this->project(cv::Point3f(p3D[0],p3D[1],p3D[2]));
}

Eigen::Vector2d KannalaBrandt8::project(const Eigen::Vector3d &v3D) {
    const double x2_plus_y2 = v3D[0] * v3D[0] + v3D[1] * v3D[1];
    const double theta = atan2f(sqrtf(x2_plus_y2), v3D[2]);
    const double psi = atan2f(v3D[1], v3D[0]);

    const double theta2 = theta * theta;
    const double theta3 = theta * theta2;
    const double theta5 = theta3 * theta2;
    const double theta7 = theta5 * theta2;
    const double theta9 = theta7 * theta2;
    const double r = theta + mvParameters[4] * theta3 + mvParameters[5] * theta5
                    + mvParameters[6] * theta7 + mvParameters[7] * theta9;

    Eigen::Vector2d res;
    res[0] = mvParameters[0] * r * cos(psi) + mvParameters[2];
    res[1] = mvParameters[1] * r * sin(psi) + mvParameters[3];

    return res;
}

cv::Mat KannalaBrandt8::projectMat(const cv::Point3f &p3D) {
    cv::Point2f point = this->project(p3D);
    cv::Mat ret = (cv::Mat_<float>(2,1) << point.x, point.y);
    return ret.clone();
}

float KannalaBrandt8::uncertainty2(const Eigen::Matrix<double,2,1> &p2D)
{
    return 1.f;
}

cv::Mat KannalaBrandt8::unprojectMat(const cv::Point2f &p2D){
    cv::Point3f ray = this->unproject(p2D);
    cv::Mat ret = (cv::Mat_<float>(3,1) << ray.x, ray.y, ray.z);
    return ret.clone();
}

    cv::Matx31f KannalaBrandt8::unprojectMat_(const cv::Point2f &p2D) {
        cv::Point3f ray = this->unproject(p2D);
        cv::Matx31f r{ray.x, ray.y, ray.z};
        return r;
}

cv::Point3f KannalaBrandt8::unproject(const cv::Point2f &p2D) {
    //Use Newthon method to solve for theta with good precision (err ~ e-6)
    cv::Point2f pw((p2D.x - mvParameters[2]) / mvParameters[0], (p2D.y - mvParameters[3]) / mvParameters[1]);
    float scale = 1.f;
    float theta_d = sqrtf(pw.x * pw.x + pw.y * pw.y);
    theta_d = fminf(fmaxf(-CV_PI / 2.f, theta_d), CV_PI / 2.f);

    if (theta_d > 1e-8) {
        //Compensate distortion iteratively
        float theta = theta_d;

        for (int j = 0; j < 10; j++) {
            float theta2 = theta * theta, theta4 = theta2 * theta2, theta6 = theta4 * theta2, theta8 =
                    theta4 * theta4;
            float k0_theta2 = mvParameters[4] * theta2, k1_theta4 = mvParameters[5] * theta4;
            float k2_theta6 = mvParameters[6] * theta6, k3_theta8 = mvParameters[7] * theta8;
            float theta_fix = (theta * (1 + k0_theta2 + k1_theta4 + k2_theta6 + k3_theta8) - theta_d) /
                              (1 + 3 * k0_theta2 + 5 * k1_theta4 + 7 * k2_theta6 + 9 * k3_theta8);
            theta = theta - theta_fix;
            if (fabsf(theta_fix) < precision)
                break;
        }
        //scale = theta - theta_d;
        scale = std::tan(theta) / theta_d;
    }

    return cv::Point3f(pw.x * scale, pw.y * scale, 1.f);
}

cv::Mat KannalaBrandt8::projectJac(const cv::Point3f &p3D) {
    float x2 = p3D.x * p3D.x, y2 = p3D.y * p3D.y, z2 = p3D.z * p3D.z;
    float r2 = x2 + y2;
    float r = sqrt(r2);
    float r3 = r2 * r;
    float theta = atan2(r, p3D.z);

    float theta2 = theta * theta, theta3 = theta2 * theta;
    float theta4 = theta2 * theta2, theta5 = theta4 * theta;
    float theta6 = theta2 * theta4, theta7 = theta6 * theta;
    float theta8 = theta4 * theta4, theta9 = theta8 * theta;

    float f = theta + theta3 * mvParameters[4] + theta5 * mvParameters[5] + theta7 * mvParameters[6] +
              theta9 * mvParameters[7];
    float fd = 1 + 3 * mvParameters[4] * theta2 + 5 * mvParameters[5] * theta4 + 7 * mvParameters[6] * theta6 +
               9 * mvParameters[7] * theta8;

    cv::Mat Jac(2, 3, CV_32F);
    Jac.at<float>(0, 0) = mvParameters[0] * (fd * p3D.z * x2 / (r2 * (r2 + z2)) + f * y2 / r3);
    Jac.at<float>(1, 0) =
            mvParameters[1] * (fd * p3D.z * p3D.y * p3D.x / (r2 * (r2 + z2)) - f * p3D.y * p3D.x / r3);

    Jac.at<float>(0, 1) =
            mvParameters[0] * (fd * p3D.z * p3D.y * p3D.x / (r2 * (r2 + z2)) - f * p3D.y * p3D.x / r3);
    Jac.at<float>(1, 1) = mvParameters[1] * (fd * p3D.z * y2 / (r2 * (r2 + z2)) + f * x2 / r3);

    Jac.at<float>(0, 2) = -mvParameters[0] * fd * p3D.x / (r2 + z2);
    Jac.at<float>(1, 2) = -mvParameters[1] * fd * p3D.y / (r2 + z2);

    return Jac.clone();
}

Eigen::Matrix<double, 2, 3> KannalaBrandt8::projectJac(const Eigen::Vector3d &v3D) {
    double x2 = v3D[0] * v3D[0], y2 = v3D[1] * v3D[1], z2 = v3D[2] * v3D[2];
    double r2 = x2 + y2;
    double r = sqrt(r2);
    double r3 = r2 * r;
    double theta = atan2(r, v3D[2]);

    double theta2 = theta * theta, theta3 = theta2 * theta;
    double theta4 = theta2 * theta2, theta5 = theta4 * theta;
    double theta6 = theta2 * theta4, theta7 = theta6 * theta;
    double theta8 = theta4 * theta4, theta9 = theta8 * theta;

    double f = theta + theta3 * mvParameters[4] + theta5 * mvParameters[5] + theta7 * mvParameters[6] +
              theta9 * mvParameters[7];
    double fd = 1 + 3 * mvParameters[4] * theta2 + 5 * mvParameters[5] * theta4 + 7 * mvParameters[6] * theta6 +
               9 * mvParameters[7] * theta8;

    Eigen::Matrix<double, 2, 3> JacGood;
    JacGood(0, 0) = mvParameters[0] * (fd * v3D[2] * x2 / (r2 * (r2 + z2)) + f * y2 / r3);
    JacGood(1, 0) =  mvParameters[1] * (fd * v3D[2] * v3D[1] * v3D[0] / (r2 * (r2 + z2)) - f * v3D[1] * v3D[0] / r3);

    JacGood(0, 1) = mvParameters[0] * (fd * v3D[2] * v3D[1] * v3D[0] / (r2 * (r2 + z2)) - f * v3D[1] * v3D[0] / r3);
    JacGood(1, 1) = mvParameters[1] * (fd * v3D[2] * y2 / (r2 * (r2 + z2)) + f * x2 / r3);

    JacGood(0, 2) = -mvParameters[0] * fd * v3D[0] / (r2 + z2);
    JacGood(1, 2) = -mvParameters[1] * fd * v3D[1] / (r2 + z2);

    return JacGood;
}

cv::Mat KannalaBrandt8::unprojectJac(const cv::Point2f &p2D) {
    return cv::Mat();
}

cv::Mat KannalaBrandt8::toK() {
    cv::Mat K = (cv::Mat_<float>(3, 3)
            << mvParameters[0], 0.f, mvParameters[2], 0.f, mvParameters[1], mvParameters[3], 0.f, 0.f, 1.f);
    return K;
}

cv::Matx33f KannalaBrandt8::toK_() {
    cv::Matx33f K{mvParameters[0], 0.f, mvParameters[2], 0.f, mvParameters[1], mvParameters[3], 0.f, 0.f, 1.f};

    return K;
}

