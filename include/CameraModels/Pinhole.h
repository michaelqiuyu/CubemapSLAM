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

#ifndef PINHOLE_H
#define PINHOLE_H

#include "GeometricCamera.h"

#include <assert.h>
#include <vector>
#include <opencv2/core/core.hpp>

class Pinhole : public GeometricCamera {

public:
    Pinhole() {
        mvParameters.resize(4);
        mnId=nNextId++;
        mnType = CAM_PINHOLE;
    }
    Pinhole(const std::vector<float> _vParameters) : GeometricCamera(_vParameters) {
        assert(mvParameters.size() == 4);
        mnId=nNextId++;
        mnType = CAM_PINHOLE;
    }

    Pinhole(Pinhole* pPinhole) : GeometricCamera(pPinhole->mvParameters) {
        assert(mvParameters.size() == 4);
        mnId=nNextId++;
        mnType = CAM_PINHOLE;
    }

    ~Pinhole() {};

    cv::Point2f project(const cv::Point3f &p3D);
    cv::Point2f project(const cv::Matx31f &m3D);
    cv::Point2f project(const cv::Mat &m3D);
    Eigen::Vector2d project(const Eigen::Vector3d & v3D);
    cv::Mat projectMat(const cv::Point3f& p3D);

    float uncertainty2(const Eigen::Matrix<double,2,1> &p2D);

    cv::Point3f unproject(const cv::Point2f &p2D);
    cv::Mat unprojectMat(const cv::Point2f &p2D);
    cv::Matx31f unprojectMat_(const cv::Point2f &p2D);


    cv::Mat projectJac(const cv::Point3f &p3D);
    Eigen::Matrix<double,2,3> projectJac(const Eigen::Vector3d& v3D);

    cv::Mat unprojectJac(const cv::Point2f &p2D);

    cv::Mat toK();
    cv::Matx33f toK_();

private:
    cv::Mat SkewSymmetricMatrix(const cv::Mat &v);
    cv::Matx33f SkewSymmetricMatrix_(const cv::Matx31f &v);

};

//BOOST_CLASS_EXPORT_KEY(ORBSLAM2::Pinhole)

#endif //PINHOLE_H
