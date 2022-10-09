#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;


class System {
public:
    System() {};

    void CreateUndistortRectifyMap();

    void CvtFisheyeToCubeMap_reverseQuery_withInterpolation(cv::Mat &cubemapImg, const cv::Mat &fisheyeImg,
         int interpolation, int borderType=cv::BORDER_CONSTANT, const cv::Scalar& borderValue=cv::Scalar());


public:


private:
    cv::Mat mMap1;
    cv::Mat mMap2;

    int width3 = 650 * 3;
    int height3 = 650 * 3;
    int Iw = 1920;
    int Ih = 1080;
    int width = 650;
    int height = 650;

};

void System::CreateUndistortRectifyMap() {
    //create map for u(mMap1) and v(mMap2)
    mMap1.create(height3, width3, CV_32F);
    mMap2.create(height3, width3, CV_32F);
    mMap1.setTo(cv::Scalar::all(0));
    mMap2.setTo(cv::Scalar::all(0));

    for(int y = 0; y < height3; ++y)
    {
        for(int x = 0; x < width3; ++x)
        {
            double u, v;
            CamModelGeneral::GetCamera()->CubemapToFisheye(u, v, static_cast<double>(x), static_cast<double>(y));
            // on some face but doesn't map to a fisheye valid region
            if(u < 0 || v < 0 || u >= Iw || v >= Ih)
                continue;
            mMap1.at<float>(y, x) = static_cast<float>(u);
            mMap2.at<float>(y, x) = static_cast<float>(v);
        }
    }
}

void System::CubemapToFisheye(double &uf, double &vf, const double &up, const double &vp)
{
    //cvt (up, vp) to (i, j)
    float i = up, j = vp;
    uf = -1; vf = -1;
    eFace face = FaceInCubemap<float>(i, j);
    if(face == UNKNOWN_FACE)
        return;

    double x, y, z = 1.0;
    i = i - static_cast<int>(i / mWCubeFace) * mWCubeFace; j = j - static_cast<int>(j / mHCubeFace) * mHCubeFace;

    x = (i - cx) * z / fx;
    y = (j - cy) * z / fy;
    const cv::Vec3d localPt(x, y, z);
    cv::Vec3d rigPt;
    double &_x = rigPt(0), &_y = rigPt(1), &_z = rigPt(2);

    cvtFacesToRig<double>(rigPt, localPt, face);
    WorldToImg(_x, _y, _z, uf, vf);
    if(uf < 0 || uf >= mWFisheye || vf < 0 || vf >= mHFisheye)
    {
        uf = -1;
        vf = -1;
    }
}


//convert fisheye image to cubemap
void System::CvtFisheyeToCubeMap_reverseQuery_withInterpolation(cv::Mat &cubemapImg, const cv::Mat &fisheyeImg,
     int interpolation, int borderType, const cv::Scalar& borderValue) {
    const int offset = 0;
    cv::Mat cubemapImg_front = cubemapImg.rowRange(height, 2 * height).colRange(width, 2 * width);
    cv::Mat cubemapImg_left = cubemapImg.rowRange(height, 2 * height).colRange(0+offset, width+offset);
    cv::Mat cubemapImg_right = cubemapImg.rowRange(height, 2 * height).colRange(2 * width-offset, 3 * width-offset);
    cv::Mat cubemapImg_upper = cubemapImg.rowRange(0+offset, height+offset).colRange(width, 2 * width);
    cv::Mat cubemapImg_lower = cubemapImg.rowRange(2 * height-offset, 3 * height-offset).colRange(width, 2 * width);

    cv::Mat mMap1_front = mMap1.rowRange(height, 2 * height).colRange(width, 2 * width);
    cv::Mat mMap1_left = mMap1.rowRange(height, 2 * height).colRange(0+offset, width+offset);
    cv::Mat mMap1_right = mMap1.rowRange(height, 2 * height).colRange(2 * width-offset, 3 * width-offset);
    cv::Mat mMap1_upper = mMap1.rowRange(0+offset, height+offset).colRange(width, 2 * width);
    cv::Mat mMap1_lower = mMap1.rowRange(2 * height-offset, 3 * height-offset).colRange(width, 2 * width);

    cv::Mat mMap2_front = mMap2.rowRange(height, 2 * height).colRange(width, 2 * width);
    cv::Mat mMap2_left = mMap2.rowRange(height, 2 * height).colRange(0+offset, width+offset);
    cv::Mat mMap2_right = mMap2.rowRange(height, 2 * height).colRange(2 * width-offset, 3 * width-offset);
    cv::Mat mMap2_upper = mMap2.rowRange(0+offset, height+offset).colRange(width, 2 * width);
    cv::Mat mMap2_lower = mMap2.rowRange(2 * height-offset, 3 * height-offset).colRange(width, 2 * width);
    //interpolation with cv::remap for each face
    cv::remap(fisheyeImg, cubemapImg_front, mMap1_front, mMap2_front, interpolation, borderType, borderValue);
    cv::remap(fisheyeImg, cubemapImg_left, mMap1_left, mMap2_left, interpolation, borderType, borderValue);
    cv::remap(fisheyeImg, cubemapImg_right, mMap1_right, mMap2_right, interpolation, borderType, borderValue);
    cv::remap(fisheyeImg, cubemapImg_upper, mMap1_upper, mMap2_upper, interpolation, borderType, borderValue);
    cv::remap(fisheyeImg, cubemapImg_lower, mMap1_lower, mMap2_lower, interpolation, borderType, borderValue);

#if 1
    cv::imshow("cubemapImg_front", cubemapImg_front);
    cv::imshow("cubemapImg_left", cubemapImg_left);
    cv::imshow("cubemapImg_right", cubemapImg_right);
    cv::imshow("cubemapImg_upper", cubemapImg_upper);
    cv::imshow("cubemapImg_lower", cubemapImg_lower);
#endif
}




int main(int argc, char *argv[]) {
    //read in params


    int offset = 0;
    int width = CamModelGeneral::GetCamera()->GetCubeFaceWidth(), height = CamModelGeneral::GetCamera()->GetCubeFaceHeight();
    cv::Mat cubemapImg(height * 3, width * 3, CV_8U, cv::Scalar::all(0));
    cv::Mat cubemapImg_front = cubemapImg.rowRange(height, 2 * height).colRange(width, 2 * width);
    cv::Mat cubemapImg_left = cubemapImg.rowRange(height, 2 * height).colRange(0+offset, width+offset);
    cv::Mat cubemapImg_right = cubemapImg.rowRange(height, 2 * height).colRange(2 * width-offset, 3 * width-offset);
    cv::Mat cubemapImg_upper = cubemapImg.rowRange(0+offset, height+offset).colRange(width, 2 * width);
    cv::Mat cubemapImg_lower = cubemapImg.rowRange(2 * height-offset, 3 * height-offset).colRange(width, 2 * width);


    std::string fisheyeImgName = "/home/xiongchao/studying/SLAM/VSLAM/cubamapSLAM/dataset/wuhan_avp/rear/1515653813.000000_00000001.tiff";
    cv::Mat fisheyeImg = cv::imread(fisheyeImgName.c_str(), cv::IMREAD_GRAYSCALE);

    if(!fisheyeImg.data)
    {
        std::cout << "fail to read image in " << fisheyeImgName << std::endl;
        exit(EXIT_FAILURE);
    }

    System system;

    system.CvtFisheyeToCubeMap_reverseQuery_withInterpolation(cubemapImg, fisheyeImg, cv::INTER_LINEAR);


    return 0;
}