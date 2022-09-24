#include <iostream>
#include <opencv2/opencv.hpp>


void create_image() {
    std::string video_path = "/home/xiongchao/workspace/leador/project/FeatMap-ORB3/featmap-to-wuhan/project_visfea/workspace/2022-09-02/rear.avi";
    cv::Mat im;
    cv::VideoCapture capture(video_path);
    capture.set(cv::CAP_PROP_POS_FRAMES, 0);

    std::string target_dir = "/home/xiongchao/studying/SLAM/VSLAM/cubemap_slam/dataset/wuhan_avp_rear/";
    double timestamp = 1515653813.0;
    int count = 1;


    while (capture.read(im)) {
        std::string image_count;
        if (count < 10)
            image_count = "_0000000" + std::to_string(count) + ".tiff";
        else if (count < 100)
            image_count = "_000000" + std::to_string(count) + ".tiff";
        else if (count < 1000)
            image_count = "_00000" + std::to_string(count) + ".tiff";
        else if (count < 10000)
            image_count = "_0000" + std::to_string(count) + ".tiff";
        else if (count < 100000)
            image_count = "_000" + std::to_string(count) + ".tiff";
        else if (count < 1000000)
            image_count = "_00" + std::to_string(count) + ".tiff";

        std::string image_name = std::to_string(timestamp) + image_count;
        std::string image_path = target_dir + image_name;
        cv::imwrite(image_path, im);

        timestamp += double(1) / 3;
    }
}

void create_mask() {
    int width = 1920, height = 1080;
    cv::Mat mask = cv::Mat(cv::Size(width, height), CV_8UC1);
    mask = cv::Scalar(255);
    cv::imwrite("/home/xiongchao/studying/SLAM/VSLAM/cubemap_slam/dataset/wuhan_avp/rear_mask.png", mask);

    if (mask.type() == CV_8UC1) {
        std::cout << "创建成功" << std::endl;
    } else {
        std::cout << "创建失败" << std::endl;
    }

}

void resize_image() {
    std::string out_dir = "/home/xiongchao/文档/wuhan_avp/resize_image";
    std::string image_dir = "/home/xiongchao/文档/wuhan_avp/rear";
    std::string image_list_path = "/home/xiongchao/文档/wuhan_avp/rear_list.txt";
    std::ifstream ifs_image(image_list_path);

    std::vector<std::string> image_names;
    std::string line;
    while (getline(ifs_image, line)) {
        image_names.push_back(line);
    }
    ifs_image.close();

    for (auto &image_name: image_names) {
        std::string image_path = image_dir + "/" + image_name;
        cv::Mat image = cv::imread(image_path, cv::IMREAD_UNCHANGED);

        cv::resize(image, image, cv::Size(1280, 720));

        cv::imwrite(out_dir + "/" + image_name, image);
    }

}

int main(int argc, char* argv[]) {
    // create_image();
    // create_mask();

    resize_image();
    return 0;
}