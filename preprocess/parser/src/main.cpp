#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>
#include <unordered_set>
#include <sstream>
#include <iomanip>

// 修剪字符串两端的空白字符
std::string Trim(const std::string& str) {
    const auto str_begin = str.find_first_not_of(" \t");
    if (str_begin == std::string::npos) {
        return "";
    }

    const auto str_end = str.find_last_not_of(" \t");
    const auto str_range = str_end - str_begin + 1;

    return str.substr(str_begin, str_range);
}

// 读取姿态时间戳文件并生成所需的图片文件名
bool ReadDesiredImageFilenames(const std::string& pose_file,
                                std::unordered_set<std::string>* desired_images) {
    std::ifstream infile(pose_file);
    if (!infile.is_open()) {
        std::cerr << "无法打开姿态文件: " << pose_file << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(infile, line)) {
        line = Trim(line);
        if (line.empty()) {
            continue;
        }

        std::istringstream ss(line);
        double timestamp, x, y, z, qx, qy, qz, qw;
        ss >> timestamp >> x >> y >> z >> qx >> qy >> qz >> qw;

        long long int_timestamp = static_cast<long long>(timestamp * 1e6);
        std::string image_name = std::to_string(int_timestamp) + "000.jpg";
        desired_images->insert(image_name);
    }
    infile.close();
    std::cout << "需要筛选的图片总数: " << desired_images->size() << std::endl;
    return true;
}

// 创建输出目录
bool CreateOutputDirectory(const std::string& output_dir) {
    if (!boost::filesystem::exists(output_dir)) {
        if (!boost::filesystem::create_directories(output_dir)) {
            std::cerr << "无法创建输出目录: " << output_dir << std::endl;
            return false;
        }
    }
    return true;
}

// 保存符合条件的图片
size_t SaveFilteredImages(const std::string& bag_file,
                         const std::string& output_dir,
                         const std::unordered_set<std::string>& desired_images,
                         int sample_interval = 1) {
    size_t saved_count = 0;
    size_t processed = 0;
    size_t valid_count = 0;
    size_t total_desired = desired_images.size();  // 需要的总图片数

    try {
        rosbag::Bag bag;
        bag.open(bag_file, rosbag::bagmode::Read);

        std::vector<std::string> topics = {"/rs_camera/rgb"};
        rosbag::View view(bag, rosbag::TopicQuery(topics));

        size_t total_messages = view.size();
        std::cout << "rosbag中的消息总数: " << total_messages << std::endl;
        std::cout << "抽样间隔: 每" << sample_interval << "张有效图片保存一张" << std::endl;
        std::cout << "需要处理的有效图片总数: " << total_desired << std::endl;

        for (const auto& message_instance : view) {
            auto img_msg = message_instance.instantiate<sensor_msgs::Image>();
            if (img_msg) {
                try {
                    ros::Time timestamp = img_msg->header.stamp;
                    long long timestamp_us = static_cast<long long>(timestamp.toSec() * 1e6);
                    std::string filename = std::to_string(timestamp_us) + "000.jpg";

                    if (desired_images.find(filename) != desired_images.end()) {
                        valid_count++;
                        
                        if ((valid_count - 1) % sample_interval == 0) {
                            cv_bridge::CvImageConstPtr cv_ptr =
                                cv_bridge::toCvShare(img_msg, "bgr8");
                            const cv::Mat& image = cv_ptr->image;

                            std::string file_path = output_dir + "/" + filename;
                            if (cv::imwrite(file_path, image)) {
                                saved_count++;
                                std::cout << "保存图片: " << file_path 
                                        << " (第" << valid_count << "张有效图片)" << std::endl;
                            } else {
                                std::cerr << "保存失败: " << file_path << std::endl;
                            }
                        }

                        // 如果已经处理完所有需要的图片，提前退出
                        if (valid_count >= total_desired) {
                            std::cout << "已处理完所有需要的图片，提前结束处理" << std::endl;
                            break;
                        }
                    }

                    processed++;
                    if (processed % 100 == 0) {
                        std::cout << "处理进度: " << processed
                                << " / " << total_messages 
                                << " (有效图片: " << valid_count << "/" << total_desired
                                << ", 已保存: " << saved_count << ")" << std::endl;
                    }
                } catch (const std::exception& e) {
                    std::cerr << "处理图片消息失败: " << e.what() << std::endl;
                }
            }
        }

        bag.close();
    } catch (const std::exception& e) {
        std::cerr << "处理rosbag文件时出错: " << e.what() << std::endl;
    }

    return saved_count;
}

int main(int argc, char** argv) {
    if (argc < 4) {
        std::cerr << "用法: " << argv[0]
                << " <bag_file> <output_dir> <pose_file> [sample_interval]" << std::endl
                << "sample_interval: 可选，每隔多少张有效图片保存一张 (默认: 1)" << std::endl;
        return 1;
    }

    const std::string bag_file = argv[1];
    const std::string output_dir = argv[2];
    const std::string pose_file = argv[3];
    const int sample_interval = (argc > 4) ? std::stoi(argv[4]) : 1;

    if (sample_interval < 1) {
        std::cerr << "错误: 抽样间隔必须大于等于1" << std::endl;
        return 1;
    }

    std::cout << "输入的bag文件路径: " << bag_file << std::endl;
    std::cout << "输出目录路径: " << output_dir << std::endl;
    std::cout << "姿态时间戳文件: " << pose_file << std::endl;
    std::cout << "抽样间隔: " << sample_interval << std::endl;

    if (!CreateOutputDirectory(output_dir)) {
        return 1;
    }

    std::unordered_set<std::string> desired_images;
    if (!ReadDesiredImageFilenames(pose_file, &desired_images)) {
        return 1;
    }

    size_t saved_images = SaveFilteredImages(bag_file, output_dir, desired_images, sample_interval);
    std::cout << "提取完成! 保存的图片总数: " << saved_images << std::endl;

    return 0;
}
