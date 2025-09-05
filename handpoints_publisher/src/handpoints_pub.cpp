#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "handpoints_publisher/msg/hand_keypoints.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std::chrono_literals;

class HandpointsPublisher : public rclcpp::Node
{
public:
    HandpointsPublisher(const std::string & csv_path, double publish_hz = 30.0)
    : Node("handpoints_pub"), publish_hz_(publish_hz)
    {
        publisher_ = this->create_publisher<handpoints_publisher::msg::HandKeypoints>(
            "Lefthandpoints", 10);

        // 读取 CSV
        if (!loadCSV(csv_path)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load CSV file: %s", csv_path.c_str());
            throw std::runtime_error("Failed to load CSV file");
        }

        RCLCPP_INFO(this->get_logger(), "CSV file loaded successfully: %s", csv_path.c_str());

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(int(1000.0 / publish_hz_)),
            std::bind(&HandpointsPublisher::publishFrame, this));
    }

private:
    bool loadCSV(const std::string & path)
    {
        std::ifstream file(path);
        if (!file.is_open()) return false;

        std::string line;
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string item;
            std::vector<double> points_row;
            try {
                while (std::getline(ss, item, ',')) {
                    points_row.push_back(std::stod(item));
                }
            } catch (const std::exception & e) {
                RCLCPP_WARN(this->get_logger(), "Error parsing line, skipping: %s", e.what());
                continue;
            }

            if (points_row.size() != 63) {
                RCLCPP_WARN(this->get_logger(), "CSV line does not have 63 values, skipping.");
                continue;
            }

            std::array<geometry_msgs::msg::Point, 21> frame_points;
            for (size_t i = 0; i < 21; ++i) {
                frame_points[i].x = points_row[i*3 + 0];
                frame_points[i].y = points_row[i*3 + 1];
                frame_points[i].z = points_row[i*3 + 2];
            }

            frames_.push_back(frame_points);
        }

        RCLCPP_INFO(this->get_logger(), "Loaded %zu frames from CSV.", frames_.size());
        return !frames_.empty();
    }

    void publishFrame()
    {
        if (frames_.empty()) return;

        handpoints_publisher::msg::HandKeypoints msg;
        msg.points = frames_[current_frame_];

        publisher_->publish(msg);

        // 循环播放
        current_frame_ = (current_frame_ + 1) % frames_.size();
    }

    rclcpp::Publisher<handpoints_publisher::msg::HandKeypoints>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    double publish_hz_;
    std::vector<std::array<geometry_msgs::msg::Point, 21>> frames_;
    size_t current_frame_ = 0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    std::string csv_path;
    try {
        csv_path = ament_index_cpp::get_package_share_directory("handpoints_publisher") + "/data/hand_3d_points.csv";
    } catch (const std::exception & e) {
        std::cerr << "Cannot find package share directory: " << e.what() << std::endl;
        return 1;
    }

    auto node = std::make_shared<HandpointsPublisher>(csv_path, 30.0);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
