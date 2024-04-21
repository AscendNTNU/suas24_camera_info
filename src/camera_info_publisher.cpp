#include <chrono>
#include <iostream>
#include <functional>
#include <memory>
#include <string>
#include <yaml-cpp/yaml.h>

#include "sensor_msgs/msg/camera_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class TestCameraInfoPublisher : public rclcpp::Node
{
    public:
        TestCameraInfoPublisher()
        : Node("test_camera_info_publisher"), count_(0)
        {
            //publish
            publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("perception/camera_info", 10);
            timer_ = this->create_wall_timer(500ms, std::bind(&TestCameraInfoPublisher::info_callback, this));
        }

        void info_callback(){
            //init camera_info message
            auto message = sensor_msgs::msg::CameraInfo();

            //parse the YAML file with intrinsics 
            YAML::Node config = YAML::LoadFile("src/suas24_camera_info/src/20240416_arducam_intrinsics.yaml");
            auto K_temp = config["camera_matrix"]["data"];
            auto D_temp = config["distortion_coefficients"]["data"];
            auto R_temp = config["rectification_matrix"]["data"];
            auto P_temp = config["projection_matrix"]["data"];

            message.height = config["image_height"].as<float>();
            message.width = config["image_width"].as<float>();
            message.distortion_model = config["distortion_model"].as<std::string>();

            int idx = 0;
            for (YAML::const_iterator it = K_temp.begin(); it != K_temp.end(); ++it){
                message.k[idx] = it->as<float>();
                idx += 1;
            }

            idx = 0;
            std::vector<double> d;
            for (YAML::const_iterator it = D_temp.begin(); it != D_temp.end(); ++it){
                d.push_back(it->as<double>());
                idx += 1;
            }
            message.d = d;

            idx = 0;
            for (YAML::const_iterator it = R_temp.begin(); it != R_temp.end(); ++it){
                message.r[idx] = it->as<float>();
                idx += 1;
            }

            idx = 0;
            for (YAML::const_iterator it = P_temp.begin(); it != P_temp.end(); ++it){
                message.p[idx] = it->as<float>();
                idx += 1;
            }

            //RCLCPP_INFO(this->get_logger(), "Publishing");
            publisher_->publish(message);
        }

    private:
        //init
        size_t count_;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_; 
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestCameraInfoPublisher>());
  rclcpp::shutdown();
  return 0;
}
