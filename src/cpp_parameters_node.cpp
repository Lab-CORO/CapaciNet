#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>
// #include "curobo_msgs/srv/generate_rm.hpp"
#include <iostream>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include "json.hpp"
// #include "robot.h"
#include "./master_ik_data.h"
#include "../include/robot.h"
#include "../include/master_ik_data.h"
#include "../include/utils.h"
#include "curobo_msgs/srv/generate_rm.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ctime>
// #include "std_srvs/srv/set_bool.hpp"
using namespace std::chrono_literals;
using json = nlohmann::json;
class MinimalParam : public rclcpp::Node
{
public:
  MinimalParam()
  : Node("minimal_param_node")
  {
    this->declare_parameter("my_parameter", "world");
    this->service_ = this->create_service<curobo_msgs::srv::GenerateRM>("generate_rm", std::bind(&MinimalParam::timer_callback, this, std::placeholders::_1, std::placeholders::_2));
    // timer_ = this->create_wall_timer(
    //   1000ms, std::bind(&MinimalParam::timer_callback, this));
  }

  void timer_callback(std::shared_ptr<curobo_msgs::srv::GenerateRM::Request> request,
    std::shared_ptr<curobo_msgs::srv::GenerateRM::Response> response)
  {
    std::string my_param = this->get_parameter("my_parameter").as_string();

    RCLCPP_INFO(this->get_logger(), "Hello %s!", my_param.c_str());

    std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("my_parameter", "world")};
    this->set_parameters(all_new_parameters);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<curobo_msgs::srv::GenerateRM>::SharedPtr service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalParam>());
  rclcpp::shutdown();
  return 0;
}