#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::placeholders;

namespace cb_group_demo
{
class ServiceNode : public rclcpp::Node
{
public:
    ServiceNode() : Node("service_node")
    {
        auto service_callback = [this](
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<std_srvs::srv::Empty::Request> request,
            const std::shared_ptr<std_srvs::srv::Empty::Response> response)
        {
            (void)request_header;
            (void)request;
            (void)response;
            RCLCPP_INFO(this->get_logger(), "Received request, responding...");
        };
        service_ptr_ = this->create_service<std_srvs::srv::Empty>(
                "test_service",
                service_callback
        );
    }

private:
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_ptr_;

};  // class ServiceNode
}   // namespace cb_group_demo

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto service_node = std::make_shared<cb_group_demo::ServiceNode>();

    RCLCPP_INFO(service_node->get_logger(), "Starting server node, shut down with CTRL-C");
    rclcpp::spin(service_node);
    RCLCPP_INFO(service_node->get_logger(), "Keyboard interrupt, shutting down.\n");

    rclcpp::shutdown();
    return 0;
}