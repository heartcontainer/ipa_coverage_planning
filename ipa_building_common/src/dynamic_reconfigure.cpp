#include "dynamic_reconfigure.h"

namespace dynamic_reconfigure
{
    bool update_parameters(rclcpp::Node *node, const std::string &remote_node_name, const std::vector<rclcpp::Parameter> &parameters)
    {
        auto client = std::make_shared<rclcpp::AsyncParametersClient>(node, remote_node_name);

        if (!client->wait_for_service(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(node->get_logger(), "Parameter service for '%s' is not available after waiting.", remote_node_name.c_str());
            return false;
        }

        RCLCPP_INFO(node->get_logger(), "Parameter service for '%s' is ready.", remote_node_name.c_str());

        auto future_result = client->set_parameters_atomically(parameters);
        auto result = rclcpp::spin_until_future_complete(node->get_node_base_interface(), future_result);

        if (result == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future_result.get();
            if (response.successful)
            {
                RCLCPP_INFO(node->get_logger(), "Parameters successfully updated on '%s'.", remote_node_name.c_str());
                return true;
            }
            else
            {
                RCLCPP_ERROR(node->get_logger(), "Failed to update parameters on '%s': %s", remote_node_name.c_str(), response.reason.c_str());
                return false;
            }
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to update parameters on '%s': timeout or interrupted.", remote_node_name.c_str());
            return false;
        }
    }
}