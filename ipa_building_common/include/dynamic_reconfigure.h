#ifndef DYNAMIC_RECONFIGURE_H_
#define DYNAMIC_RECONFIGURE_H_

#include <rclcpp/rclcpp.hpp>
#include <type_traits>
#include <iostream>

namespace dynamic_reconfigure
{
    template <typename T>
    bool reset_value_if_name_equals(std::string name, T *value, const rclcpp::Parameter *param)
    {
        if (param->get_name() == name)
        {
            if constexpr (std::is_same_v<T, int>)
            {
                *value = param->as_int();
            }
            else if constexpr (std::is_same_v<T, double>)
            {
                *value = param->as_double();
            }
            else if constexpr (std::is_same_v<T, std::string>)
            {
                *value = param->as_string();
            }
            else if constexpr (std::is_same_v<T, bool>)
            {
                *value = param->as_bool();
            }
            else
            {
                std::cout << "Unsupported type for parameter: " << name.c_str() << std::endl;
                return false;
            }

            std::cout << "Updated " << name.c_str() << " to: " << *value << std::endl;
            return true;
        }
        return false;
    }

    // client
    bool update_parameters(rclcpp::Node *node, const std::string &remote_node_name, const std::vector<rclcpp::Parameter> &parameters);
}
#endif // DYNAMIC_RECONFIGURE_H_