/*!
 *****************************************************************
 * \file
 *
 * \note
 * Copyright (c) 2015 \n
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 * Project name: Care-O-bot
 * \note
 * ROS stack name: autopnp
 * \note
 * ROS package name: ipa_room_segmentation
 *
 * \author
 * Author: Florian Jordan
 * \author
 * Supervised by: Richard Bormann
 *
 * \date Date of creation: 08.2015
 *
 * \brief
 *
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer. \n
 * - Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution. \n
 * - Neither the name of the Fraunhofer Institute for Manufacturing
 * Engineering and Automation (IPA) nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

#include <rclcpp_action/rclcpp_action.hpp>
#include <ipa_building_msgs/action/map_segmentation.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "dynamic_reconfigure.h"

class RoomSegmentationClient : public rclcpp::Node
{
public:
    using MapSegmentation = ipa_building_msgs::action::MapSegmentation;
    using GoalHandleMapSegmentation = rclcpp_action::ClientGoalHandle<MapSegmentation>;

    explicit RoomSegmentationClient(const rclcpp::NodeOptions &options)
        : Node("room_segmentation_client", options)
    {
        this->client_ptr_ = rclcpp_action::create_client<MapSegmentation>(
            this,
            "/room_segmentation/room_segmentation_server");

        // parameter_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "/room_segmentation/room_segmentation_server");

        this->declare_parameter<std::string>("image_path", "");
        this->declare_parameter<int>("room_segmentation_algorithm", 3);
        this->get_parameter("image_path", this->image_path_);
        this->get_parameter("room_segmentation_algorithm", this->room_segmentation_algorithm_);

        if (this->image_path_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "No image path provided");
            rclcpp::shutdown();
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Using image: %s", this->image_path_.c_str());
            if (dynamic_reconfigure::update_parameters(this, "/room_segmentation/room_segmentation_server",
                                                       {rclcpp::Parameter("room_segmentation_algorithm", this->room_segmentation_algorithm_)}))
            {
                send_goal();
            }
        }
    }

    // void update_parameters()
    // {
    //     while (!parameter_client_->wait_for_service(std::chrono::seconds(2)))
    //     {
    //         RCLCPP_INFO(this->get_logger(), "Waiting for parameter service to become available...");
    //     }

    //     RCLCPP_INFO(this->get_logger(), "Parameter service is ready.");

    //     auto results = parameter_client_->set_parameters_atomically(
    //         {rclcpp::Parameter("room_segmentation_algorithm", this->room_segmentation_algorithm_)});

    //     rclcpp::spin_until_future_complete(
    //         this->get_node_base_interface(),
    //         results);
    //     RCLCPP_INFO(this->get_logger(), "Updated room_segmentation_algorithm to %d", this->room_segmentation_algorithm_);
    // }

    sensor_msgs::msg::Image create_image_message()
    {
        // import maps
        cv::Mat map = cv::imread(image_path_.c_str(), 0);

        // Make non-white pixels black and others white
        for (int y = 0; y < map.rows; ++y)
        {
            for (int x = 0; x < map.cols; ++x)
            {
                map.at<unsigned char>(y, x) = (map.at<unsigned char>(y, x) < 250) ? 0 : 255;
            }
        }

        // prepare image message
        sensor_msgs::msg::Image labeling;
        cv_bridge::CvImage cv_image;
        cv_image.encoding = "mono8";
        cv_image.image = map;
        cv_image.toImageMsg(labeling);
        return labeling;
    }

    void send_goal()
    {
        using namespace std::placeholders;

        if (!this->client_ptr_->wait_for_action_server())
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }

        // send a goal to the action
        auto goal = ipa_building_msgs::action::MapSegmentation::Goal();
        goal.input_map = create_image_message();
        goal.map_origin.position.x = 0;
        goal.map_origin.position.y = 0;
        goal.map_resolution = 0.05;
        goal.return_format_in_meter = false;
        goal.return_format_in_pixel = true;
        goal.robot_radius = 0.4;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<MapSegmentation>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&RoomSegmentationClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&RoomSegmentationClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&RoomSegmentationClient::result_callback, this, _1);
        this->client_ptr_->async_send_goal(goal, send_goal_options);
    }

private:
    rclcpp_action::Client<MapSegmentation>::SharedPtr client_ptr_;
    // std::shared_ptr<rclcpp::AsyncParametersClient> parameter_client_;
    std::string image_path_;
    int room_segmentation_algorithm_;

    void goal_response_callback(const GoalHandleMapSegmentation::SharedPtr &future)
    {
        if (!future)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(
        GoalHandleMapSegmentation::SharedPtr,
        const std::shared_ptr<const MapSegmentation::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Received feedback");
    }

    void result_callback(const rclcpp_action::ClientGoalHandle<ipa_building_msgs::action::MapSegmentation>::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal finished successfully!");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
        }

        // display
        cv_bridge::CvImagePtr cv_ptr_obj;
        cv_ptr_obj = cv_bridge::toCvCopy(result.result->segmented_map, sensor_msgs::image_encodings::TYPE_32SC1);
        cv::Mat segmented_map = cv_ptr_obj->image;
        cv::Mat colour_segmented_map = segmented_map.clone();
        colour_segmented_map.convertTo(colour_segmented_map, CV_8U);
        cv::cvtColor(colour_segmented_map, colour_segmented_map, CV_GRAY2BGR);
        for (size_t i = 1; i <= result.result->room_information_in_pixel.size(); ++i)
        {
            // choose random color for each room
            int blue = (rand() % 250) + 1;
            int green = (rand() % 250) + 1;
            int red = (rand() % 250) + 1;
            for (size_t u = 0; u < segmented_map.rows; ++u)
            {
                for (size_t v = 0; v < segmented_map.cols; ++v)
                {
                    if (segmented_map.at<int>(u, v) == i)
                    {
                        colour_segmented_map.at<cv::Vec3b>(u, v)[0] = blue;
                        colour_segmented_map.at<cv::Vec3b>(u, v)[1] = green;
                        colour_segmented_map.at<cv::Vec3b>(u, v)[2] = red;
                    }
                }
            }
        }
        // draw the room centers into the map
        for (size_t i = 0; i < result.result->room_information_in_pixel.size(); ++i)
        {
            cv::Point current_center(result.result->room_information_in_pixel[i].room_center.x, result.result->room_information_in_pixel[i].room_center.y);
#if CV_MAJOR_VERSION <= 3
            cv::circle(colour_segmented_map, current_center, 2, CV_RGB(0, 0, 255), CV_FILLED);
#else
            cv::circle(colour_segmented_map, current_center, 2, CV_RGB(0, 0, 255), cv::FILLED);
#endif
        }

        cv::imshow("segmentation", colour_segmented_map);
        cv::waitKey();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RoomSegmentationClient>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
