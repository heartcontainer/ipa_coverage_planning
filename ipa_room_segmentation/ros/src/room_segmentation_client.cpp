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
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

#include <rclcpp_action/rclcpp_action.hpp>
#include <ipa_building_msgs/action/map_segmentation.hpp>
#include <sensor_msgs/msg/image.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("room_segmentation_client");

	// map names
    std::vector<std::string> map_names = {
        "lab_ipa", "lab_c_scan", "Freiburg52_scan", "Freiburg79_scan", "lab_b_scan",
        "lab_intel", "Freiburg101_scan", "lab_d_scan", "lab_f_scan", "lab_a_scan",
        "NLB", "office_a", "office_b", "office_c", "office_d", "office_e", "office_f",
        "office_g", "office_h", "office_i", "lab_ipa_furnitures", "lab_c_scan_furnitures",
        "Freiburg52_scan_furnitures", "Freiburg79_scan_furnitures", "lab_b_scan_furnitures",
        "lab_intel_furnitures", "Freiburg101_scan_furnitures", "lab_d_scan_furnitures",
        "lab_f_scan_furnitures", "lab_a_scan_furnitures", "NLB_furnitures", "office_a_furnitures",
        "office_b_furnitures", "office_c_furnitures", "office_d_furnitures", "office_e_furnitures",
        "office_f_furnitures", "office_g_furnitures", "office_h_furnitures", "office_i_furnitures"};

    for (const auto &map_name : map_names)
	{
		// import maps
        std::string image_filename = ament_index_cpp::get_package_share_directory("ipa_room_segmentation") + "/common/files/test_maps/" + map_name + ".png";
		cv::Mat map = cv::imread(image_filename.c_str(), 0);

        // make non-white pixels black
		for (int y = 0; y < map.rows; y++)
		{
			for (int x = 0; x < map.cols; x++)
			{
                // find not reachable regions and make them black
				if (map.at<unsigned char>(y, x) < 250)
				{
					map.at<unsigned char>(y, x) = 0;
				}
				//else make it white
				else
				{
					map.at<unsigned char>(y, x) = 255;
				}
			}
		}

        // prepare image message
        sensor_msgs::msg::Image labeling;
		cv_bridge::CvImage cv_image;
		cv_image.encoding = "mono8";
		cv_image.image = map;
		cv_image.toImageMsg(labeling);

        // Action client
        auto action_client = rclcpp_action::create_client<ipa_building_msgs::action::MapSegmentation>(
            node, "room_segmentation_server");

        RCLCPP_INFO(node->get_logger(), "Waiting for action server to start.");
		if (!action_client->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(node->get_logger(), "Action server not available after waiting.");
            return 1;
        }

        RCLCPP_INFO(node->get_logger(), "Action server started, sending goal.");

        // TODO: test dynamic reconfigure
		// DynamicReconfigureClient drc(nh, "room_segmentation_server/set_parameters", "room_segmentation_server/parameter_updates");
		// drc.setConfig("room_segmentation_algorithm", 5);

		// send a goal to the action
        auto goal = ipa_building_msgs::action::MapSegmentation::Goal();
		goal.input_map = labeling;
		goal.map_origin.position.x = 0;
		goal.map_origin.position.y = 0;
		goal.map_resolution = 0.05;
		goal.return_format_in_meter = false;
		goal.return_format_in_pixel = true;
		goal.robot_radius = 0.4;

        // send goal to action server
        auto send_goal_options = rclcpp_action::Client<ipa_building_msgs::action::MapSegmentation>::SendGoalOptions();
        send_goal_options.result_callback = [](const rclcpp_action::ClientGoalHandle<ipa_building_msgs::action::MapSegmentation>::WrappedResult &result)
        {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(rclcpp::get_logger("room_segmentation_client"), "Finished successfully!");
                // Process the result here
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
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("room_segmentation_client"), "Action failed.");
            }
        };

        action_client->async_send_goal(goal, send_goal_options);

        // keep the node spinning to process the result
        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
	return 0;
}
