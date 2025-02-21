#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ipa_building_msgs/action/room_exploration.hpp>
#include "dynamic_reconfigure.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

// Overload of << operator for geometry_msgs::msg::Pose2D to wanted format
std::ostream &operator<<(std::ostream &os, const geometry_msgs::msg::Pose2D &obj)
{
    std::stringstream ss;
    ss << "[" << obj.x << ", " << obj.y << ", " << obj.theta << "]";
    os << ss.rdbuf();
    return os;
}

class RoomExplorationClient : public rclcpp::Node
{
public:
    using RoomExplorationAction = ipa_building_msgs::action::RoomExploration;
    using GoalHandleRoomExploration = rclcpp_action::ClientGoalHandle<RoomExplorationAction>;

    RoomExplorationClient() : Node("room_exploration_client")
    {
        this->declare_parameter<std::string>("image_path", "");
        this->declare_parameter<bool>("save_exploration_map", false);
        this->declare_parameter<int>("room_exploration_algorithm", 8);

        this->get_parameter("image_path", image_path_);
        this->get_parameter("save_exploration_map", save_exploration_map_);
        this->get_parameter("room_exploration_algorithm", room_exploration_algorithm_);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        get_robot_pose();
        if (start_pos_.size() != 3)
        {
            RCLCPP_FATAL(this->get_logger(), "starting_position must contain 3 values");
            rclcpp::shutdown();
            return;
        }

        if (!dynamic_reconfigure::update_parameters(this, "/room_exploration/room_exploration_server",
                                                    {rclcpp::Parameter("room_exploration_algorithm", room_exploration_algorithm_)}))
        {
            rclcpp::shutdown();
            return;
        }

        if (image_path_ == "")
        {
            map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
                "/map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
                std::bind(&RoomExplorationClient::mapCallback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "Subscribed to /map topic");
        }
        else
        {
            loadMap();
        }
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        int width = msg->info.width;
        int height = msg->info.height;
        resolution_ = msg->info.resolution;
        origin_[0] = msg->info.origin.position.x;
        origin_[1] = msg->info.origin.position.y;
        origin_[2] = msg->info.origin.position.z;
        RCLCPP_INFO(this->get_logger(), "Received map with width: %d, height: %d resolution: %f, origin: [%f, %f, %f]", width, height, resolution_, origin_[0], origin_[1], origin_[2]);

        map_ = cv::Mat(height, width, CV_8UC1);

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                int index = y * width + x;
                int8_t value = msg->data[index];

                if (value == -1)
                    map_.at<uchar>(y, x) = 127;
                else if (value == 0)
                    map_.at<uchar>(y, x) = 255;
                else
                    map_.at<uchar>(y, x) = 0;
            }
        }

        sendGoal();
    }

    void loadMap()
    {
        // Load and process map
        RCLCPP_INFO(this->get_logger(), "Loading map from: %s", image_path_.c_str());
        cv::Mat map_flipped = cv::imread(image_path_, 0);

        cv::flip(map_flipped, map_, 0);
        for (int y = 0; y < map_.rows; y++)
        {
            for (int x = 0; x < map_.cols; x++)
            {
                map_.at<unsigned char>(y, x) = (map_.at<unsigned char>(y, x) < 250) ? 0 : 255;
            }
        }
        RCLCPP_INFO(this->get_logger(), "Map size: %dx%d", map_.rows, map_.cols);
        sendGoal();
    }

    void sendGoal()
    {
        // Convert map to sensor_msgs/Image
        sensor_msgs::msg::Image labeling;
        cv_bridge::CvImage cv_image;
        cv_image.encoding = "mono8";
        cv_image.image = map_;
        cv_image.toImageMsg(labeling);

        // Initialize goal
        geometry_msgs::msg::Pose map_origin;
        map_origin.position.x = origin_[0];
        map_origin.position.y = origin_[1];
        map_origin.position.z = origin_[2];

        geometry_msgs::msg::Pose2D starting_position;
        starting_position.x = start_pos_[0];
        starting_position.y = start_pos_[1];
        starting_position.theta = start_pos_[2];

        std::vector<geometry_msgs::msg::Point32> fov_points(4);
        // fov_points[0].x = 0.04035; // this field of view represents the off-center iMop floor wiping device
        // fov_points[0].y = -0.136;
        // fov_points[1].x = 0.04035;
        // fov_points[1].y = 0.364;
        // fov_points[2].x = 0.54035; // todo: this definition is mirrored on x (y-coordinates are inverted) to work properly --> check why, make it work the intuitive way
        // fov_points[2].y = 0.364;
        // fov_points[3].x = 0.54035;
        // fov_points[3].y = -0.136;
        // int planning_mode = 2;
        //	fov_points[0].x = 0.15;		// this field of view fits a Asus Xtion sensor mounted at 0.63m height (camera center) pointing downwards to the ground in a respective angle
        //	fov_points[0].y = 0.35;
        //	fov_points[1].x = 0.15;
        //	fov_points[1].y = -0.35;
        //	fov_points[2].x = 1.15;
        //	fov_points[2].y = -0.65;
        //	fov_points[3].x = 1.15;
        //	fov_points[3].y = 0.65;
        //	int planning_mode = 2;	// viewpoint planning
        //	fov_points[0].x = -0.3;		// this is the working area of a vacuum cleaner with 60 cm width
        //	fov_points[0].y = 0.3;
        //	fov_points[1].x = -0.3;
        //	fov_points[1].y = -0.3;
        //	fov_points[2].x = 0.3;
        //	fov_points[2].y = -0.3;
        //	fov_points[3].x = 0.3;
        //	fov_points[3].y = 0.3;
        int planning_mode = 1; // footprint planning

        geometry_msgs::msg::Point32 fov_origin;
        fov_origin.x = 0.0;
        fov_origin.y = 0.0;

        // Create action client
        action_client_ = rclcpp_action::create_client<RoomExplorationAction>(this, "room_exploration_server");

        if (!action_client_->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
            return;
        }

        auto goal = RoomExplorationAction::Goal();
        goal.input_map = labeling;
        goal.map_resolution = resolution_;
        goal.map_origin = map_origin;
        goal.robot_radius = robot_radius_;
        goal.coverage_radius = coverage_radius_;
        goal.field_of_view = fov_points;
        goal.field_of_view_origin = fov_origin;
        goal.starting_position = starting_position;
        goal.planning_mode = planning_mode;

        RCLCPP_INFO(this->get_logger(), "Sending goal...");

        auto send_goal_options = rclcpp_action::Client<RoomExplorationAction>::SendGoalOptions();
        send_goal_options.result_callback = std::bind(&RoomExplorationClient::result_callback, this, std::placeholders::_1);
        action_client_->async_send_goal(goal, send_goal_options);
    }

    void result_callback(
        const GoalHandleRoomExploration::WrappedResult &result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(this->get_logger(), "Goal succeeded with result: Got a path with %lu nodes", result.result->coverage_path.size());
            // display path
            const double inverse_map_resolution = 1. / resolution_;
            double scale_factor = 4.0;
            cv::Scalar red(0, 0, 255), green(0, 255, 0), blue(255, 0, 0), grey(128, 128, 128);

            cv::Mat rgb_image;
            cv::cvtColor(map_.clone(), rgb_image, cv::COLOR_GRAY2RGB);
            cv::Mat path_map;
            cv::resize(rgb_image, path_map, cv::Size(), scale_factor, scale_factor, cv::INTER_NEAREST);
            // draw start position
            cv::Point start_point((start_pos_[0] - origin_[0]) * inverse_map_resolution * scale_factor, (start_pos_[1] - origin_[1]) * inverse_map_resolution * scale_factor);
            cv::circle(path_map, start_point, 5.0, red, -1);

            cv::Point last_point, current_point;

            for (size_t i = 0; i < result.result->coverage_path.size(); i++)
            {
                current_point.x = (result.result->coverage_path[i].x - origin_[0]) * inverse_map_resolution * scale_factor;
                current_point.y = (result.result->coverage_path[i].y - origin_[1]) * inverse_map_resolution * scale_factor;
                if (i == 0)
                {
                    cv::circle(path_map, current_point, 5.0, green, -1);
                }
                else if (i == result.result->coverage_path.size() - 1)
                {
                    cv::circle(path_map, current_point, 5.0, blue, -1);
                }
                else
                {
                    cv::circle(path_map, current_point, 2.0, grey, -1);
                }
                if (i > 0)
                {
                    cv::line(path_map, last_point, current_point, grey, 1);
                }
                last_point = current_point;
                // std::cout << "coverage_path[" << i << "]: x=" << result.result->coverage_path[i].x << ", y=" << result.result->coverage_path[i].y << ", theta=" << result.result->coverage_path[i].theta << std::endl;
            }
            if (save_exploration_map_)
            {
                std::string save_path = "room_exploration/" + std::to_string(room_exploration_algorithm_) + ".png";
                cv::imwrite(save_path, path_map);
                RCLCPP_INFO(this->get_logger(), "Saved the map to %s", save_path.c_str());
            }
            else
            {
                cv::imshow("path", path_map);
                cv::waitKey();
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Goal failed with code: %d", static_cast<int>(result.code));
        }
        rclcpp::shutdown();
    }

    void get_robot_pose()
    {
        std::string tf_error;

        while (!tf_buffer_->canTransform("map", "base_link", tf2::TimePointZero, &tf_error))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for transforms...");
            tf_error.clear();
            sleep(1);
        }
        RCLCPP_INFO(this->get_logger(), "Transforms are available now!");

        try
        {
            geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero, std::chrono::milliseconds(100));

            double x = transform_stamped.transform.translation.x;
            double y = transform_stamped.transform.translation.y;
            double z = transform_stamped.transform.translation.z;

            tf2::Quaternion q(
                transform_stamped.transform.rotation.x,
                transform_stamped.transform.rotation.y,
                transform_stamped.transform.rotation.z,
                transform_stamped.transform.rotation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            RCLCPP_INFO(this->get_logger(), "Robot pose: x=%f, y=%f, z=%f, roll=%f, pitch=%f, yaw=%f", x, y, z, roll, pitch, yaw);

            start_pos_[0] = x;
            start_pos_[1] = y;
            start_pos_[2] = yaw;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not get robot pose: %s", ex.what());
        }
    }

private:
    rclcpp_action::Client<RoomExplorationAction>::SharedPtr action_client_;
    std::string image_path_;
    bool save_exploration_map_;
    int room_exploration_algorithm_;
    bool use_test_maps_ = true;
    double resolution_ = 0.05;
    std::vector<double> origin_ = {0.0, 0.0, 0.0};
    double robot_radius_ = 0.265;
    double coverage_radius_ = 0.265;
    std::vector<double> start_pos_ = {0.0, 0.0, 0.0};
    cv::Mat map_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RoomExplorationClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
