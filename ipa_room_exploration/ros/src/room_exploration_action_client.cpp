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
        this->declare_parameter<int>("room_exploration_algorithm", 8);

        this->get_parameter("room_exploration_algorithm", room_exploration_algorithm_);

        if (start_pos_.size() != 3)
        {
            RCLCPP_FATAL(this->get_logger(), "starting_position must contain 3 values");
            rclcpp::shutdown();
            return;
        }

        std::string image_path;
        if (use_test_maps_)
        {
            const std::string test_map_path = ament_index_cpp::get_package_share_directory("ipa_room_segmentation") + "/common/files/test_maps/";
            image_path = test_map_path + "lab_ipa.png";
        }
        else
        {
            std::string env_pack_path = this->declare_parameter<std::string>("env_pack", "ipa_room_segmentation");
            std::string file_name = this->declare_parameter<std::string>("image", "map.pgm");
            std::string map_name = this->declare_parameter<std::string>("robot_env", "lab_ipa");
            image_path = env_pack_path + "/envs/" + map_name + "/" + file_name;
        }

        // Load and process map
        cv::Mat map_flipped = cv::imread(image_path, 0);

        cv::flip(map_flipped, map_, 0);
        for (int y = 0; y < map_.rows; y++)
        {
            for (int x = 0; x < map_.cols; x++)
            {
                map_.at<unsigned char>(y, x) = (map_.at<unsigned char>(y, x) < 250) ? 0 : 255;
            }
        }
        RCLCPP_INFO(this->get_logger(), "Map size: %dx%d", map_.rows, map_.cols);

        if (!dynamic_reconfigure::update_parameters(this, "/room_exploration/room_exploration_server", {rclcpp::Parameter("room_exploration_algorithm", room_exploration_algorithm_), rclcpp::Parameter("execute_path", false)}))
        {
            rclcpp::shutdown();
            return;
        }

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
        fov_points[0].x = 0.04035; // this field of view represents the off-center iMop floor wiping device
        fov_points[0].y = -0.136;
        fov_points[1].x = 0.04035;
        fov_points[1].y = 0.364;
        fov_points[2].x = 0.54035; // todo: this definition is mirrored on x (y-coordinates are inverted) to work properly --> check why, make it work the intuitive way
        fov_points[2].y = 0.364;
        fov_points[3].x = 0.54035;
        fov_points[3].y = -0.136;
        int planning_mode = 2;
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
        //	int planning_mode = 1;	// footprint planning

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
        goal.robot_radius = robot_radius_; // turtlebot, used for sim 0.177, 0.4
        goal.coverage_radius = coverage_radius_;
        goal.field_of_view = fov_points;
        goal.field_of_view_origin = fov_origin;
        goal.starting_position = starting_position;
        goal.planning_mode = planning_mode;

        RCLCPP_INFO(this->get_logger(), "Sending goal...");

        auto send_goal_options = rclcpp_action::Client<RoomExplorationAction>::SendGoalOptions();
        send_goal_options.result_callback = [this](const GoalHandleRoomExploration::WrappedResult &result)
        {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(this->get_logger(), "Goal succeeded with result: Got a path with %lu nodes", result.result->coverage_path.size());
                // display path
                const double inverse_map_resolution = 1. / resolution_;
                cv::Mat path_map = map_.clone();
                for (size_t point = 0; point < result.result->coverage_path.size(); ++point)
                {
                    const cv::Point point1((result.result->coverage_path[point].x - origin_[0]) * inverse_map_resolution, (result.result->coverage_path[point].y - origin_[1]) * inverse_map_resolution);
                    cv::circle(path_map, point1, 2, cv::Scalar(128), -1);
                    if (point > 0)
                    {
                        const cv::Point point2((result.result->coverage_path[point - 1].x - origin_[0]) * inverse_map_resolution, (result.result->coverage_path[point - 1].y - origin_[1]) * inverse_map_resolution);
                        cv::line(path_map, point1, point2, cv::Scalar(128), 1);
                    }
                    // std::cout << "coverage_path[" << point << "]: x=" << result.result->coverage_path[point].x << ", y=" << result.result->coverage_path[point].y << ", theta=" << result.result->coverage_path[point].theta << std::endl;
                }
                cv::imshow("path", path_map);
                cv::waitKey();
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Goal failed with code: %d", static_cast<int>(result.code));
            }
            rclcpp::shutdown();
        };

        action_client_->async_send_goal(goal, send_goal_options);
    }

private:
    rclcpp_action::Client<RoomExplorationAction>::SharedPtr action_client_;
    int room_exploration_algorithm_;
    bool use_test_maps_ = true;
    double resolution_ = 0.05;
    std::vector<double> origin_ = {0.0, 0.0, 0.0};
    double robot_radius_ = 0.3;
    double coverage_radius_ = 1.0;
    std::vector<double> start_pos_ = {0.0, 0.0, 0.0};
    cv::Mat map_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RoomExplorationClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
