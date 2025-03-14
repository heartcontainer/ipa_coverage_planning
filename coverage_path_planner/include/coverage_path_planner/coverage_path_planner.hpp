#ifndef COVERAGE_PATH_PLANNER_COVERAGE_PATH_PLANNER_HPP_
#define COVERAGE_PATH_PLANNER_COVERAGE_PATH_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#define COVERAGE_PATH_BENCHMARK 1
namespace coverage_path_planner
{

  class CoveragePathPlanner : public nav2_util::LifecycleNode
  {
  public:
    using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
    using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

    explicit CoveragePathPlanner(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    ~CoveragePathPlanner();

  protected:
    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
    nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
#if COVERAGE_PATH_BENCHMARK
    void timerCallback();
    void saveCoverageImage(float coverage_time);
    std::string getCurrentTimeString();
#endif

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp_action::Client<FollowWaypoints>::SharedPtr follow_waypoints_client_;
    bool coverage_in_progress_ = false;
    int current_waypoint_ = 0;

#if COVERAGE_PATH_BENCHMARK
    double resolution_ = 0.05;
    std::vector<double> origin_ = {0.0, 0.0, 0.0};
    double coverage_radius_ = 0.265;
    cv::Mat map_;
    std::vector<geometry_msgs::msg::TransformStamped> robot_coverage_poses_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
#endif
  };

} // namespace coverage_path_planner

#endif // COVERAGE_PATH_PLANNER_COVERAGE_PATH_PLANNER_HPP_
