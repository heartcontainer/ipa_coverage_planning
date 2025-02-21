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

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp_action::Client<FollowWaypoints>::SharedPtr follow_waypoints_client_;
  };

} // namespace coverage_path_planner

#endif // COVERAGE_PATH_PLANNER_COVERAGE_PATH_PLANNER_HPP_
