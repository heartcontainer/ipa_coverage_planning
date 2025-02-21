#include "coverage_path_planner/coverage_path_planner.hpp"

namespace coverage_path_planner
{

  using rcl_interfaces::msg::ParameterType;
  using std::placeholders::_1;

  CoveragePathPlanner::CoveragePathPlanner(const rclcpp::NodeOptions &options)
      : nav2_util::LifecycleNode("coverage_path_planner", "", options)
  {
    RCLCPP_INFO(get_logger(), "Creating");
  }

  CoveragePathPlanner::~CoveragePathPlanner()
  {
  }

  nav2_util::CallbackReturn
  CoveragePathPlanner::on_configure(const rclcpp_lifecycle::State & /*state*/)
  {
    RCLCPP_INFO(get_logger(), "Configuring");

    follow_waypoints_client_ = rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints");
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        "/room_exploration/coverage_path", 2,
        std::bind(&CoveragePathPlanner::pathCallback, this, _1));

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  CoveragePathPlanner::on_activate(const rclcpp_lifecycle::State & /*state*/)
  {
    RCLCPP_INFO(get_logger(), "Activating");
    // create bond connection
    createBond();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  CoveragePathPlanner::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
  {
    RCLCPP_INFO(get_logger(), "Deactivating");
    // destroy bond connection
    destroyBond();

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  CoveragePathPlanner::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
  {
    RCLCPP_INFO(get_logger(), "Cleaning up");
    path_sub_.reset();
    follow_waypoints_client_.reset();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  CoveragePathPlanner::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
  {
    RCLCPP_INFO(get_logger(), "Shutting down");
    return nav2_util::CallbackReturn::SUCCESS;
  }

  void CoveragePathPlanner::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "Received path with %d waypoints.", static_cast<int>(msg->poses.size()));

    while (!follow_waypoints_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
      RCLCPP_INFO(this->get_logger(), "Waiting for 'follow_waypoints' Action Server...");
    }

    auto goal_msg = std::make_shared<FollowWaypoints::Goal>();
    goal_msg->poses.reserve(msg->poses.size());

    for (const auto &pose_stamped : msg->poses)
    {
      goal_msg->poses.push_back(pose_stamped);
    }

    RCLCPP_INFO(this->get_logger(), "Sending goal to follow waypoints action server...");

    auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
    send_goal_options.result_callback =
        [this](const GoalHandleFollowWaypoints::WrappedResult &result)
    {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
      {
        RCLCPP_INFO(this->get_logger(), "success");
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "fail");
      }
    };

    follow_waypoints_client_->async_send_goal(*goal_msg, send_goal_options);
  }

} // namespace coverage_path_planner

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(coverage_path_planner::CoveragePathPlanner)
