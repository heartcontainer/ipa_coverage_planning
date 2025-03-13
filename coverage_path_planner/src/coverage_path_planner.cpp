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
#if COVERAGE_PATH_BENCHMARK
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
#endif
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
    if (coverage_in_progress_)
    {
      RCLCPP_INFO(get_logger(), "Coverage in progress, canceling the goal...");
      // follow_waypoints_client_->async_cancel_all_goals();
      coverage_in_progress_ = false;
    }
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
    int pose_size = msg->poses.size();
    RCLCPP_INFO(get_logger(), "Received path with %d waypoints.", pose_size);

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
      coverage_in_progress_ = false;
    };
    current_waypoint_ = 0;
    send_goal_options.feedback_callback =
        [this, pose_size](GoalHandleFollowWaypoints::SharedPtr, const std::shared_ptr<const FollowWaypoints::Feedback> feedback)
    {
      if (current_waypoint_ < feedback->current_waypoint)
      {
        RCLCPP_INFO(this->get_logger(), "Received feedback: %d/%d", feedback->current_waypoint, pose_size);
        current_waypoint_ = feedback->current_waypoint;
      }
    };
    coverage_in_progress_ = true;
    follow_waypoints_client_->async_send_goal(*goal_msg, send_goal_options);
#if COVERAGE_PATH_BENCHMARK
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&CoveragePathPlanner::mapCallback, this, std::placeholders::_1));

    start_time_ = this->now();
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&CoveragePathPlanner::timerCallback, this));
#endif
  }

#if COVERAGE_PATH_BENCHMARK
  void CoveragePathPlanner::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    int width = msg->info.width;
    int height = msg->info.height;
    resolution_ = msg->info.resolution;
    origin_[0] = msg->info.origin.position.x;
    origin_[1] = msg->info.origin.position.y;
    origin_[2] = msg->info.origin.position.z;

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

    map_sub_.reset();
    RCLCPP_INFO(this->get_logger(), "Received map with width: %d, height: %d resolution: %f, origin: [%f, %f, %f]", width, height, resolution_, origin_[0], origin_[1], origin_[2]);
  }

  void CoveragePathPlanner::timerCallback()
  {
    if (coverage_in_progress_)
    {
      try
      {
        geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero, std::chrono::milliseconds(100));
        robot_coverage_poses_.push_back(transform_stamped);

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

        RCLCPP_INFO(this->get_logger(), "Robot pose[%ld]: x=%f, y=%f, z=%f, roll=%f, pitch=%f, yaw=%f", robot_coverage_poses_.size(), x, y, z, roll, pitch, yaw);
      }
      catch (const tf2::TransformException &ex)
      {
        RCLCPP_WARN(this->get_logger(), "Could not get robot pose: %s", ex.what());
      }
    }
    else
    {
      auto end_time = this->now();
      RCLCPP_INFO(this->get_logger(), "Stop watching robot coverage poses, total poses: %ld, time: %.2f min", robot_coverage_poses_.size(), (end_time - start_time_).seconds() / 60);
      saveCoverageImage();
      timer_->cancel();
      timer_.reset();
    }
  }

  void CoveragePathPlanner::saveCoverageImage()
  {
    if (!robot_coverage_poses_.empty())
    {
      const double inverse_map_resolution = 1. / resolution_;
      double scale_factor = 4.0;
      cv::Scalar red(0, 0, 255);

      cv::Mat rgb_image;
      cv::cvtColor(map_.clone(), rgb_image, cv::COLOR_GRAY2RGB);
      cv::Mat path_map;
      cv::resize(rgb_image, path_map, cv::Size(), scale_factor, scale_factor, cv::INTER_NEAREST);

      cv::Point current_point;

      for (size_t i = 0; i < robot_coverage_poses_.size(); i++)
      {
        current_point.x = (robot_coverage_poses_[i].transform.translation.x - origin_[0]) * inverse_map_resolution * scale_factor;
        current_point.y = (robot_coverage_poses_[i].transform.translation.y - origin_[1]) * inverse_map_resolution * scale_factor;

        cv::circle(path_map, current_point, coverage_radius_ * scale_factor / resolution_, red, -1);
      }

      std::string save_path = "coverage_path/" + getCurrentTimeString() + ".png";
      cv::imwrite(save_path, path_map);
      robot_coverage_poses_.clear();
      RCLCPP_INFO(this->get_logger(), "Saved the map to %s", save_path.c_str());
    }
  }

  std::string CoveragePathPlanner::getCurrentTimeString()
  {
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);

    std::tm tm{};
    localtime_r(&now_c, &tm);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S");

    return oss.str();
  }
#endif
} // namespace coverage_path_planner

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(coverage_path_planner::CoveragePathPlanner)
