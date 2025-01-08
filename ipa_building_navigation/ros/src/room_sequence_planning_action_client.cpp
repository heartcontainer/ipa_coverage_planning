#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ipa_building_msgs/action/map_segmentation.hpp>
#include <ipa_building_msgs/action/find_room_sequence_with_checkpoints.hpp>

static void print_result(rclcpp::Node &node, const std::string &tag, const rclcpp_action::ResultCode &code)
{
	switch (code)
	{
	case rclcpp_action::ResultCode::SUCCEEDED:
		RCLCPP_INFO(node.get_logger(), "Goal(%s) finished successfully!", tag.c_str());
		break;
	case rclcpp_action::ResultCode::ABORTED:
		RCLCPP_ERROR(node.get_logger(), "Goal(%s) was aborted", tag.c_str());
		return;
	case rclcpp_action::ResultCode::CANCELED:
		RCLCPP_ERROR(node.get_logger(), "Goal(%s) was canceled", tag.c_str());
		return;
	default:
		RCLCPP_ERROR(node.get_logger(), "Goal(%s) unknown result code", tag.c_str());
		return;
	}
}

class RoomSequencePlanningClient : public rclcpp::Node
{
public:
	using MapSegmentation = ipa_building_msgs::action::MapSegmentation;
	using GoalHandleMapSegmentation = rclcpp_action::ClientGoalHandle<MapSegmentation>;
	using FindRoomSequenceWithCheckpoints = ipa_building_msgs::action::FindRoomSequenceWithCheckpoints;
	using GoalHandleFindRoomSequenceWithCheckpoints = rclcpp_action::ClientGoalHandle<FindRoomSequenceWithCheckpoints>;

	explicit RoomSequencePlanningClient(const rclcpp::NodeOptions &options)
		: Node("room_sequence_planning_client", options)
	{
		this->map_segmentation_client_ptr_ = rclcpp_action::create_client<MapSegmentation>(
			this,
			"/room_segmentation/room_segmentation_server");

		this->sequence_planning_client_ptr_ = rclcpp_action::create_client<FindRoomSequenceWithCheckpoints>(
			this,
			"/room_segmentation/room_sequence_planning_server");

		segmentation_parameter_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "/room_segmentation/room_segmentation_server");
		sequence_planning_parameter_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "/room_segmentation/room_sequence_planning_server");

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
			RCLCPP_INFO(this->get_logger(), "Using image: %s", image_path_.c_str());
			// import maps
			map_ = cv::imread(image_path_.c_str(), 0);

			// Make non-white pixels black and others white
			for (int y = 0; y < map_.rows; ++y)
			{
				for (int x = 0; x < map_.cols; ++x)
				{
					map_.at<unsigned char>(y, x) = (map_.at<unsigned char>(y, x) < 250) ? 0 : 255;
				}
			}
			update_segmentation_parameters();
			send_segmentation_goal();
		}
	}

	void update_segmentation_parameters()
	{
		while (!segmentation_parameter_client_->wait_for_service(std::chrono::seconds(2)))
		{
			RCLCPP_INFO(this->get_logger(), "Waiting for parameter service to become available...");
		}

		RCLCPP_INFO(this->get_logger(), "Parameter service is ready.");

		auto results = segmentation_parameter_client_->set_parameters_atomically(
			{rclcpp::Parameter("room_segmentation_algorithm", this->room_segmentation_algorithm_)});

		rclcpp::spin_until_future_complete(
			this->get_node_base_interface(),
			results);
		RCLCPP_INFO(this->get_logger(), "Updated room_segmentation_algorithm to %d", this->room_segmentation_algorithm_);
	}

	void update_sequence_planning_parameters()
	{
		while (!sequence_planning_parameter_client_->wait_for_service(std::chrono::seconds(2)))
		{
			RCLCPP_INFO(this->get_logger(), "Waiting for parameter service to become available...");
		}

		RCLCPP_INFO(this->get_logger(), "Parameter service is ready.");

		auto results = sequence_planning_parameter_client_->set_parameters_atomically(
			{rclcpp::Parameter("tsp_solver", 3),
			 rclcpp::Parameter("problem_setting", 2),
			 rclcpp::Parameter("planning_method", 1),
			 rclcpp::Parameter("max_clique_path_length", 12.0),
			 rclcpp::Parameter("maximum_clique_size", 9001),
			 rclcpp::Parameter("map_downsampling_factor", 0.25),
			 rclcpp::Parameter("check_accessibility_of_rooms", true),
			 rclcpp::Parameter("return_sequence_map", true),
			 rclcpp::Parameter("display_map", true)});

		rclcpp::spin_until_future_complete(
			this->get_node_base_interface(),
			results);
		RCLCPP_INFO(this->get_logger(), "Updated room_segmentation_algorithm to %d", this->room_segmentation_algorithm_);
	}

	sensor_msgs::msg::Image create_image_message()
	{
		// prepare image message
		sensor_msgs::msg::Image labeling;
		cv_bridge::CvImage cv_image;
		cv_image.encoding = "mono8";
		cv_image.image = map_;
		cv_image.toImageMsg(labeling);
		return labeling;
	}

	void send_segmentation_goal()
	{
		using namespace std::placeholders;

		if (!this->map_segmentation_client_ptr_->wait_for_action_server())
		{
			RCLCPP_ERROR(this->get_logger(), "Map segmentation server not available after waiting");
			rclcpp::shutdown();
		}

		// send a goal to the action
		auto goal = MapSegmentation::Goal();
		goal.input_map = create_image_message();
		goal.map_origin.position.x = 0;
		goal.map_origin.position.y = 0;
		goal.map_resolution = 0.05;
		goal.return_format_in_meter = false;
		goal.return_format_in_pixel = true;

		RCLCPP_INFO(this->get_logger(), "Sending segmentation goal");

		auto send_goal_options = rclcpp_action::Client<MapSegmentation>::SendGoalOptions();
		send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<MapSegmentation>::WrappedResult &result)
		{
			print_result(*this, "segmentation", result.code);
			if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
			{
				update_sequence_planning_parameters();
				send_sequence_planning_goal(result.result->room_information_in_pixel);
			}
		};
		this->map_segmentation_client_ptr_->async_send_goal(goal, send_goal_options);
	}

	void send_sequence_planning_goal(const std::vector<ipa_building_msgs::msg::RoomInformation> &room_information)
	{
		using namespace std::placeholders;

		if (!this->sequence_planning_client_ptr_->wait_for_action_server())
		{
			RCLCPP_ERROR(this->get_logger(), "Sequence planning server not available after waiting");
			rclcpp::shutdown();
		}

		auto goal = FindRoomSequenceWithCheckpoints::Goal();
		goal.input_map = create_image_message();
		goal.map_origin.position.x = 0;
		goal.map_origin.position.y = 0;
		goal.map_resolution = 0.05;
		goal.room_information_in_pixel = room_information;
		goal.robot_radius = 0.3;
		cv::Mat map_eroded;
		cv::erode(map_, map_eroded, cv::Mat(), cv::Point(-1, -1), goal.robot_radius / goal.map_resolution + 2);
		cv::Mat distance_map; // variable for the distance-transformed map, type: CV_32FC1
		cv::distanceTransform(map_eroded, distance_map, CV_DIST_L2, 5);
		cv::convertScaleAbs(distance_map, distance_map); // conversion to 8 bit image
		bool robot_start_coordinate_set = false;
		for (int v = 0; v < map_eroded.rows && robot_start_coordinate_set == false; ++v)
			for (int u = 0; u < map_eroded.cols && robot_start_coordinate_set == false; ++u)
				if (map_eroded.at<uchar>(v, u) != 0 && distance_map.at<uchar>(v, u) > 15)
				{
					goal.robot_start_coordinate.position.x = u * goal.map_resolution + goal.map_origin.position.x;
					goal.robot_start_coordinate.position.y = v * goal.map_resolution + goal.map_origin.position.y;
					robot_start_coordinate_set = true;
				}
		RCLCPP_INFO(this->get_logger(), "Sending sequence planning goal");

		auto send_goal_options = rclcpp_action::Client<FindRoomSequenceWithCheckpoints>::SendGoalOptions();
		send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<FindRoomSequenceWithCheckpoints>::WrappedResult &result)
		{
			print_result(*this, "sequence_planning", result.code);
		};
		this->sequence_planning_client_ptr_->async_send_goal(goal, send_goal_options);
	}

private:
	rclcpp_action::Client<MapSegmentation>::SharedPtr map_segmentation_client_ptr_;
	rclcpp_action::Client<FindRoomSequenceWithCheckpoints>::SharedPtr sequence_planning_client_ptr_;
	std::shared_ptr<rclcpp::AsyncParametersClient> segmentation_parameter_client_;
	std::shared_ptr<rclcpp::AsyncParametersClient> sequence_planning_parameter_client_;
	std::string image_path_;
	int room_segmentation_algorithm_;
	cv::Mat map_;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<RoomSequencePlanningClient>(rclcpp::NodeOptions());
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
