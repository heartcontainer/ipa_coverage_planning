#include <ipa_room_exploration/coverage_check_server.h>

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<CoverageCheckServer>(true);
	rclcpp::spin(node);
	rclcpp::shutdown();
}
