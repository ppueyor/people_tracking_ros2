#include <rclcpp/rclcpp.hpp>
#include "roloc_ekf.hpp"

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);

	// Range-only localization instance
	auto loc = std::make_shared<RoLocEKF>();

	// Spin for ever
	rclcpp::spin(loc);

	rclcpp::shutdown();
	return 0;
}
