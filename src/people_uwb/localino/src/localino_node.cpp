#include "localino.hpp"

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	auto loc = std::make_shared<Localino>();

	if(!loc->connect())
		return -1;

	loc->process();

	rclcpp::shutdown();
	return 0;
}
