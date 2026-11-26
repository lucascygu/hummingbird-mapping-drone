#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char **argv){
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("hello_node"); // actually a shared pointer
	// publisher
	auto pub = node->create_publisher<std_msgs::msg::String>("hello", 10);
	rclcpp::WallRate rate(1);

	while (rclcpp::ok()){
		std_msgs::msg::String msg;
		msg.data = "ROS2 Jazzy test";

		pub->publish(msg);
		RCLCPP_INFO(node->get_logger(), "%s", msg.data.c_str());
		rate.sleep();
	}

	rclcpp::shutdown();
	return 0;
}
