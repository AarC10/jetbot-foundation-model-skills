#include "image_rectifier/image_rectifier_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ImageRectifierNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
