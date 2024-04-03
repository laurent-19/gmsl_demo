#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class MoveNode : public rclcpp::Node
{
public:
    MoveNode() : Node("move")
    {
        // Publisher for movement data
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Initialize the movement command
        command_.linear.x = 1.0;

        // Main loop
        timer_ = this->create_wall_timer(100ms, std::bind(&MoveNode::timerCallback, this));
    }

private:
    void timerCallback()
    {
        // Publish the movement command
        publisher_->publish(command_);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    geometry_msgs::msg::Twist command_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

