#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#define MINDISTANCE 1.0
#define SPEED 1.0

using namespace std::chrono_literals;

class WanderNode : public rclcpp::Node
{
public:
    WanderNode() : Node("wander")
    {
        // Subscribe to the laser scan topic
        laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/base_scan", 10, std::bind(&WanderNode::laserCallback, this, std::placeholders::_1));

        // Publisher for movement commands
        movement_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Initialize the movement command
        command_.linear.x = SPEED;
        command_.linear.y = 0.0;
        command_.linear.z = 0.0;
        command_.angular.x = 0.0;
        command_.angular.y = 0.0;
        command_.angular.z = 0.0;

        // Main loop
        timer_ = this->create_wall_timer(100ms, std::bind(&WanderNode::timerCallback, this));
    }

private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr laser_msg)
    {
        bool stuck = false;
        for (size_t i = 0; i < laser_msg->ranges.size(); ++i)
        {
            if (i > laser_msg->ranges.size() * 0.4 && i < laser_msg->ranges.size() * 0.6)
            {
                if (laser_msg->ranges[i] < MINDISTANCE)
                {
                    RCLCPP_INFO(this->get_logger(), "I'm stuck");
                    stuck = true;
                    break;
                }
            }
        }

        if (stuck)
        {
            command_.linear.x = 0.0;
            command_.angular.z = 0.8;
        }
        else
        {
            command_.linear.x = SPEED;
            command_.angular.z = 0.0;
        }
    }

    void timerCallback()
    {
        // Publish the movement command
        movement_publisher_->publish(command_);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr movement_publisher_;
    geometry_msgs::msg::Twist command_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WanderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

