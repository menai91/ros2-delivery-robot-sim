#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/executors.hpp"           // needed for rclpp::spin
#include "rclcpp/node.hpp"                // needed for rclpp::Node
#include "sensor_msgs/msg/laser_scan.hpp" // needed for sensor_msgs::msg

class ScanListener : public rclcpp::Node
{
public:
    ScanListener() : Node("scan_listener")
    {
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::LaserScan::SharedPtr msg)
            {
                this->scan_callback(msg);
            });

        teleop_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        const float half_cone = (turning_ ? 30.0f : 15.0f) * M_PI / 180.0f;
        const float side_start = 15.0f * M_PI / 180.0f;
        const float side_end = 90.0f * M_PI / 180.0f;

        float min_front = std::numeric_limits<float>::infinity();
        float min_left = std::numeric_limits<float>::infinity();
        float min_right = std::numeric_limits<float>::infinity();

        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            const float angle = msg->angle_min + static_cast<float>(i) * msg->angle_increment;
            const float r = msg->ranges[i];
            
            if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max)
            {
                continue;
            }

            if (std::abs(angle) <= half_cone)
            {
                min_front = std::min(min_front, r);
            }
            if (angle > side_start && angle <= side_end)
            {
                min_left = std::min(min_left, r);
            }
            if (angle < -side_start && angle >= -side_end)
            {
                min_right = std::min(min_right, r);
            }
        }

        /* If we are moving, we want to stop when there is an obstacle within 0.5m in our front cone
        If are avoiding an ostacle, we want things to be clear 0.8m in front */
        if (turning_)
        {
            if (min_front > clear_distance)
            {
                turning_ = false;
                turn_direction_ = 0;
            }
        }
        else if (min_front < stop_distance)
        {
            turning_ = true;
            turn_direction_ = (min_left > min_right) ? 1 : -1;
        }

        geometry_msgs::msg::Twist cmd;
        if (turning_)
        {
            cmd.angular.z = 0.5 * static_cast<float>(turn_direction_);
        }
        else
        {
            cmd.linear.x = 0.2;
        }

        teleop_publisher_->publish(cmd);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 500,
            "front %.2f  left %.2f  right %.2f  turning %d",
            min_front, min_left, min_right, turning_);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "angle_min %.3f, angle_max %.3f, increment %.4f rad, range %.2f to %.2f m",
            msg->angle_min, msg->angle_max, msg->angle_increment,
            msg->range_min, msg->range_max);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr teleop_publisher_;
    bool turning_ = false;
    int turn_direction_ = 0.0f;

    const float stop_distance = 0.5f;
    const float clear_distance = 0.8f;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanListener>());
    rclcpp::shutdown();
    return 0;
}