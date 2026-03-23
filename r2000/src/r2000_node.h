#ifndef R2000_NODE_H
#define R2000_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

namespace pepperl_fuchs
{

    class R2000Driver;

    class R2000Node : public rclcpp::Node
    {
    public:
        R2000Node();

        void cmdMsgCallback(const std_msgs::msg::String::SharedPtr msg);

        void getScanData();
        bool isConnected();

    private:
        bool connect();

        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_subscriber_;
        rclcpp::TimerBase::SharedPtr get_scan_data_timer_;

        rclcpp::Node::SharedPtr node;

        std::string frame_id_;
        std::string scanner_ip_;
        int scanner_port_;
        int scan_frequency_;
        int samples_per_scan_;
        int scan_start_angle;
        int scan_end_angle;

        float start_angle;
        float end_angle;
        R2000Driver *driver_;
        bool error_angleover_printed_;
        bool error_AngleDiffrenceOver_printed_;
    };

} // namespace pepperl_fuchs

#endif // R2000_NODE_H