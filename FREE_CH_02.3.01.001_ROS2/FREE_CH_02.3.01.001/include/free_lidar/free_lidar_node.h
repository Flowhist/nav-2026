#ifndef FREE_LIDAR_NODE_H_
#define FREE_LIDAR_NODE_H_

#include <free_lidar/lidar_driver.h>
#include <free_lidar/free_eth_driver.h>
#include <free_lidar/trailing_filter.h>
#include "rclcpp/time.hpp"
#include <stdint.h>
#include <stdio.h>
#include <iostream>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include <fstream> // 用于文件操作
#include <cstdlib> // 用于获取用户主目录
#include <filesystem> // 用于文件路径操作
#include <thread>
#include <vector>
#include <algorithm>
#include <cmath>


namespace free_optics {

class LidarDriver;
class FREEEthDriver;
class TrailingFilter;


class FreeLidarNode : public rclcpp::Node
{
public:
    FreeLidarNode();
    ~FreeLidarNode();

    bool isConnected() {return is_connected_;}

    bool getScanData();
    sensor_msgs::msg::LaserScan scanmsg;
    void cmdMsgCallback(const std_msgs::msg::String &msg);
    int scan_frequency_, scan_resolution_;
    bool startSocketThread();
    void parseThreadFunc();
    
    
private:
    bool connect();
    void init_timer_log();
    void write_timer_stamp(const rclcpp::Time& topic_time,
                        const rclcpp::Time& sys_recv_first_time,
                        const rclcpp::Time& sys_recv_last_time,
                       const rclcpp::Time& sys_time,
                       const rclcpp::Time& before_filter_time,
                       const rclcpp::Time& after_filter_time,
                       const rclcpp::Time& before_sfilter_time,
                       const rclcpp::Time& after_sfilter_time,
                        const rclcpp::Time& start_porcess_data_time,
                        const rclcpp::Time& end_porcess_data_time);

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_; 
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr info_publisher_; 
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_subscriber_;  
    std::string frame_id_;
    std::string topic_name_;
    

    std::string scanner_ip_, port_name_;
    std::string scanner_type_;
    uint32_t scanner_fw_, scanner_sn_;

    float range_max_, range_min_;
    int32_t start_angle_, stop_angle_;
    
    int filter_switch_;
    bool is_ethernet_;
    int NOR_switch_;
    int cluster_num_;
    int broad_filter_num_;
    int baud_;

    LidarDriver *driver_;
    TrailingFilter *filter_;
    bool is_connected_;
    bool is_reverse_postion_;
    int flag=1;
};

}

#endif
