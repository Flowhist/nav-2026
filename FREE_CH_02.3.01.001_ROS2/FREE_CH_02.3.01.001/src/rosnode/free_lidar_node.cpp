#include <free_lidar/free_lidar_node.h>
#include <rclcpp/qos.hpp>

std::ofstream g_topic_timer_file;

namespace free_optics {

FreeLidarNode::FreeLidarNode():Node("free_lidar_node")
{
    driver_ = 0;
    filter_ = 0;
    is_connected_ = false;

   this->declare_parameter<std::string>("frame_id", "laser_frame");
    this->declare_parameter<std::string>("scanner_ip", "192.168.1.111");
    this->declare_parameter<std::string>("port_name", "");
    this->declare_parameter<int>("baud", 921600);
    this->declare_parameter<int>("scan_frequency", 15);
    this->declare_parameter<int>("scan_resolution", 1000);
    this->declare_parameter<int32_t>("start_angle", -45);
    this->declare_parameter<int32_t>("stop_angle", 225);
    this->declare_parameter<float>("range_min", 0.05);
    this->declare_parameter<float>("range_max", 25.0);
    this->declare_parameter<int>("filter_switch", 0);
    this->declare_parameter<int>("cluster_num", 10);
    this->declare_parameter<int>("broad_filter_num", 20);
    
    this->declare_parameter<bool>("is_ethernet", true);
    this->declare_parameter<int>("NOR_switch", 1);
    this->declare_parameter<std::string>("topic_name", "/scan");
    this->declare_parameter<bool>("is_reverse_postion", false);
    
    
    this->get_parameter<std::string>("frame_id", frame_id_);
    this->get_parameter<std::string>("scanner_ip", scanner_ip_);
    this->get_parameter<std::string>("port_name", port_name_);
    this->get_parameter<int>("baud", baud_);
    this->get_parameter<int>("scan_frequency", scan_frequency_);
    this->get_parameter<int>("scan_resolution", scan_resolution_);
    this->get_parameter<int32_t>("start_angle", start_angle_);
    this->get_parameter<int32_t>("stop_angle", stop_angle_);
    this->get_parameter<float>("range_min", range_min_);
    this->get_parameter<float>("range_max", range_max_);
    this->get_parameter<int>("filter_switch", filter_switch_);
    this->get_parameter<int>("cluster_num", cluster_num_);
    this->get_parameter<int>("broad_filter_num", broad_filter_num_);
    this->get_parameter<bool>("is_ethernet", is_ethernet_);
    this->get_parameter<int>("NOR_switch", NOR_switch_);
    this->get_parameter<std::string>("topic_name", topic_name_);
    this->get_parameter<bool>("is_reverse_postion", is_reverse_postion_);
    
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1))   // 队列深度可自定
            .best_effort();                     // ← 关键：不等待 ACK

    scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(topic_name_, qos);
    info_publisher_ = this->create_publisher<std_msgs::msg::String>("info", qos);
    cmd_subscriber_ = this->create_subscription<std_msgs::msg::String>("control_command", qos,  
            std::bind(&FreeLidarNode::cmdMsgCallback, this, std::placeholders::_1));  // 声明订阅器  

         //  init_timer_log();


    if (!connect()) {
        return;
    }


    is_connected_ = true;

    
}

FreeLidarNode::~FreeLidarNode()
{
    if (driver_) {
        driver_->stop_socket_thread_ = true;
        driver_->disconnect();
        if (driver_->socket_thread_.joinable()) driver_->socket_thread_.join();
        if (driver_->parse_thread_.joinable())  driver_->parse_thread_.join();
    }
    delete filter_;
    filter_ = nullptr;
    delete driver_;
    driver_ = nullptr;
}

// 连接成功后
bool FreeLidarNode::startSocketThread()
{
    if (!driver_) return false;
    if (!driver_->isConnected()) return false;
    driver_->stop_socket_thread_ = false;

    // 接收线程
    driver_->socket_thread_ =
        std::thread(&LidarDriver::socketThreadFunc, driver_);

    // 解析线程
    driver_->parse_thread_ =
        std::thread(&FreeLidarNode::parseThreadFunc, this);
    return true;
}

void FreeLidarNode::parseThreadFunc()
{
    while (!driver_->stop_socket_thread_) {
           // 检查是否连接
        if (driver_->isConnected()) {
            // 如果已连接，尝试获取扫描数据
            if (!getScanData()) {
                std::cerr << "Error in getScanData(), attempting to reconnect..." << std::endl;
                driver_->disconnect();  // 断开当前连接
            }
        } else {
            // 如果未连接，尝试重新连接
            std::cerr << "Not connected, attempting to reconnect..." << std::endl;
            if (!connect()) {
                std::cerr << "Reconnection failed! Retrying in 5 seconds..." << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(5));  // 等待 5 秒后重试
            } else {
                std::cerr << "Reconnected successfully." << std::endl;
            }
        }

        // 每隔 500us 调用一次 getScanData()      
        std::this_thread::sleep_for(std::chrono::microseconds(500)); // 500 µs
    }
}


bool FreeLidarNode::connect()
{
    delete driver_;
    delete filter_;
    driver_ = nullptr;
    filter_ = nullptr;

    if (!is_ethernet_) {
        RCLCPP_WARN(
            this->get_logger(),
            "Serial mode is not built in finav; using ethernet driver with scanner_ip='%s'.",
            scanner_ip_.c_str()
        );
    }

    driver_ = new FREEEthDriver();
    std::cout << "Connecting to scanner at " << scanner_ip_ << " ... ";
    if (driver_->connect(scanner_ip_, 2111)) {
        std::cout << "OK" << std::endl;
    }
    else {
        std::cout << "FAILED!" << std::endl;
        std::cerr << "Connection to scanner at " << scanner_ip_ << " failed!" << std::endl;
        return false;
    }

    driver_->ClearBuf();
    uint8_t log_state = 0;
    if (!driver_->login(log_state) || log_state == 0) {
        driver_->disconnect();
        return false;
    }
    


    uint8_t retry = 0;

    while (1) {
        uint8_t device_state;
        if (!driver_->getDeviceState(device_state)) {
            return false;
        }

        if (device_state == 0x01 || device_state == 0x06) {
            break;
        }

        std::cerr << "Scanner is not ready, retrying..." << std::endl;
        retry++;
        if (retry > RETRY_NUM) {
            std::cerr << "Scanner running failed!" << std::endl;
            return false;
        }

        // ros::Duration(1).sleep();
        sleep(1);
    }

    filter_ = new TrailingFilter();

    std::string temp_type = scanner_type_;
    driver_->getDeviceName(scanner_type_);

    // if (scanner_type_.find('H') != std::string::npos || scanner_type_.find('h') != std::string::npos) {
    //     if (temp_type.find('C') != std::string::npos || temp_type.find('c') != std::string::npos) {
    //         std::cerr << "Scanner type mismatch!" << std::endl;
    //         return false;
    //     }
    // }

    if (scanner_type_.find('C') != std::string::npos || scanner_type_.find('c') != std::string::npos) {
        if (temp_type.find('H') != std::string::npos || temp_type.find('h') != std::string::npos) {
            std::cerr << "Scanner type mismatch!" << std::endl;
            return false;
        }
    }

    driver_->getFirmwareVersion(scanner_fw_);
    driver_->getSerialNumber(scanner_sn_);
    if (!driver_->setScanAngle(start_angle_*10000, stop_angle_*10000)) {
        return false;
    }
    if (!driver_->setScanConfig(scan_frequency_*100, scan_resolution_)) {
        return false;
    }
     // 检查 scanner_type_ 是否包含 'H' 或 'h'
    if (scanner_type_.find('H') != std::string::npos || scanner_type_.find('h') != std::string::npos) {        
        if (!driver_->setReflectivityNormalization(NOR_switch_)) {
            std::cerr << "Failed to set reflectivity normalization." << std::endl;
            return false;
        }
    }
   
    
    
    // driver_->getMeasureRange(range_min_, range_max_);
    driver_->getScanData(1);


    return true;
}




bool FreeLidarNode::getScanData()
{
    while (driver_->is_full_ == true) {
        rclcpp::Time start_porcess_data_time=this->now();
        ScanData scandata = driver_->getFullScan();

        rclcpp::Time before_filter_time=this->now();
        switch(filter_switch_)
        {
            case 0://无滤波
                break;
            case 1:{
                filter_->Filter(scandata, scan_resolution_,cluster_num_,broad_filter_num_);
            }
                break;
            case 2:
            {
                filter_->FilterHD(scandata);
                
            }   break;
            case 3:
            {
                filter_->FilterH1(scandata, scan_resolution_);
            }   break;
            
        }
        rclcpp::Time after_filter_time=this->now();
      
        
        scanmsg.header.frame_id = frame_id_;
        // 使用ROS时钟，保证与TF缓存处于同一时间基准
        rclcpp::Time topic_time = this->get_clock()->now();
        scanmsg.header.stamp = topic_time;
       // scanmsg.header.seq=(uint32_t)scandata.frame_seq;

        scanmsg.angle_min = (float)start_angle_*M_PI/180;
        scanmsg.angle_max = (float)stop_angle_*M_PI/180;
        scanmsg.angle_increment = 2*M_PI*scan_resolution_/(360.0f*10000);
        scanmsg.time_increment = 1*scan_resolution_/(float)scan_frequency_/(360.0f*10000);
        scanmsg.scan_time = 1/(float)scan_frequency_;
        //scanmsg.scan_time =(float)scandata.frame_seq;
        scanmsg.range_min = range_min_;
        scanmsg.range_max = range_max_;
        //const std::size_t n = scandata.ranges_data.size();
        scanmsg.ranges.resize(scandata.ranges_data.size());
        scanmsg.intensities.resize(scandata.intensities_data.size());

        std::vector<float> temp_distance,temp_ampletude;
        temp_distance.resize(scandata.ranges_data.size());  
        temp_ampletude.resize(scandata.intensities_data.size()); 

        
        for (std::size_t i=0; i<scandata.ranges_data.size(); i++) {
            if (scandata.ranges_data[i] == 0) {
                temp_distance[i] = 1/0.0f;
                 temp_ampletude[i] = 1/0.0f;
            }
            else {
                temp_distance[i] = float(scandata.ranges_data[i])/1000.0f;
                 temp_ampletude[i] = scandata.intensities_data[i];
            }
        }

        if(is_reverse_postion_==true)//反装
{
    for(int i=0;i<(int)temp_distance.size();i++)
    {
        scanmsg.ranges[i]=temp_distance[temp_distance.size()-1-i];
        scanmsg.intensities[i]=temp_ampletude[temp_distance.size()-1-i];
    }
}else//正装
{
 for(int i=0;i<(int)temp_distance.size();i++)
    {
        scanmsg.ranges[i]=temp_distance[i];
        scanmsg.intensities[i]=temp_ampletude[i];
    }
}
        




        rclcpp::Time before_sfilter_time=this->now();
        if(scanmsg.ranges.size() > 0){
            scanmsg.ranges = filter_->singleFilter(scanmsg.ranges,scanmsg.ranges.size(), scan_resolution_);
        }
        rclcpp::Time after_sfilter_time=this->now();
         // 设置消息头的时间戳

          // timer_log  
          rclcpp::Time time2(scandata.recv_first_sec, scandata.recv_first_nsec);
          rclcpp::Time time3(scandata.recv_last_sec, scandata.recv_last_nsec);
          rclcpp::Time end_porcess_data_time=this->now();
          write_timer_stamp(topic_time, time2,time3,this->now(),before_filter_time,after_filter_time,before_sfilter_time,after_sfilter_time,
                            start_porcess_data_time,
                            end_porcess_data_time);

        

        scan_publisher_->publish(scanmsg);

        if (driver_->getFullScanAvailable() == 0) {
            driver_->is_full_ = false;
        }
        else {
            // std::cout << "Scan data buffer is not empty." << std::endl;
        }
    }
    return driver_->scanDataReceiver();
}

void FreeLidarNode::cmdMsgCallback(const std_msgs::msg::String &msg)
{
    const std::string &cmd = msg.data;
    std::cout << cmd.c_str() << std::endl;
}


void FreeLidarNode::init_timer_log()
{
    namespace fs = std::filesystem;
//     std::string home_dir = std::getenv("HOME");
//    std::string dir =home_dir+"/userdata/"; 
   std::string dir = "/userdata/";   
    fs::create_directories(dir);

    std::string path = dir + "topic_timer_data.txt";

    // 1. 先以截断模式打开，清空旧内容
    {
        std::ofstream tmp(path, std::ios::out | std::ios::trunc);
        if (!tmp.is_open())
            throw std::runtime_error("Cannot open " + path);
    }

    // 2. 再打开为追加模式，供后续 write_timer_stamp 使用
    g_topic_timer_file.open(path, std::ios::app);
    if (!g_topic_timer_file.is_open())
        throw std::runtime_error("Cannot open " + path);

    // 设置固定 9 位小数
    g_topic_timer_file << std::fixed << std::setprecision(9);
}

void FreeLidarNode::write_timer_stamp(const rclcpp::Time& topic_time,
                       const rclcpp::Time& sys_recv_first_time,
                       const rclcpp::Time& sys_recv_last_time,
                       const rclcpp::Time& sys_send_time,
                       const rclcpp::Time& before_filter_time,
                       const rclcpp::Time& after_filter_time,
                       const rclcpp::Time& before_sfilter_time,
                       const rclcpp::Time& after_sfilter_time,
                        const rclcpp::Time& start_porcess_data_time,
                        const rclcpp::Time& end_porcess_data_time )
{
    double diff1=sys_recv_first_time.seconds()-topic_time.seconds();
    double diff2=sys_send_time.seconds()-topic_time.seconds();
    double diff3=sys_send_time.seconds()-sys_recv_first_time.seconds();
    double diff4=after_filter_time.seconds()-before_filter_time.seconds();
    double diff5=after_sfilter_time.seconds()-before_sfilter_time.seconds();
    double diff6=end_porcess_data_time.seconds()-start_porcess_data_time.seconds();
    double diff7=sys_recv_last_time.seconds()-sys_recv_first_time.seconds();

    g_topic_timer_file << " topic_stamp:"
                       << topic_time.seconds()
                       << " recv_stamp:"
                       << sys_recv_first_time.seconds()
                       << " send_stamp:"
                       << sys_send_time.seconds()
                       <<" diff1_recv_topic:"
                       <<diff1
                       <<" diff3_send_recv:"
                       <<diff3
                       <<" diff2_send_topic:"
                       <<diff2
                       <<" diff7_recvfirst_recvlast "
                       <<diff7
                       <<" diff4_filter_time "
                       <<diff4
                       <<" diff5_single_filter_time "
                       <<diff5               
                       <<" diff6_processdata_time "           
                       <<diff6               
                       << '\n';
    
    // if(diff2>0.1)
    // {
    //     std::cout <<std::fixed << std::setprecision(9)<< "[timer diff] recv_stamp:"<<sys_send_time.seconds()<<" topic_stamp:"<<topic_time.seconds()<<" diff2:"<<diff2<< std::endl;
    // }
    g_topic_timer_file.flush();   // 可选：立即落盘
}
}


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<free_optics::FreeLidarNode>();  // 直接创建FreeLidarNode节点

    int retry = 0; 
    while (rclcpp::ok()) {
        if (node->isConnected()) {
             node->startSocketThread();   // <-- 这里启动 1 ms 收线程
            break;
        }

        retry++;
        if (retry >= RETRY_NUM) {
            std::cerr << "Failed to connect after " << RETRY_NUM << " retries." << std::endl;
            return 0;
        }
        std::cerr << "Retrying..." << std::endl;
        rclcpp::sleep_for(std::chrono::seconds(1));  // 等待1秒再重试
    }

    while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

    // bool result = true;
    
    // while (rclcpp::ok() && result) {
    //     rclcpp::spin_some(node);  // 只处理这个节点的回调
    //     result = node->getScanData();

    //     if (!result) {
    //         std::cerr << "Scanner data timeout, attempting to restart!" << std::endl;
    //         // 尝试重启连接
    //         node = std::make_shared<free_optics::FreeLidarNode>();  // 重新初始化节点
    //         retry = 0;
    //         while (rclcpp::ok() && !node->isConnected()) {
    //             retry++;
    //             if (retry >= RETRY_NUM) {
    //                 std::cerr << "Failed to reconnect after " << RETRY_NUM << " retries." << std::endl;
    //                 return 0;
    //             }
    //             std::cerr << "Reconnecting..." << std::endl;
    //             rclcpp::sleep_for(std::chrono::seconds(1));  // 等待1秒再重试
    //         }
    //         std::cerr << "Reconnected successfully." << std::endl;
    //         result = true;  // 重置结果为true，继续循环
    //     }
    // }

    return 0;
}
