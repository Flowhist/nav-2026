// Copyright (c) 2014, Pepperl+Fuchs GmbH, Mannheim
// Copyright (c) 2014, Denis Dillenberger
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice, this
//   list of conditions and the following disclaimer in the documentation and/or
//   other materials provided with the distribution.
//
// * Neither the name of Pepperl+Fuchs nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
// ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "r2000_node.h"
#include "r2000_driver.h"

#include <limits>
#include <stdexcept>

namespace pepperl_fuchs
{

    R2000Node::R2000Node()
        : Node("r2000_node"),
          start_angle(0.0f),
          end_angle(0.0f),
          driver_(nullptr),
          error_angleover_printed_(false),
          error_AngleDiffrenceOver_printed_(false)
    {
        // Reading and checking parameters
        this->declare_parameter<std::string>("frame_id", "/scan");
        this->declare_parameter<std::string>("scanner_ip", "");
        this->declare_parameter<int>("scanner_port", 80);
        this->declare_parameter<int>("scan_frequency", 35);
        this->declare_parameter<int>("samples_per_scan", 3600);
        this->declare_parameter<int>("scan_start_angle", 0);
        this->declare_parameter<int>("scan_end_angle", 90);

        this->get_parameter<std::string>("frame_id", frame_id_);
        this->get_parameter<std::string>("scanner_ip", scanner_ip_);
        this->get_parameter<int>("scanner_port", scanner_port_);
        this->get_parameter<int>("scan_frequency", scan_frequency_);
        this->get_parameter<int>("samples_per_scan", samples_per_scan_);
        this->get_parameter<int>("scan_start_angle", scan_start_angle);
        this->get_parameter<int>("scan_end_angle", scan_end_angle);
        std::cout << "scan_start_angle: " << scan_start_angle << std::endl;
        std::cout << "scan_end_angle:" << scan_end_angle << std::endl;

        if (scanner_ip_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "IP of laser range finder not set!");
            return;
        }

        if (!connect())
        {
            return;
        }

        // 使用 RELIABLE QoS（RViz 需要）
        scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 100);
        cmd_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "control_command", 100, std::bind(&R2000Node::cmdMsgCallback, this, std::placeholders::_1));
        get_scan_data_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / (2 * scan_frequency_)), std::bind(&R2000Node::getScanData, this));
    }
    //-----------------------------------------------------------------------------
    bool R2000Node::connect()
    {
        delete driver_;

        // Connecting to laser range finder
        //-------------------------------------------------------------------------
        driver_ = new R2000Driver();
        std::cout << "Connecting to scanner at " << scanner_ip_ << ":" << scanner_port_ << " ... ";
        if (driver_->connect(scanner_ip_, scanner_port_))
            std::cout << "OK" << std::endl;
        else
        {
            std::cout << "FAILED!" << std::endl;
            std::cerr << "Connection to scanner at " << scanner_ip_ << " failed!" << std::endl;
            return false;
        }

        // Setting, reading and displaying parameters
        //-------------------------------------------------------------------------
        driver_->setScanFrequency(scan_frequency_);
        driver_->setSamplesPerScan(samples_per_scan_);
        auto params = driver_->getParameters();
        std::cout << "Current scanner settings:" << std::endl;
        std::cout << "============================================================" << std::endl;
        for (const auto &p : params)
            std::cout << p.first << " : " << p.second << std::endl;
        std::cout << "============================================================" << std::endl;

        // Start capturing scanner data
        //-------------------------------------------------------------------------
        std::cout << "Starting capturing: ";
        if (driver_->startCapturingTCP())
            std::cout << "OK" << std::endl;
        else
        {
            std::cout << "FAILED!" << std::endl;
            return false;
        }

        return true;
    }

    // 标准化角度到-180-180度之间
    float normalizeAngle(int angle)
    {
        if (angle < -180)
            return (float)angle + 360.0f;
        else if (angle > 180)
            return (float)angle - 360.0f;
        else
            return (float)angle;
    }

    //-----------------------------------------------------------------------------
    // R2000Node类的成员函数，用于获取激光扫描数据并发布
    void R2000Node::getScanData()

    {
        // 检查激光测距仪是否正在捕获数据
        if (!driver_->isCapturing())
        {
            // 如果未捕获数据，则输出错误信息，并尝试重新连接
            std::cout << "\033[1;31mERROR: Laser range finder disconnected. Trying to reconnect..." << std::endl;
            // 循环尝试重新连接，直到成功或用户中断
            while (!connect())
            {
                // 输出重新连接失败的错误信息
                std::cout << "\033[1;31mERROR: Reconnect failed. Trying again in 2 seconds..." << std::endl;
                // 休眠2秒后再尝试重新连接
                usleep((2 * 1000000)); // 2秒 = 2 * 1000000微秒
            }
        }
        // 从激光测距仪驱动获取完整的扫描数据
        auto scandata = driver_->getFullScan();
        if (scandata.amplitude_data.empty() || scandata.distance_data.empty())
        {
            return;
        }

        start_angle = normalizeAngle(scan_start_angle);
        end_angle = normalizeAngle(scan_end_angle);
        if (start_angle < -360 || start_angle > 360 || end_angle < -360 || end_angle > 360)
        {
            if (error_angleover_printed_ == false)
            {
                std::cout << "\033[1;31mERROR: angle value over limit. angle is limited to -360~360" << std::endl;
                error_angleover_printed_ = true;
            }
            return;
        }

        if ((end_angle - start_angle) > 360) /*终止角度与起始角度差值大于360度*/
        {
            // 输出输入角度超限制的错误信息
            if (error_AngleDiffrenceOver_printed_ == false)
            {
                std::cout << "\033[1;31mERROR: angle value over limit. the diffence between end_angle and start_angle must be less than 360" << std::endl;
                error_AngleDiffrenceOver_printed_ = true;
            }
            return;
        }

        // 计算要显示的扫描点的数量
        const std::size_t num_points = scandata.distance_data.size();

        double scan_frequency_hz = static_cast<double>(scan_frequency_);
        const auto &cached_params = driver_->getParametersCached();
        const auto scan_freq_it = cached_params.find("scan_frequency");
        if (scan_freq_it != cached_params.end())
        {
            try
            {
                const double parsed_frequency = std::stod(scan_freq_it->second);
                if (parsed_frequency > 0.0)
                {
                    scan_frequency_hz = parsed_frequency;
                }
            }
            catch (const std::exception &)
            {
            }
        }
        const double scan_time_sec = (scan_frequency_hz > 0.0) ? (1.0 / scan_frequency_hz) : 0.0;

        // 创建一个sensor_msgs::LaserScan消息对象
        sensor_msgs::msg::LaserScan scanmsg;
        // 设置消息头信息，包括帧ID和时间戳
        scanmsg.header.frame_id = frame_id_; // 帧ID

        // 获取当前时间戳
        // std::cout << "getclock" << std::endl;
        const auto now = this->get_clock()->now();

        // LaserScan 的时间戳应尽量对应首束激光时刻，而不是发布时刻。
        scanmsg.header.stamp = now - rclcpp::Duration::from_seconds(scan_time_sec);

        // 当前驱动输出的点序列对应 [-pi, pi)。
        scanmsg.angle_min = -M_PI;
        scanmsg.angle_increment = static_cast<float>((2.0 * M_PI) / static_cast<double>(num_points));
        scanmsg.angle_max = scanmsg.angle_min + scanmsg.angle_increment * static_cast<float>(num_points - 1);
        scanmsg.time_increment = static_cast<float>(scan_time_sec / static_cast<double>(num_points));
        scanmsg.scan_time = static_cast<float>(scan_time_sec);
        scanmsg.range_min = std::atof(driver_->getParametersCached().at("radial_range_min").c_str());
        scanmsg.range_max = std::atof(driver_->getParametersCached().at("radial_range_max").c_str());

        // 调整ranges和intensities的大小并填充数据
        // std::cout << "resize angle" << std::endl;
        scanmsg.ranges.resize(num_points);
        scanmsg.intensities.resize(num_points);
        // 处理周期性：如果结束角度小于起始角度，则需要检查两个范围
        const bool crosses_zero = end_angle < start_angle;
        // std::cout << "transdata" << std::endl;
        for (std::size_t i = 0; i < num_points; i++)
        {
            const float current_angle = -180.0f + i * 360.0f / static_cast<float>(num_points); // 数据是从-180到180度
                                                                                                 // 根据是否穿过0点来更新数据
            if (crosses_zero)
            {
                // 如果角度在起始角度和180度之间，或者在0度和结束角度之间
                if ((current_angle >= start_angle && current_angle < 180.0f) || (current_angle >= -180.0f && current_angle <= end_angle))
                {
                    // 保留数据
                    scanmsg.ranges[i] = float(scandata.distance_data[i]) / 1000.0f;
                    scanmsg.intensities[i] = scandata.amplitude_data[i];
                }
                else
                {
                    // 用 inf 表示该方向无有效回波，避免与近距离障碍物混淆。
                    scanmsg.ranges[i] = std::numeric_limits<float>::infinity();
                    scanmsg.intensities[i] = 0.0f;
                }
            }
            else
            {
                // 如果角度在起始角度和结束角度之间
                if (current_angle >= start_angle && current_angle <= end_angle)
                {
                    // 保留数据
                    scanmsg.ranges[i] = float(scandata.distance_data[i]) / 1000.0f;
                    scanmsg.intensities[i] = scandata.amplitude_data[i];
                }
                else
                {
                    // 用 inf 表示该方向无有效回波，避免与近距离障碍物混淆。
                    scanmsg.ranges[i] = std::numeric_limits<float>::infinity();
                    scanmsg.intensities[i] = 0.0f;
                }
            }
        }

        // 发布激光扫描消息
        scan_publisher_->publish(scanmsg);
    }

    //-----------------------------------------------------------------------------
    void R2000Node::cmdMsgCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        const std::string &cmd = msg->data;
        static const std::string set_scan_frequency_cmd("set scan_frequency=");
        static const std::string set_samples_per_scan_cmd("set samples_per_scan=");

        // Setting of scan_frequency
        //-------------------------------------------------------------------------
        if (cmd.substr(0, set_scan_frequency_cmd.size()) == set_scan_frequency_cmd)
        {
            std::string value = cmd.substr(set_scan_frequency_cmd.size());
            int frequency = std::atoi(value.c_str());
            if (frequency >= 10 && frequency <= 50)
            {
                scan_frequency_ = frequency;
                driver_->setScanFrequency(frequency);
            }
        }

        // Setting of samples_per_scan
        //-------------------------------------------------------------------------
        if (cmd.substr(0, set_samples_per_scan_cmd.size()) == set_samples_per_scan_cmd)
        {
            std::string value = cmd.substr(set_samples_per_scan_cmd.size());
            int samples = std::atoi(value.c_str());
            if (samples >= 72 && samples <= 25200)
            {
                samples_per_scan_ = samples;
                driver_->setSamplesPerScan(samples);
            }
        }
    }

    //-----------------------------------------------------------------------------
} // NS

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<pepperl_fuchs::R2000Node>();
    rclcpp::spin(node);
    // rclcpp::shutdown();
    return 0;
}
