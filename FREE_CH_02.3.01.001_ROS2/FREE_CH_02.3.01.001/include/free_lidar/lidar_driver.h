#ifndef LIDAR_DRIVER_H_
#define LIDAR_DRIVER_H_

#include <free_lidar/lidar_struct.h>
#include <string>
#include <deque>
#include <atomic>
#include <mutex>
#include <thread>

namespace free_optics {

class LidarDriver
{
public:
    LidarDriver() {};
    virtual ~LidarDriver() {};

    virtual bool connect(const std::string hostname, int port) {
        (void)hostname; // 抑制未使用警告
        (void)port; // 抑制未使用警告
        return false;};
    virtual bool connect(const std::string portname) {         
    (void)portname; // 抑制未使用警告
    return false;};
    virtual void disconnect()=0;
    virtual void ClearBuf()=0;
    bool isConnected() {return is_connected_;}

    virtual bool getDeviceState(uint8_t &state)=0;
    virtual bool getDeviceName(std::string &name)=0;
    virtual bool getFirmwareVersion(uint32_t &version)=0;
    virtual bool getSerialNumber(uint32_t &sn)=0;
    virtual bool login(uint8_t &state)=0;
    virtual bool getScanAngle(int32_t &start_angle, int32_t &stop_angle)=0;
    virtual bool setScanAngle(int32_t start_angle, int32_t stop_angle)=0;
    virtual bool setReflectivityNormalization(uint8_t normalizationFactor)=0;
    virtual bool setScanConfig(uint16_t frequency, uint16_t resolution)=0;
    virtual bool getMeasureRange(float &range_min, float &range_max)=0;
    virtual bool getScanData(uint8_t state)=0;
    virtual bool scanDataReceiver()=0;
    virtual bool getBaudrate() {return false;};
    virtual void socketThreadFunc()=0;


  ScanData getFullScan()
    {
        ScanData data;
        if (scan_data_buf_.size() > 0) {
            data = scan_data_buf_[0];
            scan_data_buf_.pop_front();
        }
        return data;
    };
    std::size_t getFullScanAvailable() const
    {
        return scan_data_buf_.size();
    };

    bool is_full_;
    bool is_connected_;
    bool is_capturing_;

    ScanData scan_data_;
    std::deque<ScanData> scan_data_buf_;
    std::deque<uint8_t> data;

    uint16_t byte_count_;

        std::mutex data_mutex_;
    std::thread socket_thread_;
    std::atomic<bool> stop_socket_thread_{false};
    std::thread parse_thread_;
};

}

#endif
