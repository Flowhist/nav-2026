#ifndef C2_UART_DRIVER_H_
#define C2_UART_DRIVER_H_

#include <free_lidar/lidar_struct.h>
#include <free_lidar/lidar_driver.h>
#include <serial/serial.h>
#include <string>
#include <deque>
#include <stdint.h>

#define C2_ETH_ROS_VER         "3.00.02.005"

const uint32_t baudrate[3] = {460800, 921600, 1500000};

namespace free_optics{

class C2UartDriver : public LidarDriver
{
public:
    C2UartDriver();
    ~C2UartDriver();

    bool connect(const std::string portname,int32_t baud);

    void ClearBuf();

    void disconnect();

    bool getBaudrate();

    bool getDeviceState(uint8_t &state);

    bool getDeviceName(std::string &name);

    bool getFirmwareVersion(uint32_t &version);

    bool getSerialNumber(uint32_t &sn);

    bool login(uint8_t &state);

    bool getScanAngle(int32_t &start_angle, int32_t &stop_angle);

    bool setScanAngle(int32_t start_angle, int32_t stop_angle);

    bool setScanConfig(uint16_t frequency, uint16_t resolution);

    bool getMeasureRange(float &range_min, float &range_max);

    bool getScanData(uint8_t state);

    bool scanDataReceiver();

    bool setReflectivityNormalization(uint8_t normalizationFactor);

     void socketThreadFunc();  

private:
    serial::Serial sp_;

     struct timeval  last_packet_time_;   // 上一包时间
    uint16_t last_cloud_   = 0xffff;   // 上一帧扫描计数
  uint8_t  last_pkt_    = 0xff;     // 上一包序号
  bool current_ok_=true;    //is current cloud error
  uint16_t bad_scan_cnt=0xffff;//错误数据圈号
  bool     frame_started_   = false;    // 是否已开始收集当前帧

};

}

#endif
