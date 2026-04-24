#ifndef LIDAR_STRUCT_H_
#define LIDAR_STRUCT_H_

#include <stdint.h>
#include <vector>


#define READ_DOWN       0x00
#define WRITE_DOWN      0x01
#define METHOD_DOWN     0x02
#define READ_UP         0x10
#define WRITE_UP        0x11
#define METHOD_UP       0x12

#define LOGIN           1
#define GET_STATE       11
#define GET_FW_VER      12
#define GET_NAME        14
#define GET_SN          24
#define SET_CONFIG      25
#define SET_ANGLE       27
#define GET_ANGLE       28
#define SET_NORSWITCH   33
#define GET_DATA        49
#define SCAN_DATA       0x32
#define GET_RANGE       93

#define RETRY_NUM       10

const uint8_t login_psw[4] = {0x20, 0x21, 0x05, 0x18};

namespace free_optics {

struct ScanData
{
    std::vector<uint16_t> ranges_data;
    std::vector<uint16_t> intensities_data;
    uint32_t sec;
    uint32_t nsec;
    
    uint32_t recv_first_sec;
    uint32_t recv_first_nsec;
    uint32_t recv_last_sec;
    uint32_t recv_last_nsec;
    uint16_t frame_seq;
    
};

}


#endif
