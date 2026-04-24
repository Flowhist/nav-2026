#ifndef TRAILING_FILTER_H_
#define TRAILING_FILTER_H_

#include <free_lidar/lidar_struct.h>
// #include <sensor_msgs/LaserScan.h>
#include <vector>

namespace free_optics {

class TrailingFilter
{
public:
    TrailingFilter();

    int test = 100;
    std::vector<float> Dist_pre;
    std::vector<float> Dist_out;
    std::vector<float> Dist_flag;
    std::vector<float> Dist_flag2;
    int angle_m = 0;
    int angle_n = 0;
    int Anglenum = 0;
    int Speednum = 30;
    float idiff1 = 0;
    float idiff2 = 0;
    float idiff3 = 0;
    float idiff4 = 0;
    float idiff = 0;
    int Runtimes = 1 ; 
    void Filter(ScanData &data, uint16_t resolution,uint16_t cluster_num,uint16_t broad_filter_num);
    void FilterHD(ScanData &data);
    void FilterH1(ScanData &data, uint16_t resolution);
    void FilterC2(ScanData &data, uint16_t resolution);
    std::vector<float> singleFilter(std::vector<float> tempdisx , int pointnum, int resolution);
    
    
};

} // namespace free_optics 


#endif
