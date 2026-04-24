#include <free_lidar/trailing_filter.h>
#include <free_lidar/free_lidar_node.h>
#include <math.h>
#include <cmath>
#include <iostream>
#include <vector>
#include <thread>

#include <algorithm>


namespace free_optics {



TrailingFilter::TrailingFilter()
{
    ;
}

std::vector<float> TrailingFilter::singleFilter(std::vector<float> tempdisx , int pointnum, int resolution)
{   
    int check_Anglenum = (Anglenum *10000)/resolution;
    if(Runtimes == 1)
    {
        Runtimes =2;
        for(int i = 0; i < pointnum; i++)
        {
            Dist_flag.push_back(0);
            Dist_flag2.push_back(0);
            Dist_pre.push_back(0);
            Dist_out.push_back(0);
        }
    }

    for(int i = 0; i < pointnum; i++)
    {
         Dist_pre[i] = (tempdisx[i]*1000);   //读入点云数据
         Dist_out[i] = (tempdisx[i]*1000);   //预赋值给输出buf
    }
    //将上一圈产生的孤立点标志位赋值给将要使用的
    for(int i = 0; i < pointnum; i++)
    {
         
         Dist_flag2[i] = Dist_flag[i]; 
         Dist_flag[i] = 0;                       //将上一圈产生的孤立点标志位清零
    }
    // std::fill(Dist_flag.begin(), Dist_flag.end(), 0); 
    for(int i = 0; i < pointnum; i++)
    {
        if(i==0)
        {
            idiff3 = Dist_pre[i] - Dist_pre[i+1];
            idiff4 = Dist_pre[i] - Dist_pre[i+2];
            if(abs(idiff3)>=30 && abs(idiff4)>=30) //判断孤立点
            {
                if(i > check_Anglenum)
                {
                    angle_m = i -check_Anglenum;
                }
                else
                {
                    angle_m = i;
                }
                if(i < pointnum - check_Anglenum)
                {
                    angle_n = i + check_Anglenum;
                }
                else
                {
                    angle_n = pointnum;
                }
                for(int j = angle_m; j < angle_n; j++)
                {
                    Dist_flag[j] = Dist_pre[i];      //孤立点左右角度赋值标志位（距离）
                }

                idiff = Dist_flag2[i] - Dist_pre[i];

                if(Dist_flag2[i] == 0)               //根据上一圈产生的孤立点标志位（距离）判断此孤立点是否应保留
                {
                    Dist_out[i] = 0;
                }
                else if(abs(idiff)>=Speednum)
                {
                    Dist_out[i] = 0;
                }
            }
        }

        else if(i==1)
        {   
            idiff2 = Dist_pre[i] - Dist_pre[i-1];
            idiff3 = Dist_pre[i] - Dist_pre[i+1];
            idiff4 = Dist_pre[i] - Dist_pre[i+2];
            if(abs(idiff2)>=30 && abs(idiff3)>=30 && abs(idiff4)>=30 ) //判断孤立点
            {
                if(i > check_Anglenum)
                {
                    angle_m = i -check_Anglenum;
                }
                else
                {
                    angle_m = i;
                }
                if(i < pointnum - check_Anglenum)
                {
                    angle_n = i + check_Anglenum;
                }
                else
                {
                    angle_n = pointnum;
                }
                for(int j = angle_m; j < angle_n; j++)
                {
                    Dist_flag[j] = Dist_pre[i];      //孤立点左右角度赋值标志位（距离）
                }

                idiff = Dist_flag2[i] - Dist_pre[i];

                if(Dist_flag2[i] == 0)               //根据上一圈产生的孤立点标志位（距离）判断此孤立点是否应保留
                {
                    Dist_out[i] = 0;
                }
                else if(abs(idiff)>=Speednum)
                {
                    Dist_out[i] = 0;
                }
            }
        }

        else if(i == pointnum-2)
        {   
            idiff1 = Dist_pre[i] - Dist_pre[i-2];
            idiff2 = Dist_pre[i] - Dist_pre[i-1];
            idiff3 = Dist_pre[i] - Dist_pre[i+1];
            
            if(abs(idiff2)>=30 && abs(idiff3)>=30 && abs(idiff1)>=30 ) //判断孤立点
            {
                if(i > check_Anglenum)
                {
                    angle_m = i -check_Anglenum;
                }
                else
                {
                    angle_m = i;
                }
                if(i < pointnum - check_Anglenum)
                {
                    angle_n = i + check_Anglenum;
                }
                else
                {
                    angle_n = pointnum;
                }
                for(int j = angle_m; j < angle_n; j++)
                {
                    Dist_flag[j] = Dist_pre[i];      //孤立点左右角度赋值标志位（距离）
                }

                idiff = Dist_flag2[i] - Dist_pre[i];

                if(Dist_flag2[i] == 0)               //根据上一圈产生的孤立点标志位（距离）判断此孤立点是否应保留
                {
                    Dist_out[i] = 0;
                }
                else if(abs(idiff)>=Speednum)
                {
                    Dist_out[i] = 0;
                }
            }
        }
        
        else if(i == pointnum-1)
        {   
            idiff1 = Dist_pre[i] - Dist_pre[i-2];
            idiff2 = Dist_pre[i] - Dist_pre[i-1];
            
            if(abs(idiff2)>=30 && abs(idiff1)>=30 ) //判断孤立点
            {
                if(i > check_Anglenum)
                {
                    angle_m = i -check_Anglenum;
                }
                else
                {
                    angle_m = i;
                }
                if(i < pointnum - check_Anglenum)
                {
                    angle_n = i + check_Anglenum;
                }
                else
                {
                    angle_n = pointnum;
                }
                for(int j = angle_m; j < angle_n; j++)
                {
                    Dist_flag[j] = Dist_pre[i];      //孤立点左右角度赋值标志位（距离）
                }

                idiff = Dist_flag2[i] - Dist_pre[i];

                if(Dist_flag2[i] == 0)               //根据上一圈产生的孤立点标志位（距离）判断此孤立点是否应保留
                {
                    Dist_out[i] = 0;
                }
                else if(abs(idiff)>=Speednum)
                {
                    Dist_out[i] = 0;
                }
            }
        }
        else
        {   
            idiff1 = Dist_pre[i] - Dist_pre[i-2];
            idiff2 = Dist_pre[i] - Dist_pre[i-1];
            idiff3 = Dist_pre[i] - Dist_pre[i+1];
            idiff4 =  Dist_pre[i] - Dist_pre[i+2];
            if(abs(idiff2)>=30 && abs(idiff3)>=30 && abs(idiff1)>=30 && abs(idiff4)>=30 ) //判断孤立点
            {
                if(i > check_Anglenum)
                {
                    angle_m = i -check_Anglenum;
                }
                else
                {
                    angle_m = i;
                }
                if(i < pointnum - check_Anglenum)
                {
                    angle_n = i + check_Anglenum;
                }
                else
                {
                    angle_n = pointnum;
                }
                for(int j = angle_m; j < angle_n; j++)
                {
                    Dist_flag[j] = Dist_pre[i];      //孤立点左右角度赋值标志位（距离）
                }

                idiff = Dist_flag2[i] - Dist_pre[i];

                if(Dist_flag2[i] == 0)               //根据上一圈产生的孤立点标志位（距离）判断此孤立点是否应保留
                {
                    Dist_out[i] = 0;
                }
                else if(abs(idiff)>=Speednum)
                {
                    Dist_out[i] = 0;
                }
            }
        }
    }
    for(int i = 0; i < pointnum; i++)
        {
            tempdisx[i] = Dist_out[i]/1000.0f;
        }
    return tempdisx;
}


void TrailingFilter::Filter(ScanData &data, uint16_t resolution,uint16_t cluster_num,uint16_t broad_filter_num)
{
    if (data.ranges_data.size() <= 2 || data.intensities_data.size() <= 2) {
        return;
    }

    std::size_t point_num = data.ranges_data.size();

    ScanData temp_data;
    for (std::size_t i=0; i<point_num; i++) {
        temp_data.ranges_data.push_back(data.ranges_data[i]);
        temp_data.intensities_data.push_back(data.intensities_data[i]);
    }

    std::vector<uint16_t> filter_buf(point_num, 0);
    const uint16_t filter_value = 20;//差值比例 5%
    const uint16_t filter_value_high=2000;
    const uint16_t loop2=broad_filter_num;//展宽过滤点数
    const uint16_t loop1=100;

    uint16_t temp_filter_value = 0;

    for (std::size_t i=0; i<point_num; i++) {
        filter_buf[i] = filter_value;
    }

    uint16_t len_value = 10;
    uint16_t rssi_threshold=4095;
    uint8_t participation_points = 4;


    for (std::size_t i=0; i<point_num-1; i++) {
        if (data.intensities_data[i] >= rssi_threshold && data.intensities_data[i+1] < rssi_threshold) {
            std::size_t start_index = i;
            std::size_t stop_index;

            if (i+loop1 <= point_num-1) {
                stop_index = i + loop1;
            }
            else {
                stop_index = point_num - 1;
            }

            for (std::size_t j=start_index; j<stop_index; j++) {
                filter_buf[j] = filter_value_high;
                if (data.ranges_data[j]<= data.ranges_data[i] && data.ranges_data[j] > 10) {
                    filter_buf[j] = filter_value;
                }
            }
            //展宽过滤
            std::size_t start_index2 = i;
            std::size_t stop_index2;

            if (i+loop2 <= point_num-1) {
                stop_index2 = i + loop2;
            }
            else {
                stop_index2 = point_num - 1;
            }

            for (std::size_t j=start_index2; j<stop_index2; j++) {
               data.ranges_data[j]=0;
               data.intensities_data[j]=0;
               temp_data.ranges_data[j]=0;
               temp_data.intensities_data[j]=0;
            }
        }
        else if (data.intensities_data[i] < rssi_threshold&& data.intensities_data[i+1] >= rssi_threshold) {
            std::size_t start_index;
            std::size_t stop_index = i;

            if (i >= loop1) {
                start_index = i - loop1;
            }
            else {
                start_index = 0;
            }

            for (std::size_t j=start_index; j<stop_index; j++) {
                filter_buf[j] = filter_value_high;
                if (data.ranges_data[j]<= data.ranges_data[i+1] && data.ranges_data[j] > 10) {
                    filter_buf[j] = filter_value;
                }
            }
            //展宽过滤
              std::size_t start_index2;
            std::size_t stop_index2 = i;

            if (i >= loop2) {
                start_index2 = i - loop2;
            }
            else {
                start_index2 = 0;
            }
              for (std::size_t j=start_index2; j<stop_index2; j++) {
               data.ranges_data[j]=0;
               data.intensities_data[j]=0;
               temp_data.ranges_data[j]=0;
               temp_data.intensities_data[j]=0;
            }
        }
    }

    for (std::size_t i=0; i<point_num-participation_points-2; i++) {
        temp_filter_value = filter_buf[i];
        if (temp_data.ranges_data[i] <= 8000) {
            if (temp_data.ranges_data[i] > 10 && temp_data.ranges_data[i+participation_points] >= (temp_data.ranges_data[i]+len_value)) {
                if (temp_filter_value*(temp_data.ranges_data[i+participation_points]-temp_data.ranges_data[i]) >= temp_data.ranges_data[i]) {
                    
                    for(int j=0;j<participation_points;j++)
                    {
                        data.ranges_data[i+j+1] = 0;
                        data.intensities_data[i+j+1] = 0;
                    }
                }
            }
            else if (temp_data.ranges_data[i+participation_points] > 10 && temp_data.ranges_data[i] >= (temp_data.ranges_data[i+participation_points]+len_value)) {
                if (temp_filter_value*(temp_data.ranges_data[i]-temp_data.ranges_data[i+participation_points]) >= temp_data.ranges_data[i+participation_points]) {
                     for(int j=0;j<participation_points;j++)
                    {
                        data.ranges_data[i+j] = 0;
                        data.intensities_data[i+j] = 0;
                    }
                }
            }
        }
    }

    for (std::size_t i=1; i<point_num-1; i++) {
        if (temp_data.ranges_data[i-1] <= 10 && temp_data.ranges_data[i+1] <= 10) {
            data.ranges_data[i] = 0;
            data.intensities_data[i] = 0;
        }
    }



    //聚类
    for (std::size_t i=0; i<point_num-cluster_num+1; i++) {

    float min_cluster_distance=(float)temp_data.ranges_data[i]*2*M_PI*resolution/(360.0f*10000);
    
    bool is_Cluster=true;

    for(int j=0;j<cluster_num-1;j++)
    {
        if(abs(temp_data.ranges_data[i+j]-temp_data.ranges_data[i+j+1]) > min_cluster_distance)
        {
            is_Cluster=false;
            break;
        }      
    }

    for(int j=0;j<cluster_num;j++)
    {
      if(temp_data.ranges_data[i+j]<=10)
        {
            is_Cluster=false;
            break;
        }
    }

    if(is_Cluster)
    {
             for(int j=0;j<cluster_num;j++)
    {
        data.ranges_data[i+j] = temp_data.ranges_data[i+j];
        data.intensities_data[i+j] = temp_data.intensities_data[i+j];
    }
    }

    }
}

void TrailingFilter::FilterHD(ScanData &data)
{
    if (data.ranges_data.size() <= 10 || data.intensities_data.size() <= 10) {
        return;
    }

    std::size_t point_num = data.ranges_data.size();

    ScanData temp_data;
    for (std::size_t i=0; i<point_num; i++) {
        temp_data.ranges_data.push_back(data.ranges_data[i]);
        temp_data.intensities_data.push_back(data.intensities_data[i]);
    }

    std::vector<uint16_t> filter_buf(point_num, 0);
    const uint16_t filter_value = 35;

    for (std::size_t i=0; i<point_num; i++) {
        filter_buf[i] = filter_value;
    }

    uint16_t len_value = 5;

    for (std::size_t i=0; i<point_num-1; i++) {
        if (data.intensities_data[i] >= 1000 && data.intensities_data[i+1] < 1000) {
            std::size_t start_index = i;
            std::size_t stop_index;

            if (i+30 <= point_num-1) {
                stop_index = i + 30;
            }
            else {
                stop_index = point_num - 1;
            }

            for (std::size_t j=start_index; j<stop_index; j++) {
                filter_buf[j] = 50;
                if (data.ranges_data[j] + 50 <= data.ranges_data[i] && data.ranges_data[j] > 10) {
                    filter_buf[j] = filter_value;
                }
            }
        }
        else if (data.intensities_data[i] < 1000 && data.intensities_data[i+1] >= 1000) {
            std::size_t start_index;
            std::size_t stop_index = i;
            if (i >= 29) {
                start_index = i - 29;
            }
            else {
                start_index = 1;
            }

            for (std::size_t j=start_index; j<stop_index; j++) {
                filter_buf[j] = 50;
                if (data.ranges_data[j]+50 <= data.ranges_data[i+1] && data.ranges_data[j] > 10) {
                    filter_buf[j] = filter_value;
                }
            }
        }
    }

    for (std::size_t i=0; i<point_num-7; i++) {
        if (temp_data.ranges_data[i] <= 8000 && temp_data.ranges_data[i+1] <= 8000 && \
            temp_data.ranges_data[i+2] <= 8000 && temp_data.ranges_data[i+3] <= 8000 && \
            temp_data.ranges_data[i+4] <= 8000 && temp_data.ranges_data[i+5] <= 8000 && \
            temp_data.ranges_data[i+6] <= 8000 && temp_data.ranges_data[i+7] <= 8000) {
            if (temp_data.ranges_data[i] > 10 && temp_data.ranges_data[i+7] >= temp_data.ranges_data[i]+len_value) {
                if (filter_buf[i]*(temp_data.ranges_data[i+7]-temp_data.ranges_data[i]) >= temp_data.ranges_data[i]) {
                    data.ranges_data[i+1] = 0;
                    data.ranges_data[i+2] = 0;
                    data.ranges_data[i+3] = 0;
                    data.ranges_data[i+4] = 0;
                    data.ranges_data[i+5] = 0;
                    data.ranges_data[i+6] = 0;
                    data.ranges_data[i+7] = 0;
                    data.intensities_data[i+1] = 0;
                    data.intensities_data[i+2] = 0;
                    data.intensities_data[i+3] = 0;
                    data.intensities_data[i+4] = 0;
                    data.intensities_data[i+5] = 0;
                    data.intensities_data[i+6] = 0;
                    data.intensities_data[i+7] = 0;
                }
                // if (filter_buf[i]*(temp_data.ranges_data[i+2]-temp_data.ranges_data[i]) >= temp_data.ranges_data[i]) {
                //     data.ranges_data[i+1] = 0;
                //     data.ranges_data[i+2] = 0;
                //     data.intensities_data[i+1] = 0;
                //     data.intensities_data[i+2] = 0;
                // }median_value
                    data.ranges_data[i+3] = 0;
                    data.ranges_data[i+4] = 0;
                    data.ranges_data[i+5] = 0;
                    data.ranges_data[i+6] = 0;
                    data.intensities_data[i] = 0;
                    data.intensities_data[i+1] = 0;
                    data.intensities_data[i+2] = 0;
                    data.intensities_data[i+3] = 0;
                    data.intensities_data[i+4] = 0;
                    data.intensities_data[i+5] = 0;
                    data.intensities_data[i+6] = 0;
                }
                // if (filter_buf[i]*(temp_data.ranges_data[i]-temp_data.ranges_data[i+2]) >= temp_data.ranges_data[i+2]) {
                //     data.ranges_data[i] = 0;
                //     data.ranges_data[i+1] = 0;
                //     data.intensities_data[i] = 0;
                //     data.intensities_data[i+1] = 0;
                // }
            }
        }
    

    for (std::size_t i=1; i<point_num-1; i++) {
        if (temp_data.ranges_data[i-1] <= 10 && temp_data.ranges_data[i+1] <= 10) {
            data.ranges_data[i] = 0;
            data.intensities_data[i] = 0;
        }
    }

    // for (std::size_t i=2; i<point_num-2; i++) {
    //     if (temp_data.ranges_data[i-2] > 10 && temp_data.ranges_data[i-1] > 10 && 
    //         temp_data.ranges_data[i+1] > 10 && temp_data.ranges_data[i+2] > 10 && 
    //         temp_data.ranges_data[i] < temp_data.ranges_data[i-2]) {
    //         if ((temp_data.ranges_data[i]-temp_data.ranges_data[i-2])*(temp_data.ranges_data[i]-temp_data.ranges_data[i+2]) >= 1000) {
    //             data.ranges_data[i] = temp_data.ranges_data[i];
    //             data.intensities_data[i] = temp_data.intensities_data[i];
    //         }
    //     }
    // }

    // for (std::size_t i=0; i<point_num-2; i++) {
    //     if (abs(temp_data.ranges_data[i]-temp_data.ranges_data[i+1]) <= len_value && 
    //         abs(temp_data.ranges_data[i+1]-temp_data.ranges_data[i+2]) <= len_value && 
    //         temp_data.ranges_data[i] > 10 && temp_data.ranges_data[i+1] > 10 && temp_data.ranges_data[i+2] > 10) {
    //         data.ranges_data[i] = temp_data.ranges_data[i];
    //         data.ranges_data[i+1] = temp_data.ranges_data[i+1];
    //         data.ranges_data[i+2] = temp_data.ranges_data[i+2];
    //         data.intensities_data[i] = temp_data.intensities_data[i];
    //         data.intensities_data[i+1] = temp_data.intensities_data[i+1];
    //         data.intensities_data[i+2] = temp_data.intensities_data[i+2];
    //     }
    // }
}

void TrailingFilter::FilterC2(ScanData &data, uint16_t resolution)
{
    
    if (data.ranges_data.size() <= 2 || data.intensities_data.size() <= 2) {
        return;
    }
    
    std::size_t point_num = data.ranges_data.size();
    std::vector<double> angle_value;
    std::vector<double> angle_pro_value;
    std::vector<int> filter_buf;
    std::vector<int> diff_dis;
    angle_value.push_back(90.0f);
    std::size_t high_rssi = 200;
    std::size_t diff_rssi = 3;
    
    for (std::size_t i=1; i<point_num; i++) {
        filter_buf.push_back(15);  // 除反光条，角度阈值为15
        diff_dis.push_back(10);  // 除反光条，距离阈值为10
    }

    std::size_t widen_point_num = (point_num - 1)/270*5;  // 针对反光条两边分别拓宽点数

    for (std::size_t i=widen_point_num; i<point_num-widen_point_num; i++) {  // 反光条左右拓宽的点，角度阈值30，距离阈值5
        if (data.intensities_data[i]>=high_rssi && data.intensities_data[i+1]<=high_rssi) {
            for (std::size_t j=0; j<widen_point_num; j++) {
                filter_buf[i+j] = 30;
                diff_dis[i+j] = 5;
            }
        }
        else if (data.intensities_data[i]<=high_rssi && data.intensities_data[i+1]>=high_rssi) {
            for (std::size_t j=0; j<widen_point_num; j++) {
                filter_buf[i-j] = 30;
                diff_dis[i-j] = 5;
            }
        }
    }
    // 计算相邻点角度
    for (std::size_t i=1; i<point_num; i++) {
        if ((double)data.ranges_data[i-1] == (double)data.ranges_data[i] * cos(M_PI / 180 * double(resolution) / 10000)) {
            angle_value.push_back(90);
        }
        else {
            double temp_value = 180 / M_PI * atan((double)data.ranges_data[i] * sin(M_PI / 180 * (double)resolution / 10000) / \
                            ((double)data.ranges_data[i-1] - (double)data.ranges_data[i] * cos(M_PI / 180 * double(resolution) / 10000)));
        if (temp_value < 0) {
            temp_value += 180;
        }
        angle_value.push_back(temp_value);
        }
    }

    //纠正【非极小-极小-极大-非极大】数据，因为这是距离抖动造成的，将其与细小物体数据区分开
    std::vector<int> change_flag;
    for (std::size_t i=0; i<point_num; i++) {
        change_flag.push_back(0);
    }

    for (std::size_t i=5; i<point_num-5; i++) {
        //if ((angle_value[i-1] > filter_buf[i-1] || change_flag[i-1] == 1) && angle_value[i] <= filter_buf[i] && angle_value[i+1] >= (180-filter_buf[i+1]) && angle_value[i+2] < (180-filter_buf[i+2]))  // 识别数据：【非极小-极小-极大-非极大】
            uint8_t big_num = 0;
            uint8_t small_num = 0;
            for (std::size_t j=-5; j<5; j++) {
                if (angle_value[i+j] <= filter_buf[i+j]) {
                    small_num++;
                }
                else if (angle_value[i+j] >= (180-filter_buf[i+j])) {
                    big_num++;
                }
            if (big_num > small_num) {
                angle_value[i] = 180-angle_value[i];
            }
            else if (big_num < small_num) {
                angle_value[i+1] = 180-angle_value[i+1];
            }
            else {
                if ((angle_value[i]+angle_value[i+1]) <= 180) {
                    angle_value[i+1] = 180-angle_value[i+1];
                }
                else {
                    angle_value[i] = 180-angle_value[i];
                }
            }
            }
            
            
    }

    // 一群都是极大或都是极小的点中，不单调的个别点变为单调
    std::size_t start_index;
    std::size_t end_index;
    // printf("resolution:%f",double(resolution) / 10000);
    
    if (double(resolution) / 10000 == 0.1) {  // 角分辨率为0.1
        start_index = 5;
        end_index = 6;
    }
    else {  // 角分辨率为0.05
        start_index = 7;
        end_index = 8;
    }
    for (std::size_t i=0; i<start_index; i++) {  // 前start_index个角度直接拿过来
        angle_pro_value.push_back(angle_value[i]);
    }
    
    for (std::size_t i=start_index; i<point_num-end_index; i++) { //第start_index到-end_index个点
        uint8_t big_num = 0;
        uint8_t small_num = 0;
        std::vector<double> list;
        for(std::size_t j=i-start_index;j<i+end_index;j++){ //每个i-start_index到i+end_index之间的点放到列表
            list.push_back(angle_value[j]);
        }
        for (std::size_t j=0; j<start_index+end_index; j++) {
            if (list[j] <= filter_buf[i-start_index+j]) {
                small_num++;
            }
            else if (list[j] >= (180 - filter_buf[i-start_index+j])) {
                big_num++;
            }
        }
        if (small_num > 5 and (small_num - big_num) > 1 and angle_value[i] >= (180 - filter_buf[i])) {
            angle_pro_value.push_back(180 - angle_value[i]);
        }
        else if (big_num > 5 and (big_num - small_num) > 1 and angle_value[i] <= filter_buf[i]) {
            angle_pro_value.push_back(180 - angle_value[i]);
        }
        else {
            angle_pro_value.push_back(angle_value[i]);
        }
    }
    for (std::size_t i=point_num-end_index; i<point_num; i++) {  // 最后end_index个角度直接拿过来
        angle_pro_value.push_back(angle_value[i]);
    }
    
    double temp_buf[5];
    std::vector<double> median_value;
    std::vector<double> average_value;
    // 计算中值
    for (std::size_t i=0; i<point_num; i++) {
        // printf("angle_pro_value:%f",angle_pro_value[i]);
        if (i==0) {
            temp_buf[0] = 90.0f;
            temp_buf[1] = 90.0f;
            temp_buf[2] = angle_pro_value[i];
            temp_buf[3] = angle_pro_value[i+1];
            temp_buf[4] = angle_pro_value[i+2];
        }
        else if (i==1) {
            temp_buf[0] = 90.0f;
            temp_buf[1] = angle_pro_value[i];
            temp_buf[2] = angle_pro_value[i+1];
            temp_buf[3] = angle_pro_value[i+2];
            temp_buf[4] = angle_pro_value[i-1];
        }
        else if (i==point_num-2) {
            temp_buf[0] = angle_pro_value[i-2];
            temp_buf[1] = angle_pro_value[i-1];
            temp_buf[2] = angle_pro_value[i];
            temp_buf[3] = angle_pro_value[i+1];
            temp_buf[4] = 90.0f;
        }
        else if (i==point_num-1) {
            temp_buf[0] = angle_pro_value[i-2];
            temp_buf[1] = angle_pro_value[i-1];
            temp_buf[2] = angle_pro_value[i];
            temp_buf[3] = 90.0f;
            temp_buf[4] = 90.0f;
        }
        else {
            temp_buf[0] = angle_pro_value[i-2];
            temp_buf[1] = angle_pro_value[i-1];
            temp_buf[2] = angle_pro_value[i];
            temp_buf[3] = angle_pro_value[i+1];
            temp_buf[4] = angle_pro_value[i+2];
        }

        for (std::size_t j=0; j<4; j++) {
            for (std::size_t k=0; k<4-j; k++) {
                if (temp_buf[k] > temp_buf[k+1]) {
                    double temp_change = temp_buf[k];
                    temp_buf[k] = temp_buf[k+1];
                    temp_buf[k+1] = temp_change;
                }
            }
        }

        median_value.push_back(temp_buf[2]);
    }
    // 计算平均值
    // cout <<"point num = " << point_num << endl;
    for (std::size_t i=0; i<point_num; i++) {
        if (i==0) {
            temp_buf[0] = 90.0f;
            temp_buf[1] = 90.0f;
            temp_buf[2] = median_value[i];
            temp_buf[3] = median_value[i+1];
            temp_buf[4] = median_value[i+2];
        }
        else if (i==1) {
            temp_buf[0] = 90.0f;
            temp_buf[1] = median_value[i];
            temp_buf[2] = median_value[i+1];
            temp_buf[3] = median_value[i+2];
            temp_buf[4] = median_value[i-1];
        }
        else if (i==point_num-2) {
            temp_buf[0] = median_value[i-2];
            temp_buf[1] = median_value[i-1];
            temp_buf[2] = median_value[i];
            temp_buf[3] = median_value[i+1];
            temp_buf[4] = 90.0f;
        }
        else if (i==point_num-1) {
            temp_buf[0] = median_value[i-2];
            temp_buf[1] = median_value[i-1];
            temp_buf[2] = median_value[i];
            temp_buf[3] = 90.0f;
            temp_buf[4] = 90.0f;
        }
        else {
            temp_buf[0] = median_value[i-2];
            temp_buf[1] = median_value[i-1];
            temp_buf[2] = median_value[i];
            temp_buf[3] = median_value[i+1];
            temp_buf[4] = median_value[i+2];
        }

        double temp_average = 0.0f;
        for (std::size_t j=0; j<5; j++) {
            temp_average += temp_buf[j];
        }

        average_value.push_back(temp_average / 5);
    }
    // 滤掉同方向单个没被滤掉的点
    for (std::size_t i=1; i<point_num-1; i++) {
        if (average_value[i-1] >= (180 - filter_buf[i]) && average_value[i+1] >= (180 - filter_buf[i])) {
            average_value[i] = (180 - filter_buf[i]);
        }
        else if (average_value[i-1] <= filter_buf[i] && average_value[i+1] <= filter_buf[i]) {
            average_value[i] = filter_buf[i];
        }
    }
    // 滤掉同方向多个没被滤掉的点
    // int filter_point_num = (point_num-1)/270*5; //过滤反光柱两边葫芦形拖尾的点数范围
    // for (std::size_t i=0; i<point_num-1; i++) {
    //     if (average_value[i]<=filter_buf[i] && average_value[i+1]>filter_buf[i] && average_value[i+1]<=(180 - filter_buf[i])) {
    //         for (std::size_t j=2; j<filter_point_num; j++) {
    //             if ((i+j)>point_num-1) {
    //                 break;
    //             }
    //             if (average_value[i+j]>=(180 - filter_buf[i])) {
    //                 break;
    //             }
    //             else if (average_value[i+j]<=filter_buf[i]) {
    //                 for (std::size_t m=i; m<i+j; m++) {
    //                     average_value[m] = filter_buf[i];
    //                     }
    //                 break;
                    
    //             }
    //         }
    //     }
    //     else if (average_value[i]>=(180 - filter_buf[i]) && average_value[i+1]>=filter_buf[i] && average_value[i+1]<=(180 - filter_buf[i])) {
    //         for (std::size_t j=2; j<filter_point_num; j++) {
    //             if ((i+j)>point_num-1) {
    //                 break;
    //             }
    //             if (average_value[i+j]<=filter_buf[i]) {
    //                 break;
    //             }
    //             else if (average_value[i+j]>=(180 - filter_buf[i])) {
    //                 for (std::size_t m=i; m<i+j; m++) {
    //                     average_value[m] = (180 - filter_buf[i]);
    //                     }
    //                 break;
    //             }
    //         }
    //     }
    // }
    
    // 根据反射率补点
    std::vector<int> rssi_flag1;
    std::vector<int> rssi_flag2;
    
    for (std::size_t i=0; i<point_num-1; i++) {
        if (abs(data.intensities_data[i+1] - data.intensities_data[i]) >= diff_rssi && data.intensities_data[i] != 0 && data.intensities_data[i+1] != 0) {
            rssi_flag1.push_back(1);
        }
        else {
            rssi_flag1.push_back(0);
        }
    }
    rssi_flag1.push_back(0);
    // 11个里有3个flag就置为1
    for (std::size_t i=0; i<5; i++) {
        rssi_flag2.push_back(rssi_flag1[i]);
    }
    for (std::size_t i=5; i<point_num-5; i++) {
        int num = 0;
        for (std::size_t j=0; j<11; j++) {
            if (rssi_flag1[i-5+j] == 1) {
                num++;
            }
        }
        if (num >= 3) {
            rssi_flag2.push_back(1);
        }
        else {
            rssi_flag2.push_back(0);
        }
    }
    for (std::size_t i=point_num-5; i<point_num; i++) {
        rssi_flag2.push_back(rssi_flag1[i]);
    }

    // 根据距离补点
    std::vector<int> dis_flag1;
    std::vector<int> dis_flag2;
    
    for (std::size_t i=0; i<point_num-1; i++) {
        if (abs(data.ranges_data[i+1] - data.ranges_data[i]) >= diff_dis[i] && data.ranges_data[i] != 0 && data.ranges_data[i+1] != 0) {
            dis_flag1.push_back(1);
        }
        else {
            dis_flag1.push_back(0);
        }
    }
    dis_flag1.push_back(0);
    // 11个里有5个flag就置为1
    for (std::size_t i=0; i<5; i++) {
        dis_flag2.push_back(dis_flag1[i]);
    }
    for (std::size_t i=5; i<point_num-5; i++) {
        int num = 0;
        for (std::size_t j=0; j<11; j++) {
            if (dis_flag1[i-5+j] == 1) {
                num++;
            }
        }
        if (num >= 3) {
            dis_flag2.push_back(1);
        }
        else {
            dis_flag2.push_back(0);
        }
    }
    for (std::size_t i=point_num-5; i<point_num; i++) {
        dis_flag2.push_back(dis_flag1[i]);
    }
    for (std::size_t i=1; i<point_num-1; i++) { // 过滤dis_flag2中零星噪点
        if (dis_flag2[i-1] == 0 && dis_flag2[i] == 1 && dis_flag2[i+1] == 0 && dis_flag1[i] == 0) {
            dis_flag2[i] = 0;
        }
        else if (dis_flag2[i-1] == 1 && dis_flag2[i] == 0 && dis_flag2[i+1] == 1 && dis_flag1[i] == 1) {
            dis_flag2[i] = 1;
        }
    }

    std::vector<int> filter_flag;
    for (std::size_t i=0; i<point_num; i++) {
        if ((average_value[i] >= (180 - filter_buf[i]) || average_value[i] <= filter_buf[i]) && (dis_flag2[i] == 1 || rssi_flag2[i] == 1) && data.ranges_data[i] < 8000) {
            filter_flag.push_back(1);
        }
        else {
            filter_flag.push_back(0);
        }
    }

    // 处理小物体的拖尾，选中小物体及其拖尾，选其距离最小值30mm内的点保留下来【left_flag | i | mid_flag | right_flag】
    std::size_t flag = 0;
    std::size_t left_flag;
    std::size_t mid_flag;
    std::size_t right_flag;
    for (std::size_t i=0; i<point_num-11; i++) {
        if (angle_pro_value[i] <= filter_buf[i] && angle_pro_value[i+1] >= filter_buf[i+1] && angle_pro_value[i+1] <= (180-filter_buf[i+1])) { //判断是否符合小物体特征【极小-中-极大】
            for (std::size_t j=2; j<10; j++) {
                if (angle_pro_value[i+j] >= (180-filter_buf[i+j])) {
                    flag = 1;
                    break;
                }
            }
        }
        else if (angle_pro_value[i] <= filter_buf[i] && angle_pro_value[i+1] >= (180-filter_buf[i+1])) {//判断是否符合小物体特征【极小-极大】
            flag = 1;
        }
        if (flag == 1) {
            for (std::size_t j=1; j<6; j++) {  // 找到左边的界限left_flag，不超过往左5个点
                if (angle_pro_value[i-j] <= filter_buf[i-j]) {
                    left_flag = i-j;  // 一直到了最左边还符合条件，left_flag = i-5
                }
                else {
                    left_flag = i-j+1;  // 找到了边界，退出
                    break;
                }
            }
            for (std::size_t j=1; j<5; j++) {  // 找到中间的界限mid_flag
                if (angle_pro_value[i+j] >= filter_buf[i+j] && angle_pro_value[i+j] <= (180-filter_buf[i+j])) {
                    mid_flag = i+j;
                }
                else {
                    mid_flag = i+j-1;
                    break;
                }
            }
            for (std::size_t j=1; j<5; j++) {  // 找到右边的界限right_flag，不超过5个点
                if (angle_pro_value[mid_flag+j] >= (180-filter_buf[mid_flag+j])) {
                    right_flag = mid_flag+j;
                }
                else {
                    right_flag = mid_flag+j-1;
                }
            }
            std::vector<int> small_thing_dis_index;
            for (std::size_t j=i-1; j<mid_flag+2; j++) {  // 取出小物体角度中及其左右点的距离
                small_thing_dis_index.push_back(data.ranges_data[j]);
            }
            std::size_t min_value;
            for (std::size_t j=0; j<mid_flag-i+3; j++) {  // 给一个非0初值
                if (small_thing_dis_index[j] > 0)  {
                    min_value = small_thing_dis_index[j];
                    break;
                }
            }
            for (std::size_t j=0; j<mid_flag-i+3; j++) {  // 求非0最小值
                if ((static_cast<std::size_t>(small_thing_dis_index[j]) > 0) && (min_value > static_cast<std::size_t>(small_thing_dis_index[j]))) {
                    min_value = small_thing_dis_index[j];
                }
            }
            for (std::size_t j=0; j<right_flag-left_flag+1; j++) {
                if ((data.ranges_data[left_flag+j] - min_value) <= 30) {
                    filter_flag[left_flag+j] = 0;
                }
                else {
                    filter_flag[left_flag+j] = 1;
                }
            }
        }
        flag = 0;
    }

    // 处理物体边缘孤立拖尾点
    for (std::size_t i=1; i<point_num-1; i++) {
        if (data.ranges_data[i-1] != 0 && data.ranges_data[i+1] != 0 && abs(data.ranges_data[i]-data.ranges_data[i-1]) > 50 && abs(data.ranges_data[i+1]-data.ranges_data[i]) > 50) {
            filter_flag[i] = 1;
        }
    }

    for (std::size_t i=0; i<point_num; i++) {
        if (filter_flag[i]) {
            data.ranges_data[i] = 0;
            // data.intensities_data[i] = 0;
        }

        else {
            // data.intensities_data[i] = 200;
        }
    }
    // data.intensities_data[0] = 255;

}

void TrailingFilter::FilterH1(ScanData &data, uint16_t resolution)
{
 
    
    if (data.ranges_data.size() <= 10 || data.intensities_data.size() <= 10) {
        return;
    }
    
    std::size_t point_num = data.ranges_data.size();
    std::vector<double> angle_value;
    std::vector<double> angle_pro_value;
    std::vector<int> filter_buf;
    std::vector<int> diff_dis;
    angle_value.push_back(90.0f);
    std::size_t high_rssi = 1200;//4000
    std::size_t diff_rssi = 5;//50
    
    for (std::size_t i=1; i<point_num; i++) {
        filter_buf.push_back(15);  // 除反光条，角度阈值为15
        diff_dis.push_back(10);  // 除反光条，距离阈值为10
    }

    int widen_point_num = (point_num - 1)/270*5;  // 针对反光条两边分别拓宽点数

    for (std::size_t i=widen_point_num; i<point_num-static_cast<std::size_t>(widen_point_num); i++) {  // 反光条左右拓宽的点，角度阈值30，距离阈值5
        if (data.intensities_data[i]>=high_rssi && data.intensities_data[i+1]<=high_rssi) {
            for (std::size_t j=0; j<static_cast<std::size_t>(widen_point_num); j++) {
                filter_buf[i+j] = 30;
                diff_dis[i+j] = 5;
            }
        }
        else if (data.intensities_data[i]<=high_rssi && data.intensities_data[i+1]>=high_rssi) {
            for (std::size_t j=0; j<static_cast<std::size_t>(widen_point_num); j++) {
                filter_buf[i-j] = 30;
                diff_dis[i-j] = 5;
            }
        }
    }
    // 计算相邻点角度
    for (std::size_t i=1; i<point_num; i++) {
        if ((double)data.ranges_data[i-1] == (double)data.ranges_data[i] * cos(M_PI / 180 * double(resolution) / 10000)) {
            angle_value.push_back(90);
        }
        else {
            double temp_value = 180 / M_PI * atan((double)data.ranges_data[i] * sin(M_PI / 180 * (double)resolution / 10000) / \
                            ((double)data.ranges_data[i-1] - (double)data.ranges_data[i] * cos(M_PI / 180 * double(resolution) / 10000)));
        if (temp_value < 0) {
            temp_value += 180;
        }
        angle_value.push_back(temp_value);
        }
    }

    //纠正【非极小-极小-极大-非极大】数据，因为这是距离抖动造成的，将其与细小物体数据区分开
    std::vector<int> change_flag;
    for (std::size_t i=0; i<point_num; i++) {
        change_flag.push_back(0);
    }

    for (std::size_t i=5; i<point_num-5; i++) {
        //if ((angle_value[i-1] > filter_buf[i-1] || change_flag[i-1] == 1) && angle_value[i] <= filter_buf[i] && angle_value[i+1] >= (180-filter_buf[i+1]) && angle_value[i+2] < (180-filter_buf[i+2]))  // 识别数据：【非极小-极小-极大-非极大】
          
            uint8_t big_num = 0;
            uint8_t small_num = 0;
            for (std::size_t j=-5; j<5; j++) {
                if (angle_value[i+j] <= filter_buf[i+j]) {
                    small_num++;
                }
                else if (angle_value[i+j] >= (180-filter_buf[i+j])) {
                    big_num++;
                }
            if (big_num > small_num) {
                angle_value[i] = 180-angle_value[i];
            }
            else if (big_num < small_num) {
                angle_value[i+1] = 180-angle_value[i+1];
            }
            else {
                if ((angle_value[i]+angle_value[i+1]) <= 180) {
                    angle_value[i+1] = 180-angle_value[i+1];
                }
                else {
                    angle_value[i] = 180-angle_value[i];
                }
            }
            }
        
    }

    // 一群都是极大或都是极小的点中，不单调的个别点变为单调
    std::size_t start_index;
    std::size_t end_index;
    // printf("resolution:%f",double(resolution) / 10000);
    
    if (double(resolution) / 10000 == 0.1) {  // 角分辨率为0.1
        start_index = 5;
        end_index = 6;
    }
    else {  // 角分辨率为0.05
        start_index = 7;
        end_index = 8;
    }
    for (std::size_t i=0; i<start_index; i++) {  // 前start_index个角度直接拿过来
        angle_pro_value.push_back(angle_value[i]);
    }
    
    for (std::size_t i=start_index; i<point_num-end_index; i++) { //第start_index到-end_index个点
        uint8_t big_num = 0;
        uint8_t small_num = 0;
        std::vector<double> list;
        for(std::size_t j=i-start_index;j<i+end_index;j++){ //每个i-start_index到i+end_index之间的点放到列表
            list.push_back(angle_value[j]);
        }
        for (std::size_t j=0; j<start_index+end_index; j++) {
            if (list[j] <= filter_buf[i-start_index+j]) {
                small_num++;
            }
            else if (list[j] >= (180 - filter_buf[i-start_index+j])) {
                big_num++;
            }
        }
        if (small_num > 5 and (small_num - big_num) > 1 and angle_value[i] >= (180 - filter_buf[i])) {
            angle_pro_value.push_back(180 - angle_value[i]);
        }
        else if (big_num > 5 and (big_num - small_num) > 1 and angle_value[i] <= filter_buf[i]) {
            angle_pro_value.push_back(180 - angle_value[i]);
        }
        else {
            angle_pro_value.push_back(angle_value[i]);
        }
    }
    for (std::size_t i=point_num-end_index; i<point_num; i++) {  // 最后end_index个角度直接拿过来
        angle_pro_value.push_back(angle_value[i]);
    }
    
    double temp_buf[5];
    std::vector<double> median_value;
    std::vector<double> average_value;
    // 计算中值
    for (std::size_t i=0; i<point_num; i++) {
        // printf("angle_pro_value:%f",angle_pro_value[i]);
        if (i==0) {
            temp_buf[0] = 90.0f;
            temp_buf[1] = 90.0f;
            temp_buf[2] = angle_pro_value[i];
            temp_buf[3] = angle_pro_value[i+1];
            temp_buf[4] = angle_pro_value[i+2];
        }
        else if (i==1) {
            temp_buf[0] = 90.0f;
            temp_buf[1] = angle_pro_value[i];
            temp_buf[2] = angle_pro_value[i+1];
            temp_buf[3] = angle_pro_value[i+2];
            temp_buf[4] = angle_pro_value[i-1];
        }
        else if (i==point_num-2) {
            temp_buf[0] = angle_pro_value[i-2];
            temp_buf[1] = angle_pro_value[i-1];
            temp_buf[2] = angle_pro_value[i];
            temp_buf[3] = angle_pro_value[i+1];
            temp_buf[4] = 90.0f;
        }
        else if (i==point_num-1) {
            temp_buf[0] = angle_pro_value[i-2];
            temp_buf[1] = angle_pro_value[i-1];
            temp_buf[2] = angle_pro_value[i];
            temp_buf[3] = 90.0f;
            temp_buf[4] = 90.0f;
        }
        else {
            temp_buf[0] = angle_pro_value[i-2];
            temp_buf[1] = angle_pro_value[i-1];
            temp_buf[2] = angle_pro_value[i];
            temp_buf[3] = angle_pro_value[i+1];
            temp_buf[4] = angle_pro_value[i+2];
        }

        for (std::size_t j=0; j<4; j++) {
            for (std::size_t k=0; k<4-j; k++) {
                if (temp_buf[k] > temp_buf[k+1]) {
                    double temp_change = temp_buf[k];
                    temp_buf[k] = temp_buf[k+1];
                    temp_buf[k+1] = temp_change;
                }
            }
        }

        median_value.push_back(temp_buf[2]);
    }
    // 计算平均值
    // cout <<"point num = " << point_num << endl;
    for (std::size_t i=0; i<point_num; i++) {
        if (i==0) {
            temp_buf[0] = 90.0f;
            temp_buf[1] = 90.0f;
            temp_buf[2] = median_value[i];
            temp_buf[3] = median_value[i+1];
            temp_buf[4] = median_value[i+2];
        }
        else if (i==1) {
            temp_buf[0] = 90.0f;
            temp_buf[1] = median_value[i];
            temp_buf[2] = median_value[i+1];
            temp_buf[3] = median_value[i+2];
            temp_buf[4] = median_value[i-1];
        }
        else if (i==point_num-2) {
            temp_buf[0] = median_value[i-2];
            temp_buf[1] = median_value[i-1];
            temp_buf[2] = median_value[i];
            temp_buf[3] = median_value[i+1];
            temp_buf[4] = 90.0f;
        }
        else if (i==point_num-1) {
            temp_buf[0] = median_value[i-2];
            temp_buf[1] = median_value[i-1];
            temp_buf[2] = median_value[i];
            temp_buf[3] = 90.0f;
            temp_buf[4] = 90.0f;
        }
        else {
            temp_buf[0] = median_value[i-2];
            temp_buf[1] = median_value[i-1];
            temp_buf[2] = median_value[i];
            temp_buf[3] = median_value[i+1];
            temp_buf[4] = median_value[i+2];
        }

        double temp_average = 0.0f;
        for (std::size_t j=0; j<5; j++) {
            temp_average += temp_buf[j];
        }

        average_value.push_back(temp_average / 5);
    }
    // 滤掉同方向单个没被滤掉的点
    for (std::size_t i=1; i<point_num-1; i++) {
        if (average_value[i-1] >= (180 - filter_buf[i]) && average_value[i+1] >= (180 - filter_buf[i])) {
            average_value[i] = (180 - filter_buf[i]);
        }
        else if (average_value[i-1] <= filter_buf[i] && average_value[i+1] <= filter_buf[i]) {
            average_value[i] = filter_buf[i];
        }
    }
    // 滤掉同方向多个没被滤掉的点
    // int filter_point_num = (point_num-1)/270*5; //过滤反光柱两边葫芦形拖尾的点数范围
    // for (std::size_t i=0; i<point_num-1; i++) {
    //     if (average_value[i]<=filter_buf[i] && average_value[i+1]>filter_buf[i] && average_value[i+1]<=(180 - filter_buf[i])) {
    //         for (std::size_t j=2; j<filter_point_num; j++) {
    //             if ((i+j)>point_num-1) {
    //                 break;
    //             }
    //             if (average_value[i+j]>=(180 - filter_buf[i])) {
    //                 break;
    //             }
    //             else if (average_value[i+j]<=filter_buf[i]) {
    //                 for (std::size_t m=i; m<i+j; m++) {
    //                     average_value[m] = filter_buf[i];
    //                     }
    //                 break;
                    
    //             }
    //         }
    //     }
    //     else if (average_value[i]>=(180 - filter_buf[i]) && average_value[i+1]>=filter_buf[i] && average_value[i+1]<=(180 - filter_buf[i])) {
    //         for (std::size_t j=2; j<filter_point_num; j++) {
    //             if ((i+j)>point_num-1) {
    //                 break;
    //             }
    //             if (average_value[i+j]<=filter_buf[i]) {
    //                 break;
    //             }
    //             else if (average_value[i+j]>=(180 - filter_buf[i])) {
    //                 for (std::size_t m=i; m<i+j; m++) {
    //                     average_value[m] = (180 - filter_buf[i]);
    //                     }
    //                 break;
    //             }
    //         }
    //     }
    // }
    
    // 根据反射率补点
    std::vector<int> rssi_flag1;
    std::vector<int> rssi_flag2;
    
    for (std::size_t i=0; i<point_num-1; i++) {
        if (abs(data.intensities_data[i+1] - data.intensities_data[i]) >= diff_rssi && data.intensities_data[i] != 0 && data.intensities_data[i+1] != 0) {
            rssi_flag1.push_back(1);
        }
        else {
            rssi_flag1.push_back(0);
        }
    }
    rssi_flag1.push_back(0);
    // 11个里有3个flag就置为1
    for (std::size_t i=0; i<5; i++) {
        rssi_flag2.push_back(rssi_flag1[i]);
    }
    for (std::size_t i=5; i<point_num-5; i++) {
        int num = 0;
        for (std::size_t j=0; j<11; j++) {
            if (rssi_flag1[i-5+j] == 1) {
                num++;
            }
        }
        if (num >= 3) {
            rssi_flag2.push_back(1);
        }
        else {
            rssi_flag2.push_back(0);
        }
    }
    for (std::size_t i=point_num-5; i<point_num; i++) {
        rssi_flag2.push_back(rssi_flag1[i]);
    }

    // 根据距离补点
    std::vector<int> dis_flag1;
    std::vector<int> dis_flag2;
    
    for (std::size_t i=0; i<point_num-1; i++) {
        if (abs(data.ranges_data[i+1] - data.ranges_data[i]) >= diff_dis[i] && data.ranges_data[i] != 0 && data.ranges_data[i+1] != 0) {
            dis_flag1.push_back(1);
        }
        else {
            dis_flag1.push_back(0);
        }
    }
    dis_flag1.push_back(0);
    // 11个里有5个flag就置为1
    for (std::size_t i=0; i<5; i++) {
        dis_flag2.push_back(dis_flag1[i]);
    }
    for (std::size_t i=5; i<point_num-5; i++) {
        int num = 0;
        for (std::size_t j=0; j<11; j++) {
            if (dis_flag1[i-5+j] == 1) {
                num++;
            }
        }
        if (num >= 3) {
            dis_flag2.push_back(1);
        }
        else {
            dis_flag2.push_back(0);
        }
    }
    for (std::size_t i=point_num-5; i<point_num; i++) {
        dis_flag2.push_back(dis_flag1[i]);
    }
    for (std::size_t i=1; i<point_num-1; i++) { // 过滤dis_flag2中零星噪点
        if (dis_flag2[i-1] == 0 && dis_flag2[i] == 1 && dis_flag2[i+1] == 0 && dis_flag1[i] == 0) {
            dis_flag2[i] = 0;
        }
        else if (dis_flag2[i-1] == 1 && dis_flag2[i] == 0 && dis_flag2[i+1] == 1 && dis_flag1[i] == 1) {
            dis_flag2[i] = 1;
        }
    }

    std::vector<int> filter_flag;
    for (std::size_t i=0; i<point_num; i++) {
        if ((average_value[i] >= (180 - filter_buf[i]) || average_value[i] <= filter_buf[i]) && (dis_flag2[i] == 1 || rssi_flag2[i] == 1) && data.ranges_data[i] < 8000) {
            filter_flag.push_back(1);
        }
        else {
            filter_flag.push_back(0);
        }
    }

    // 处理小物体的拖尾，选中小物体及其拖尾，选其距离最小值30mm内的点保留下来【left_flag | i | mid_flag | right_flag】
    std::size_t flag = 0;
    std::size_t left_flag;
    std::size_t mid_flag;
    std::size_t right_flag;
    for (std::size_t i=0; i<point_num-11; i++) {
        if (angle_pro_value[i] <= filter_buf[i] && angle_pro_value[i+1] >= filter_buf[i+1] && angle_pro_value[i+1] <= (180-filter_buf[i+1])) { //判断是否符合小物体特征【极小-中-极大】
            for (std::size_t j=2; j<10; j++) {
                if (angle_pro_value[i+j] >= (180-filter_buf[i+j])) {
                    flag = 1;
                    break;
                }
            }
        }
        else if (angle_pro_value[i] <= filter_buf[i] && angle_pro_value[i+1] >= (180-filter_buf[i+1])) {//判断是否符合小物体特征【极小-极大】
            flag = 1;
        }
        if (flag == 1) {
            for (std::size_t j=1; j<6; j++) {  // 找到左边的界限left_flag，不超过往左5个点
                if (angle_pro_value[i-j] <= filter_buf[i-j]) {
                    left_flag = i-j;  // 一直到了最左边还符合条件，left_flag = i-5
                }
                else {
                    left_flag = i-j+1;  // 找到了边界，退出
                    break;
                }
            }
            for (std::size_t j=1; j<5; j++) {  // 找到中间的界限mid_flag
                if (angle_pro_value[i+j] >= filter_buf[i+j] && angle_pro_value[i+j] <= (180-filter_buf[i+j])) {
                    mid_flag = i+j;
                }
                else {
                    mid_flag = i+j-1;
                    break;
                }
            }
            for (std::size_t j=1; j<5; j++) {  // 找到右边的界限right_flag，不超过5个点
                if (angle_pro_value[mid_flag+j] >= (180-filter_buf[mid_flag+j])) {
                    right_flag = mid_flag+j;
                }
                else {
                    right_flag = mid_flag+j-1;
                }
            }
            std::vector<int> small_thing_dis_index;
            for (std::size_t j=i-1; j<mid_flag+2; j++) {  // 取出小物体角度中及其左右点的距离
                small_thing_dis_index.push_back(data.ranges_data[j]);
            }
            std::size_t min_value;
            for (std::size_t j=0; j<mid_flag-i+3; j++) {  // 给一个非0初值
                if (small_thing_dis_index[j] > 0)  {
                    min_value = small_thing_dis_index[j];
                    break;
                }
            }
            for (std::size_t j=0; j<mid_flag-i+3; j++) {  // 求非0最小值
                if ((small_thing_dis_index[j] > 0) && (min_value > static_cast<std::size_t>(small_thing_dis_index[j]))) {
                    min_value = small_thing_dis_index[j];
                }
            }
            for (std::size_t j=0; j<right_flag-left_flag+1; j++) {
                if ((data.ranges_data[left_flag+j] - min_value) <= 30) {
                    filter_flag[left_flag+j] = 0;
                }
                else {
                    filter_flag[left_flag+j] = 1;
                }
            }
        }
        flag = 0;
    }

    // 处理物体边缘孤立拖尾点
    for (std::size_t i=1; i<point_num-1; i++) {
        if (data.ranges_data[i-1] != 0 && data.ranges_data[i+1] != 0 && abs(data.ranges_data[i]-data.ranges_data[i-1]) > 30 && abs(data.ranges_data[i+1]-data.ranges_data[i]) > 30) {
            filter_flag[i] = 1;
        }
    }

    for (std::size_t i=0; i<point_num; i++) {
        if (filter_flag[i]) {
            data.ranges_data[i] = 0;
            // data.intensities_data[i] = 0;
        }

        else {
            // data.intensities_data[i] = 200;
        }
    }
    // data.intensities_data[0] = 255;

}

}