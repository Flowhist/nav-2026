#include <free_lidar/free_eth_driver.h>

#include <iostream>
#include <iomanip>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <cmath>
#include "rclcpp/time.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sys/time.h>

namespace free_optics {

FREEEthDriver::FREEEthDriver()
{
    std::cout << "====== " << "FREE ETH version: " << C2_ETH_ROS_VER << " ======" << std::endl;

    is_connected_ = false;
    is_capturing_ = false;

    scan_data_buf_.clear();
    scan_data_.ranges_data.clear();
    scan_data_.intensities_data.clear();
    data.clear();

    byte_count_ = 0;

   //initlog()；

    struct timeval tv_packet;
 gettimeofday(&tv_packet, nullptr);
    last_packet_time_ = tv_packet;
};

FREEEthDriver::~FREEEthDriver()
{
    scan_data_buf_.clear();
    scan_data_.ranges_data.clear();
    scan_data_.intensities_data.clear();
    data.clear();

    disconnect();
}

bool FREEEthDriver::connect(const std::string hostname, int port)
{
    socket_fd_ = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);

    if (socket_fd_ < 0) {
        std::cerr << "Creating socket failed: " << socket_fd_ << std::endl;
        is_connected_ = false;
        return false;
    }

    struct timeval timeout;
    timeout.tv_sec = 5;
    timeout.tv_usec = 0;
    setsockopt(socket_fd_, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    struct sockaddr_in sock_addr;
    memset(&sock_addr, 0, sizeof(sockaddr_in));
    sock_addr.sin_family = PF_INET;
    sock_addr.sin_port = htons(port);
    sock_addr.sin_addr.s_addr = inet_addr(hostname.c_str());

    if (::connect(socket_fd_, (struct sockaddr *) &sock_addr, sizeof(sock_addr)) < 0) {
        is_connected_ = false;
        return false;
    }

    is_connected_ = true;
    return true;
}

void FREEEthDriver::disconnect()
{
    if (is_capturing_ == true) {
        getScanData(0);
    }

    if (is_connected_ == true) {
        int ret = close(socket_fd_);

        if (ret == 0) {
            std::cout << "Socket close success!" << std::endl;
        }
        else {
            std::cerr << "Socket close failed!" << std::endl;
        }
    }

    is_connected_ = false;
    is_capturing_ = false;
}

void FREEEthDriver::ClearBuf()
{
    struct timeval tmOut;
    tmOut.tv_sec = 1.5;
    tmOut.tv_usec = 0;
    fd_set         fds;
    FD_ZERO(&fds);
    FD_SET(socket_fd_, &fds);
    int    nRet;
    char tmp[65535];
    memset(tmp, 0, sizeof(tmp));

    getScanData(0);
    while(1)
    {
        nRet= select(FD_SETSIZE, &fds, NULL, NULL, &tmOut);
        if(nRet== 0)
            break;
        recv(socket_fd_, tmp, 65535,0);
    }
    std::cerr << "Clear Buf!" << std::endl;
}

bool FREEEthDriver::getDeviceState(uint8_t &state)
{
    const size_t cmd_length = 9;
    uint8_t tx_buf[50] = {0,};
    uint8_t check_sum = 0;

    for (std::size_t i=0; i<4; i++) {
        tx_buf[i] = 0x02;
    }
    
    tx_buf[4] = (uint8_t)((cmd_length & 0xff00) >> 8);
    tx_buf[5] = (uint8_t)(cmd_length & 0x00ff);
    tx_buf[6] = METHOD_DOWN;
    tx_buf[7] = GET_STATE;

    for (std::size_t i=0; i<cmd_length-1; i++) {
        check_sum += tx_buf[i];
    }

    tx_buf[cmd_length-1] = check_sum;

    fd_set read_set;
    struct timeval time_out;

    FD_ZERO(&read_set);
    FD_SET(socket_fd_, &read_set);

    time_out.tv_sec = 1;
    time_out.tv_usec = 0;

    if (write(socket_fd_, tx_buf, cmd_length) != cmd_length) {
        std::cerr << "Socket write failed!" << std::endl;
        return false;
    }

    if (select(socket_fd_+1, &read_set, NULL, NULL, &time_out) <= 0) {
        std::cerr << "Socket read timeout!" << std::endl;
        return false;
    }

    uint8_t rx_buf[50] = {0,};
    ssize_t len = read(socket_fd_, rx_buf, 50);

    if (len <= 0) {
        std::cerr << "Socket read error: " << len << std::endl;
        return false;
    }

    check_sum = 0;
    for (std::size_t i=0; i< static_cast<std::size_t>(len-1); i++) {
        check_sum += rx_buf[i];
    }

    bool result = true;

    if (rx_buf[0] != 0x02 || rx_buf[1] != 0x02 || rx_buf[2] != 0x02 || rx_buf[3] != 0x02 || len != 10) {
        std::cerr << "Invalid packet!" << std::endl;
        result = false;
    }
    else if (rx_buf[6] != METHOD_UP) {
        std::cerr << "Invalid opt code!" << std::endl;
        result = false;
    }
    else if (rx_buf[7] != GET_STATE) {
        std::cerr << "Invalid cmd code!" << std::endl;
        result = false;
    }
    else if (rx_buf[len-1] != check_sum) {
        std::cerr << "Invalid check sum!" << std::endl;
        result = false;
    }

    if (result == false) {
        std::cout << "RX: ";

        for (std::size_t i=0; i< static_cast<std::size_t>(len); i++) {
            std::cout << " " << std::hex << (int)rx_buf[i];
        }

        std::cout << std::endl;
    }
    else {
        state = rx_buf[8];
        switch (rx_buf[8]) {
            case 0x00: std::cout << "Scanner is busy! ";break;
            case 0x01: std::cout << "Scanner is ready!" << std::endl;break;
            case 0x02: std::cout << "Scanner is error! ";break;
            case 0x03: std::cout << "High temp error! ";break;
            case 0x04: std::cout << "High volt error! ";break;
            case 0x05: std::cout << "Pulse width error! ";break;
            case 0x06: std::cout << "Scanner is ready!" << std::endl;break;
            case 0x07: std::cout << "Optical cover is blocked! ";break;
            default: std::cout << "Scanner is in another state! ";break;
        }
    }

    return result;
}

bool FREEEthDriver::getDeviceName(std::string &name)
{
    const size_t cmd_length = 9;
    uint8_t tx_buf[50] = {0,};
    uint8_t check_sum = 0;

    for (std::size_t i=0; i<4; i++) {
        tx_buf[i] = 0x02;
    }
    
    tx_buf[4] = (uint8_t)((cmd_length & 0xff00) >> 8);
    tx_buf[5] = (uint8_t)(cmd_length & 0x00ff);
    tx_buf[6] = READ_DOWN;
    tx_buf[7] = GET_NAME;

    for (std::size_t i=0; i<cmd_length-1; i++) {
        check_sum += tx_buf[i];
    }

    tx_buf[cmd_length-1] = check_sum;

    fd_set read_set;
    struct timeval time_out;

    FD_ZERO(&read_set);
    FD_SET(socket_fd_, &read_set);

    time_out.tv_sec = 1;
    time_out.tv_usec = 0;

    if (write(socket_fd_, tx_buf, cmd_length) != cmd_length) {
        std::cerr << "Socket write failed!" << std::endl;
        return false;
    }

    if (select(socket_fd_+1, &read_set, NULL, NULL, &time_out) <= 0) {
        std::cerr << "Socket read timeout!" << std::endl;
        return false;
    }

    uint8_t rx_buf[50] = {0,};
    ssize_t len = read(socket_fd_, rx_buf, 50);

    if (len <= 0) {
        std::cerr << "Socket read error: " << len << std::endl;
        return false;
    }

    check_sum = 0;
    for (std::size_t i=0; i< static_cast<std::size_t>(len-1); i++) {
        check_sum += rx_buf[i];
    }

    bool result = true;

    if (rx_buf[0] != 0x02 || rx_buf[1] != 0x02 || rx_buf[2] != 0x02 || rx_buf[3] != 0x02 || len != 13) {
        std::cerr << "Invalid packet!" << std::endl;
        result = false;
    }
    else if (rx_buf[6] != READ_UP) {
        std::cerr << "Invalid opt code!" << std::endl;
        result = false;
    }
    else if (rx_buf[7] != GET_NAME) {
        std::cerr << "Invalid cmd code!" << std::endl;
        result = false;
    }
    else if (rx_buf[len-1] != check_sum) {
        std::cerr << "Invalid check sum!" << std::endl;
        result = false;
    }

    if (result == false) {
        std::cout << "RX: ";

        for (std::size_t i=0; i< static_cast<std::size_t>(len); i++) {
            std::cout << " " << std::hex << (int)rx_buf[i];
        }

        std::cout << std::endl;
    }
    else {
        name.resize(4);
        name[0] = rx_buf[8];
        name[1] = rx_buf[9];
        name[2] = rx_buf[10];
        name[3] = rx_buf[11];
        std::cout << "Device name: " << name << std::endl;

    }

    return result;
}

bool FREEEthDriver::getFirmwareVersion(uint32_t &version)
{
    const size_t cmd_length = 9;
    uint8_t tx_buf[50] = {0,};
    uint8_t check_sum = 0;

    for (std::size_t i=0; i<4; i++) {
        tx_buf[i] = 0x02;
    }
    
    tx_buf[4] = (uint8_t)((cmd_length & 0xff00) >> 8);
    tx_buf[5] = (uint8_t)(cmd_length & 0x00ff);
    tx_buf[6] = METHOD_DOWN;
    tx_buf[7] = GET_FW_VER;

    for (std::size_t i=0; i<cmd_length-1; i++) {
        check_sum += tx_buf[i];
    }

    tx_buf[cmd_length-1] = check_sum;

    fd_set read_set;
    struct timeval time_out;

    FD_ZERO(&read_set);
    FD_SET(socket_fd_, &read_set);

    time_out.tv_sec = 1;
    time_out.tv_usec = 0;

    if (write(socket_fd_, tx_buf, cmd_length) != cmd_length) {
        std::cerr << "Socket write failed!" << std::endl;
        return false;
    }

    if (select(socket_fd_+1, &read_set, NULL, NULL, &time_out) <= 0) {
        std::cerr << "Socket read timeout!" << std::endl;
        return false;
    }

    uint8_t rx_buf[50] = {0,};
    ssize_t len = read(socket_fd_, rx_buf, 50);

    if (len <= 0) {
        std::cerr << "Socket read error: " << len << std::endl;
        return false;
    }

    check_sum = 0;
    for (std::size_t i=0; i< static_cast<std::size_t>(len-1); i++) {
        check_sum += rx_buf[i];
    }

    bool result = true;

    if (rx_buf[0] != 0x02 || rx_buf[1] != 0x02 || rx_buf[2] != 0x02 || rx_buf[3] != 0x02 || len != 13) {
        std::cerr << "Invalid packet!" << std::endl;
        result = false;
    }
    else if (rx_buf[6] != METHOD_UP) {
        std::cerr << "Invalid opt code!" << std::endl;
        result = false;
    }
    else if (rx_buf[7] != GET_FW_VER) {
        std::cerr << "Invalid cmd code!" << std::endl;
        result = false;
    }
    else if (rx_buf[len-1] != check_sum) {
        std::cerr << "Invalid check sum!" << std::endl;
        result = false;
    }
    
    if (result == false) {
        std::cout << "RX: ";

        for (std::size_t i=0; i< static_cast<std::size_t>(len); i++) {
            std::cout << " " << std::hex << (int)rx_buf[i];
        }

        std::cout << std::endl;
    }
    else {
        version = (((uint32_t)rx_buf[8] << 24) & 0xff000000) + 
                  (((uint32_t)rx_buf[9] << 16) & 0x00ff0000) + 
                  (((uint32_t)rx_buf[10] << 8) & 0x0000ff00) + 
                  (((uint32_t)rx_buf[11]) & 0x000000ff);
        std::cout << "Firmware version: " << std::hex << std::setfill('0') << std::setw(8) << version << std::endl;
    }

    return result;
}

bool FREEEthDriver::getSerialNumber(uint32_t &sn)
{
    const size_t cmd_length = 9;
    uint8_t tx_buf[50] = {0,};
    uint8_t check_sum = 0;

    for (std::size_t i=0; i<4; i++) {
        tx_buf[i] = 0x02;
    }
    
    tx_buf[4] = (uint8_t)((cmd_length & 0xff00) >> 8);
    tx_buf[5] = (uint8_t)(cmd_length & 0x00ff);
    tx_buf[6] = READ_DOWN;
    tx_buf[7] = GET_SN;

    for (std::size_t i=0; i<cmd_length-1; i++) {
        check_sum += tx_buf[i];
    }

    tx_buf[cmd_length-1] = check_sum;

    fd_set read_set;
    struct timeval time_out;

    FD_ZERO(&read_set);
    FD_SET(socket_fd_, &read_set);

    time_out.tv_sec = 1;
    time_out.tv_usec = 0;

    if (write(socket_fd_, tx_buf, cmd_length) != cmd_length) {
        std::cerr << "Socket write failed!" << std::endl;
        return false;
    }

    if (select(socket_fd_+1, &read_set, NULL, NULL, &time_out) <= 0) {
        std::cerr << "Socket read timeout!" << std::endl;
        return false;
    }

    uint8_t rx_buf[50] = {0,};
    ssize_t len = read(socket_fd_, rx_buf, 50);

    if (len <= 0) {
        std::cerr << "Socket read error: " << len << std::endl;
        return false;
    }

    check_sum = 0;
    for (std::size_t i=0; i< static_cast<std::size_t>(len-1); i++) {
        check_sum += rx_buf[i];
    }

    bool result = true;

    if (rx_buf[0] != 0x02 || rx_buf[1] != 0x02 || rx_buf[2] != 0x02 || rx_buf[3] != 0x02 || len != 13) {
        std::cerr << "Invalid packet!" << std::endl;
        result = false;
    }
    else if (rx_buf[6] != READ_UP) {
        std::cerr << "Invalid opt code!" << std::endl;
        result = false;
    }
    else if (rx_buf[7] != GET_SN) {
        std::cerr << "Invalid cmd code!" << std::endl;
        result = false;
    }
    else if (rx_buf[len-1] != check_sum) {
        std::cerr << "Invalid check sum!" << std::endl;
        result = false;
    }
    
    if (result == false) {
        std::cout << "RX: ";

        for (std::size_t i=0; i< static_cast<std::size_t>(len); i++) {
            std::cout << " " << std::hex << (int)rx_buf[i];
        }

        std::cout << std::endl;
    }
    else {
        sn = (((uint32_t)rx_buf[8] << 24) & 0xff000000) + 
             (((uint32_t)rx_buf[9] << 16) & 0x00ff0000) + 
             (((uint32_t)rx_buf[10] << 8) & 0x0000ff00) + 
             (((uint32_t)rx_buf[11]) & 0x000000ff);
        std::cout << "Serial number: WL20" << std::hex << std::setfill('0') 
                  << std::setw(8) << sn << "-C2" << std::endl;
    }

    return result;
}

bool FREEEthDriver::login(uint8_t &state)
{
    const size_t cmd_length = 14;
    uint8_t tx_buf[50] = {0,};
    uint8_t check_sum = 0;

    for (std::size_t i=0; i<4; i++) {
        tx_buf[i] = 0x02;
    }
    
    tx_buf[4] = (uint8_t)((cmd_length & 0xff00) >> 8);
    tx_buf[5] = (uint8_t)(cmd_length & 0x00ff);
    tx_buf[6] = METHOD_DOWN;
    tx_buf[7] = LOGIN;
    tx_buf[8] = 0x02;
    tx_buf[9] = login_psw[0];
    tx_buf[10] = login_psw[1];
    tx_buf[11] = login_psw[2];
    tx_buf[12] = login_psw[3];

    for (std::size_t i=0; i<cmd_length-1; i++) {
        check_sum += tx_buf[i];
    }

    tx_buf[cmd_length-1] = check_sum;

    fd_set read_set;
    struct timeval time_out;

    FD_ZERO(&read_set);
    FD_SET(socket_fd_, &read_set);

    time_out.tv_sec = 1;
    time_out.tv_usec = 0;

    if (write(socket_fd_, tx_buf, cmd_length) != cmd_length) {
        std::cerr << "Socket write failed!" << std::endl;
        return false;
    }

    if (select(socket_fd_+1, &read_set, NULL, NULL, &time_out) <= 0) {
        std::cerr << "Socket read timeout!" << std::endl;
        return false;
    }

    uint8_t rx_buf[50] = {0,};
    ssize_t len = read(socket_fd_, rx_buf, 50);

    if (len <= 0) {
        std::cerr << "Socket read error: " << len << std::endl;
        return false;
    }

    check_sum = 0;
    for (std::size_t i=0; i< static_cast<std::size_t>(len-1); i++) {
        check_sum += rx_buf[i];
    }

    bool result = true;

    if (rx_buf[0] != 0x02 || rx_buf[1] != 0x02 || rx_buf[2] != 0x02 || rx_buf[3] != 0x02 || len != 10) {
        std::cerr << "Invalid packet!" << std::endl;
        result = false;
    }
    else if (rx_buf[6] != METHOD_UP) {
        std::cerr << "Invalid opt code!" << std::endl;
        result = false;
    }
    else if (rx_buf[7] != LOGIN) {
        std::cerr << "Invalid cmd code!" << std::endl;
        result = false;
    }
    else if (rx_buf[len-1] != check_sum) {
        std::cerr << "Invalid check sum!" << std::endl;
        result = false;
    }

    state = rx_buf[8];
    if (rx_buf[8] != 0x01) {
        std::cerr << "Log in failed!" << std::endl;
    }

    if (result == false) {
        std::cout << "RX: ";

        for (std::size_t i=0; i< static_cast<std::size_t>(len); i++) {
            std::cout << " " << std::hex << (int)rx_buf[i];
        }

        std::cout << std::endl;
    }
    else {
        std::cout << "Log in success." << std::endl;
    }

    return result;
}

bool FREEEthDriver::getScanAngle(int32_t &start_angle, int32_t &stop_angle)
{
    const size_t cmd_length = 9;
    uint8_t tx_buf[50] = {0,};
    uint8_t check_sum = 0;

    for (std::size_t i=0; i<4; i++) {
        tx_buf[i] = 0x02;
    }
    
    tx_buf[4] = (uint8_t)((cmd_length & 0xff00) >> 8);
    tx_buf[5] = (uint8_t)(cmd_length & 0x00ff);
    tx_buf[6] = READ_DOWN;
    tx_buf[7] = GET_ANGLE;

    for (std::size_t i=0; i<cmd_length-1; i++) {
        check_sum += tx_buf[i];
    }

    tx_buf[cmd_length-1] = check_sum;

    fd_set read_set;
    struct timeval time_out;

    FD_ZERO(&read_set);
    FD_SET(socket_fd_, &read_set);

    time_out.tv_sec = 1;
    time_out.tv_usec = 0;

    if (write(socket_fd_, tx_buf, cmd_length) != cmd_length) {
        std::cerr << "Socket write failed!" << std::endl;
        return false;
    }

    if (select(socket_fd_+1, &read_set, NULL, NULL, &time_out) <= 0) {
        std::cerr << "Socket read timeout!" << std::endl;
        return false;
    }

    uint8_t rx_buf[50] = {0,};
    ssize_t len = read(socket_fd_, rx_buf, 50);

    if (len <= 0) {
        std::cerr << "Socket read error: " << len << std::endl;
        return false;
    }

    check_sum = 0;
    for (std::size_t i=0; i< static_cast<std::size_t>(len-1); i++) {
        check_sum += rx_buf[i];
    }

    bool result = true;

    if (rx_buf[0] != 0x02 || rx_buf[1] != 0x02 || rx_buf[2] != 0x02 || rx_buf[3] != 0x02 || len != 18) {
        std::cerr << "Invalid packet!" << std::endl;
        result = false;
    }
    else if (rx_buf[6] != READ_UP) {
        std::cerr << "Invalid opt code!" << std::endl;
        result = false;
    }
    else if (rx_buf[7] != GET_ANGLE) {
        std::cerr << "Invalid cmd code!" << std::endl;
        result = false;
    }
    else if (rx_buf[len-1] != check_sum) {
        std::cerr << "Invalid check sum!" << std::endl;
        result = false;
    }

    if (result == false) {
        std::cout << "RX: ";

        for (std::size_t i=0; i< static_cast<std::size_t>(len); i++) {
            std::cout << " " << std::hex << (int)rx_buf[i];
        }

        std::cout << std::endl;
    }
    else {
        start_angle = (((int32_t)rx_buf[9] << 24) & 0xff000000) + 
                      (((int32_t)rx_buf[10] << 16) & 0x00ff0000) + 
                      (((int32_t)rx_buf[11] << 8) & 0x0000ff00) + 
                      (((int32_t)rx_buf[12]) & 0x000000ff);

        stop_angle = (((int32_t)rx_buf[13] << 24) & 0xff000000) + 
                     (((int32_t)rx_buf[14] << 16) & 0x00ff0000) + 
                     (((int32_t)rx_buf[15] << 8) & 0x0000ff00) + 
                     (((int32_t)rx_buf[16]) & 0x000000ff);
        std::cout << "Get scan angle success, start angle: " << ((double)start_angle)/10000 
                  << ", stop angle: " << ((double)stop_angle)/10000 << std::endl;
    }

    return result;
}

bool FREEEthDriver::setScanAngle(int32_t start_angle, int32_t stop_angle)
{
    const size_t cmd_length = 18;
    uint8_t tx_buf[50] = {0,};
    uint8_t check_sum = 0;

    for (std::size_t i=0; i<4; i++) {
        tx_buf[i] = 0x02;
    }
    
    tx_buf[4] = (uint8_t)((cmd_length & 0xff00) >> 8);
    tx_buf[5] = (uint8_t)(cmd_length & 0x00ff);
    tx_buf[6] = WRITE_DOWN;
    tx_buf[7] = SET_ANGLE;
    tx_buf[8] = 0x02;
    tx_buf[9] = (uint8_t)((start_angle & 0xff000000) >> 24);
    tx_buf[10] = (uint8_t)((start_angle & 0x00ff0000) >> 16);
    tx_buf[11] = (uint8_t)((start_angle & 0x0000ff00) >> 8);
    tx_buf[12] = (uint8_t)((start_angle & 0x000000ff));
    tx_buf[13] = (uint8_t)((stop_angle & 0xff000000) >> 24);
    tx_buf[14] = (uint8_t)((stop_angle & 0x00ff0000) >> 16);
    tx_buf[15] = (uint8_t)((stop_angle & 0x0000ff00) >> 8);
    tx_buf[16] = (uint8_t)((stop_angle & 0x000000ff));

    for (std::size_t i=0; i<cmd_length-1; i++) {
        check_sum += tx_buf[i];
    }

    tx_buf[cmd_length-1] = check_sum;

    fd_set read_set;
    struct timeval time_out;

    FD_ZERO(&read_set);
    FD_SET(socket_fd_, &read_set);

    time_out.tv_sec = 1;
    time_out.tv_usec = 0;

    if (write(socket_fd_, tx_buf, cmd_length) != cmd_length) {
        std::cerr << "Socket write failed!" << std::endl;
        return false;
    }

    if (select(socket_fd_+1, &read_set, NULL, NULL, &time_out) <= 0) {
        std::cerr << "Socket read timeout!" << std::endl;
        return false;
    }

    uint8_t rx_buf[50] = {0,};
    ssize_t len = read(socket_fd_, rx_buf, 50);

    if (len <= 0) {
        std::cerr << "Socket read error: " << len << std::endl;
        return false;
    }

    check_sum = 0;
    for (std::size_t i=0; i< static_cast<std::size_t>(len-1); i++) {
        check_sum += rx_buf[i];
    }

    bool result = true;

    if (rx_buf[0] != 0x02 || rx_buf[1] != 0x02 || rx_buf[2] != 0x02 || rx_buf[3] != 0x02 || len != 18) {
        std::cerr << "Invalid packet!" << std::endl;
        result = false;
    }
    else if (rx_buf[6] != WRITE_UP) {
        std::cerr << "Invalid opt code!" << std::endl;
        result = false;
    }
    else if (rx_buf[7] != SET_ANGLE) {
        std::cerr << "Invalid cmd code!" << std::endl;
        result = false;
    }
    else if (rx_buf[len-1] != check_sum) {
        std::cerr << "Invalid check sum!" << std::endl;
        result = false;
    }

    int32_t tmp_start, tmp_stop;
    tmp_start = (((int32_t)rx_buf[9] << 24) & 0xff000000) + 
                (((int32_t)rx_buf[10] << 16) & 0x00ff0000) + 
                (((int32_t)rx_buf[11] << 8) & 0x0000ff00) + 
                (((int32_t)rx_buf[12]) & 0x000000ff);

    tmp_stop = (((int32_t)rx_buf[13] << 24) & 0xff000000) + 
               (((int32_t)rx_buf[14] << 16) & 0x00ff0000) + 
               (((int32_t)rx_buf[15] << 8) & 0x0000ff00) + 
               (((int32_t)rx_buf[16]) & 0x000000ff);

    if (tmp_start != start_angle || tmp_stop != stop_angle) {
        std::cerr << "Set scan angle failed!" << std::endl;
        result = false;
    }

    if (result == false) {
        std::cout << "RX: ";

        for (std::size_t i=0; i< static_cast<std::size_t>(len); i++) {
            std::cout << " " << std::hex << (int)rx_buf[i];
        }

        std::cout << std::endl;
    }
    else {
        std::cout << "Set start angle: " << std::dec << start_angle 
                  << ", stop angle: " << stop_angle << std::endl;
    }

    return result;
}

bool FREEEthDriver::setReflectivityNormalization(uint8_t normalizationFactor)
{
    const size_t cmd_length = 10; // 命令长度需根据实际协议进行调整
    uint8_t tx_buf[50] = {0,};
    uint8_t check_sum = 0;

    // 填充协议头，具体协议头内容需根据设备协议确定
    for (std::size_t i = 0; i < 4; i++) {
        tx_buf[i] = 0x02; // 协议头为0x02重复四次
    }

    // 填充命令长度等信息，需根据具体协议进行调整
    tx_buf[4] = (uint8_t)((cmd_length & 0xff00) >> 8); // 命令长度高位
    tx_buf[5] = (uint8_t)(cmd_length & 0x00ff); // 命令长度低位
    tx_buf[6] = WRITE_DOWN; // 操作码为WRITE_DOWN，需根据实际协议确定
    tx_buf[7] = SET_NORSWITCH; // 命令码，需根据实际协议确定
    tx_buf[8] = normalizationFactor; // 反射率归一化因子

    // 计算校验和
    for (std::size_t i = 0; i < cmd_length - 1; i++) {
        check_sum += tx_buf[i];
    }
    tx_buf[cmd_length - 1] = check_sum;

    fd_set read_set;
    struct timeval time_out;

    FD_ZERO(&read_set);
    FD_SET(socket_fd_, &read_set);

    time_out.tv_sec = 1;
    time_out.tv_usec = 0;

    // 发送数据
    if (write(socket_fd_, tx_buf, cmd_length) != cmd_length) {
        std::cerr << "Socket write failed!" << std::endl;
        return false;
    }

    // 等待接收数据
    if (select(socket_fd_ + 1, &read_set, NULL, NULL, &time_out) <= 0) {
        std::cerr << "Socket read timeout!" << std::endl;
        return false;
    }

    uint8_t rx_buf[50] = {0,};
    ssize_t len = read(socket_fd_, rx_buf, 50);

    if (len <= 0) {
        std::cerr << "Socket read error: " << len << std::endl;
        return false;
    }

    // 验证接收到的数据
    check_sum = 0;
    for (std::size_t i = 0; i < static_cast<std::size_t>(len - 1); i++) {
        check_sum += rx_buf[i];
    }

    bool result = true;

    // 验证协议头、操作码、命令码等信息，具体验证内容需根据协议确定
    if (rx_buf[0] != 0x02 || rx_buf[1] != 0x02 || rx_buf[2] != 0x02 || rx_buf[3] != 0x02 || len != 10) {
        std::cerr << "Invalid packet!" << std::endl;
        result = false;
    }
    else if (rx_buf[6] != WRITE_UP) {
        std::cerr << "Invalid opt code!" << std::endl;
        result = false;
    }
    else if (rx_buf[7] != SET_NORSWITCH) {
        std::cerr << "Invalid cmd code!" << std::endl;
        result = false;
    }
    else if (rx_buf[len - 1] != check_sum) {
        std::cerr << "Invalid check sum!" << std::endl;
        result = false;
    }

    // 验证反射率归一化因子是否设置成功
    uint8_t tmp_normalizationFactor = rx_buf[8];
    if (tmp_normalizationFactor != normalizationFactor) {
        std::cerr << "Set reflectivity normalization failed!" << std::endl;
        result = false;
    }

    // 输出接收数据和结果
    if (result == false) {
        std::cout << "RX: ";
        for (std::size_t i = 0; i < static_cast<std::size_t>(len); i++) {
            std::cout << " " << std::hex << (int)rx_buf[i];
        }
        std::cout << std::endl;
    }
    else {
        std::cout << "Set reflectivity normalization factor: " << std::dec << (int)normalizationFactor << std::endl;
    }

    return result;
}

bool FREEEthDriver::setScanConfig(uint16_t frequency, uint16_t resolution)
{
    const size_t cmd_length = 13;
    uint8_t tx_buf[50] = {0,};
    uint8_t check_sum = 0;

    for (std::size_t i=0; i<4; i++) {
        tx_buf[i] = 0x02;
    }
    
    tx_buf[4] = (uint8_t)((cmd_length & 0xff00) >> 8);
    tx_buf[5] = (uint8_t)(cmd_length & 0x00ff);
    tx_buf[6] = WRITE_DOWN;
    tx_buf[7] = SET_CONFIG;
    tx_buf[8] = (uint8_t)((frequency & 0xff00) >> 8);
    tx_buf[9] = (uint8_t)(frequency & 0x00ff);
    tx_buf[10] = (uint8_t)((resolution & 0xff00) >> 8);
    tx_buf[11] = (uint8_t)(resolution & 0x00ff);

    for (std::size_t i=0; i<cmd_length-1; i++) {
        check_sum += tx_buf[i];
    }

    tx_buf[cmd_length-1] = check_sum;

    fd_set read_set;
    struct timeval time_out;

    FD_ZERO(&read_set);
    FD_SET(socket_fd_, &read_set);

    time_out.tv_sec = 1;
    time_out.tv_usec = 0;

    if (write(socket_fd_, tx_buf, cmd_length) != cmd_length) {
        std::cerr << "Socket write failed!" << std::endl;
        return false;
    }

    if (select(socket_fd_+1, &read_set, NULL, NULL, &time_out) <= 0) {
        std::cerr << "Socket read timeout!" << std::endl;
        return false;
    }

    uint8_t rx_buf[50] = {0,};
    ssize_t len = read(socket_fd_, rx_buf, 50);

    if (len <= 0) {
        std::cerr << "Socket read error: " << len << std::endl;
        return false;
    }
    
    check_sum = 0;
    for (std::size_t i=0; i< static_cast<std::size_t>(len-1); i++) {
        check_sum += rx_buf[i];
    }

    bool result = true;

    if (rx_buf[0] != 0x02 || rx_buf[1] != 0x02 || rx_buf[2] != 0x02 || rx_buf[3] != 0x02 || len != 13) {
        std::cerr << "Invalid packet!" << std::endl;
        result = false;
    }
    else if (rx_buf[6] != WRITE_UP) {
        std::cerr << "Invalid opt code!" << std::endl;
        result = false;
    }
    else if (rx_buf[7] != SET_CONFIG) {
        std::cerr << "Invalid cmd code!" << std::endl;
        result = false;
    }
    else if (rx_buf[len-1] != check_sum) {
        std::cerr << "Invalid check sum!" << std::endl;
        result = false;
    }

    uint16_t tmp_frequency, tmp_resolution;
    tmp_frequency = (((uint16_t)rx_buf[8] << 8) & 0xff00) + 
                    (((uint16_t)rx_buf[9]) & 0x00ff);

    tmp_resolution = (((uint16_t)rx_buf[10] << 8) & 0xff00) + 
                     (((uint16_t)rx_buf[11]) & 0x00ff);

    if (tmp_frequency != frequency || tmp_resolution != resolution) {
        std::cerr << "Set scan config failed!" << std::endl;
        result = false;
    }

    if (result == false) {
        std::cout << "RX: ";

        for (std::size_t i=0; i< static_cast<std::size_t>(len); i++) {
            std::cout << " " << std::hex << (int)rx_buf[i];
        }

        std::cout << std::endl;
    }
    else {
        std::cout << "Set scan frequency: " << std::dec << tmp_frequency 
                  << ", resolution: " << tmp_resolution << std::endl;
    }

    return result;
}

bool FREEEthDriver::getMeasureRange(float &range_min, float &range_max)
{
    const size_t cmd_length = 9;
    uint8_t tx_buf[50] = {0,};
    uint8_t check_sum = 0;

    for (std::size_t i=0; i<4; i++) {
        tx_buf[i] = 0x02;
    }
    
    tx_buf[4] = (uint8_t)((cmd_length & 0xff00) >> 8);
    tx_buf[5] = (uint8_t)(cmd_length & 0x00ff);
    tx_buf[6] = READ_DOWN;
    tx_buf[7] = GET_RANGE;

    for (std::size_t i=0; i<cmd_length-1; i++) {
        check_sum += tx_buf[i];
    }

    tx_buf[cmd_length-1] = check_sum;

    fd_set read_set;
    struct timeval time_out;

    FD_ZERO(&read_set);
    FD_SET(socket_fd_, &read_set);

    time_out.tv_sec = 1;
    time_out.tv_usec = 0;

    if (write(socket_fd_, tx_buf, cmd_length) != cmd_length) {
        std::cerr << "Socket write failed!" << std::endl;
        return false;
    }

    if (select(socket_fd_+1, &read_set, NULL, NULL, &time_out) <= 0) {
        std::cerr << "Socket read timeout!" << std::endl;
        return false;
    }

    uint8_t rx_buf[50] = {0,};
    ssize_t len = read(socket_fd_, rx_buf, 50);

    if (len <= 0) {
        std::cerr << "Socket read error: " << len << std::endl;
        return false;
    }

    check_sum = 0;
    for (std::size_t i=0; i< static_cast<std::size_t>(len-1); i++) {
        check_sum += rx_buf[i];
    }

    bool result = true;

    if (rx_buf[0] != 0x02 || rx_buf[1] != 0x02 || rx_buf[2] != 0x02 || rx_buf[3] != 0x02 || len != 13) {
        std::cerr << "Invalid packet!" << std::endl;
        result = false;
    }
    else if (rx_buf[6] != READ_UP) {
        std::cerr << "Invalid opt code!" << std::endl;
        result = false;
    }
    else if (rx_buf[7] != GET_RANGE) {
        std::cerr << "Invalid cmd code!" << std::endl;
        result = false;
    }
    else if (rx_buf[len-1] != check_sum) {
        std::cerr << "Invalid check sum!" << std::endl;
        result = false;
    }

    if (result == false) {
        std::cout << "RX: ";

        for (std::size_t i=0; i< static_cast<std::size_t>(len); i++) {
            std::cout << " " << std::hex << (int)rx_buf[i];
        }

        std::cout << std::endl;
    }
    else {
        uint16_t temp_min, temp_max;
        temp_min = (((int32_t)rx_buf[8] << 8) & 0xff00) + 
                   (((int32_t)rx_buf[9]) & 0x00ff);

        temp_max = (((int32_t)rx_buf[10] << 8) & 0xff00) + 
                   (((int32_t)rx_buf[11]) & 0x00ff);

        range_min = (float)temp_min/1000;
        range_max = (float)temp_max/1000;

        std::cout << "Get scan range success, range min: " << range_min 
                    << ", range max: " << range_max << std::endl;
    }

    return result;
}

bool FREEEthDriver::getScanData(uint8_t state)
{
    const size_t cmd_length = 10;
    uint8_t tx_buf[50] = {0,};
    uint8_t check_sum = 0;

    for (std::size_t i=0; i<4; i++) {
        tx_buf[i] = 0x02;
    }
    
    tx_buf[4] = (uint8_t)((cmd_length & 0xff00) >> 8);
    tx_buf[5] = (uint8_t)(cmd_length & 0x00ff);
    tx_buf[6] = METHOD_DOWN;
    tx_buf[7] = GET_DATA;
    tx_buf[8] = state;

    for (std::size_t i=0; i<cmd_length-1; i++) {
        check_sum += tx_buf[i];
    }

    tx_buf[cmd_length-1] = check_sum;

    fd_set read_set;
    // struct timeval time_out;//未使用

    FD_ZERO(&read_set);
    FD_SET(socket_fd_, &read_set);

    // time_out.tv_sec = 1;
    // time_out.tv_usec = 0;

    if (write(socket_fd_, tx_buf, cmd_length) != cmd_length) {
        std::cerr << "Socket write failed!" << std::endl;
        return false;
    }

    if (state == 0x00) {
        std::cout << "Stop scan output!" << std::endl;
        is_capturing_ = false;
    }
    else if (state == 0x01) {
        std::cout << "Start scan output!" << std::endl;
        is_capturing_ = true;
    }
    return true;
}

static uint8_t calc_checksum(const uint8_t* p, uint16_t len) {
    if (len == 0) return 0;
    uint8_t sum = 0;
    for (uint16_t i = 0; i < len - 1; ++i) {
       // printf("i=%03hu  val=0x%02X  sum=0x%02X\n", i, p[i], sum);
        sum += p[i];
    }  
    return sum;
}
void FREEEthDriver::socketThreadFunc()
{
    while (!stop_socket_thread_) {
        fd_set read_set;
        FD_ZERO(&read_set);
        FD_SET(socket_fd_, &read_set);

        struct timeval tv;
        tv.tv_sec  = 0;
        tv.tv_usec = 2000;   // 2 ms 超时，比 1.4 ms 略大即可

        int ret = select(socket_fd_ + 1, &read_set, nullptr, nullptr, &tv);
        if (ret <= 0) continue;          // 超时或无数据

        uint8_t rx_buf[65535];
        ssize_t len = ::read(socket_fd_, rx_buf, sizeof(rx_buf));
        if (len > 0) {
            std::lock_guard<std::mutex> lock(data_mutex_);
           // uint16_t get_packet_size=len;
           //  writelog(get_packet_size,0);
            data.insert(data.end(), rx_buf, rx_buf + len);
            if (data.size() >= 172950) {
                data.clear();
                std::cerr << "Too many data in receiver queue, dropping data!" << std::endl;
            }
        }else if (len == 0) {
                // 对端关闭连接
                std::cerr << "Connection closed by peer" << std::endl;
                is_connected_ = false;
                break;
            } else if (len < 0) {
                // 读取错误
                if (errno == ECONNRESET) {
                    std::cerr << "Connection reset by peer" << std::endl;
                } else {
                    std::cerr << "Read error: " << strerror(errno) << std::endl;
                }
                is_connected_ = false;
                break;
            }


    }
}



bool FREEEthDriver::scanDataReceiver()
{

    std::lock_guard<std::mutex> lock(data_mutex_);

    /* 若数据不足，直接返回，等待下次调用 */
    if (data.size() < 22) return true;
     while (data.size() >= 22) {
        /* 2.1 包头检查 */
        if (!(data[0] == 0x02 && data[1] == 0x02 &&
              data[2] == 0x02 && data[3] == 0x02 &&
              data[6] == METHOD_UP && data[7] == SCAN_DATA)) {
            data.pop_front(); continue;
        }
       // std::cerr << "find packetheader,process data: "<< '\n';
        /* 2.2 解析包头字段 */
        uint16_t packet_size = (((uint16_t)data[4] << 8) & 0xff00) + ((uint16_t)data[5] & 0x00ff);
        uint16_t scan_points = (((uint16_t)data[16] << 8) & 0xff00) + ((uint16_t)data[17] & 0x00ff);
        const uint16_t points_index = (((uint16_t)data[18] << 8) & 0xff00) + ((uint16_t)data[19] & 0x00ff);
        const uint16_t cloud_num    = (((uint16_t)data[9] << 8) & 0xff00) + ((uint16_t)data[10] & 0x00ff);
        const uint8_t  packet_idx   = data[11];
        const uint16_t pkt_points   =  (((uint16_t)data[20] << 8) & 0xff00) + ((uint16_t)data[21] & 0x00ff);
        const uint16_t expect_size  = 33 + pkt_points * 4;
        if (packet_size != expect_size) {
         printf("packet_size=0x%04X (%u)  expect_size=0x%04X (%u)  pkt_points=%u\n",
           packet_size, packet_size,
           expect_size, expect_size,
           pkt_points);
         data.pop_front();
          continue; }
	//std::cerr << " cloud_num: "<<+cloud_num<<" packet_idx "<<+packet_idx<<" packet_size: "<<+packet_size<< '\n';
        /* 2.3 数据长度 & 校验和 */
        if (data.size() < packet_size)
        {
        //std::cerr << "data.size(): "<<+data.size()<<" packet_size: "<<+packet_size<< '\n';
         break;}
         // 1. 取出校验字节（提前保存，避免 erase 后失效）
        const uint8_t expect_chks = data[packet_size - 1];
        // 2. 拷贝整包（包含 0~packet_size-2）
	std::vector<uint8_t> packet_copy(data.begin(), data.begin() + packet_size);

	// 3. 用拷贝后的数据做校验
	const uint8_t actual_chks = calc_checksum(packet_copy.data(), packet_copy.size());
	
        if (expect_chks != actual_chks) {
           std::cerr << "Checksum error! expect 0x"
          << std::hex << std::setw(2) << std::setfill('0') << +expect_chks
          << " actual 0x"
          << std::hex << std::setw(2) << std::setfill('0') << +actual_chks
          << " packet_size 0x" << std::hex << std::setw(4) << std::setfill('0') << packet_size
          << " (dec " << std::dec << packet_size << ")"
          << " cloud " << cloud_num <<" packet_idx"<<+packet_idx<< '\n';
            data.erase(data.begin(), data.begin() + packet_size);
            current_ok_ = false;        // 整圈失效
            scan_data_.ranges_data.clear();
            scan_data_.intensities_data.clear();
            continue;
        }

        /* 2.4 圈/包连续性 */
        //const uint8_t max_pkt = (scan_points + pkt_points - 1) / pkt_points - 1;
        bool ok = true;
        if (cloud_num != last_cloud_) {                 // 新圈
            last_cloud_ = cloud_num;
            last_pkt_   = 0xff;                         // 期望下一包为 0
            current_ok_ = true;                         // 新圈开始
        } else {
            ok = (packet_idx == last_pkt_ + 1);         // 同一圈应连续
        }

        if (!ok) {
            std::cerr << "Sequence error! drop cloud " << cloud_num
                      << " expect=" << (last_pkt_ + 1)
                      << " got=" << (int)packet_idx << '\n';
            bad_scan_cnt=cloud_num;
            data.erase(data.begin(), data.begin() + packet_size);
            current_ok_ = false;                        // 整圈失效
            scan_data_.ranges_data.clear();
            scan_data_.intensities_data.clear();
            continue;
        }

        last_pkt_ = packet_idx;

         /* 2.5 收集数据 */
         if (data[11] == 0x00) {
                scan_data_ = ScanData(); 
                scan_data_.frame_seq=cloud_num;     
                
                struct timeval tv;
                gettimeofday(&tv, nullptr);
                scan_data_.recv_first_sec = tv.tv_sec;
                scan_data_.recv_first_nsec= tv.tv_usec*1000;
                
          
                //以第一个包的NTP时间戳作为topic发送时间戳
               
            scan_data_.sec = ((uint32_t)data[packet_size - 9] << 24) + 
                                                  ((uint32_t)data[packet_size - 8] << 16) + 
                                                  ((uint32_t)data[packet_size - 7] << 8) + 
                                                  (uint32_t)data[packet_size - 6];
		    if(scan_data_.sec > 2208988800)
                    {
		      scan_data_.sec-= 2208988800;
		    }
                    scan_data_.nsec= ((uint32_t)data[packet_size - 5] << 24) + 
                                         ((uint32_t)data[packet_size - 4] << 16) + 
                                         ((uint32_t)data[packet_size - 3] << 8) + 
                                         (uint32_t)data[packet_size - 2];
		    //std::cout<<std::hex<<laser_scan.header.stamp.sec<<"  "<<temp_nsec<<std::endl;
                    scan_data_.nsec = (uint32_t)round((double)scan_data_.nsec/ pow(2, 32) * pow(10, 9));
  
            }

           // writelog(cloud_num,packet_idx);
            if(current_ok_){
                //std::cerr << "check finished,ready to get packet data"<< '\n';
                for (std::size_t i=0; i<pkt_points; i++) {
                uint16_t range = (((uint16_t)data[22+i*4] << 8) & 0xff00) + ((uint16_t)data[23+i*4] & 0x00ff);
                uint16_t intensity = (((uint16_t)data[24+i*4] << 8) & 0xff00) + ((uint16_t)data[25+i*4] & 0x00ff);

                scan_data_.ranges_data.push_back(range);
                scan_data_.intensities_data.push_back(intensity);
            }
            }             

             /* 2.6 最后一包 → 整圈完成 */
        if ((points_index + pkt_points )== scan_points && current_ok_) {
            //std::cerr << "[OK]get points: "<<(points_index + pkt_points )<<" expect points: "<<scan_points<< '\n';
                    struct timeval tv;
                    gettimeofday(&tv, nullptr);
                    scan_data_.recv_last_sec = tv.tv_sec;
                    scan_data_.recv_last_nsec= tv.tv_usec*1000;

            scan_data_buf_.push_back(scan_data_);
            scan_data_ = ScanData();  // 清空，准备下一圈
                    

                    is_full_ = true;
        }else{
            //std::cerr << "get points: "<<(points_index + pkt_points )<<" expect points: "<<scan_points<< '\n';
        }            
            data.erase(data.begin(), data.begin()+packet_size);
            byte_count_ = 0;
        }
       
    

    return true;
}

void FREEEthDriver::initlog()
{

namespace fs = std::filesystem;
    
//    std::string home_dir = std::getenv("HOME");
//    std::string dir =home_dir+"/userdata/";   
    std::string dir ="/userdata/";  
    fs::create_directories(dir);

    std::string path = dir + "recv_timer_data.txt";

    // 1. 先以截断模式打开，清空旧内容
    {
        std::ofstream tmp(path, std::ios::out | std::ios::trunc);
        if (!tmp.is_open())
            throw std::runtime_error("Cannot open " + path);
    }

    // 2. 再打开为追加模式，供后续 write_timer_stamp 使用
    raw_log_.open(path, std::ios::app);
    if (!raw_log_.is_open())
        throw std::runtime_error("Cannot open " + path);

    // 设置固定 9 位小数
    raw_log_<< std::fixed << std::setprecision(9);
}

void FREEEthDriver::writelog(uint16_t cloud_num)
{
          /* ---------- 计时 & 日志 ---------- */

 struct timeval tv_packet;
 gettimeofday(&tv_packet, nullptr);         // 节点内可用 this->now()
double now_sec = tv_packet.tv_sec + tv_packet.tv_usec / 1e6;
double diff      = 0.0;

if (last_packet_time_.tv_sec != 0 || last_packet_time_.tv_usec != 0) {
    double last_sec = last_packet_time_.tv_sec + last_packet_time_.tv_usec / 1e6;
    diff = now_sec - last_sec;
}

raw_log_ <<" packet_size: " <<cloud_num
        << " packet_time: " << now_sec
         << " inter_packet_diff: " << diff
         << '\n';
raw_log_.flush();

last_packet_time_ = tv_packet;
}


}