#include <free_lidar/c2_uart_driver.h>
#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <errno.h>
#include <cmath>
#include "rclcpp/time.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sys/time.h>


namespace free_optics {

C2UartDriver::C2UartDriver()
{
   std::cout << "====== " << "FREE ETH version: " << C2_ETH_ROS_VER << " ======" << std::endl;
    is_connected_ = false;
    is_capturing_ = false;

    scan_data_buf_.clear();
    scan_data_.ranges_data.clear();
    scan_data_.intensities_data.clear();
    data.clear();

    byte_count_ = 0;
};

C2UartDriver::~C2UartDriver()
{
    disconnect();
}

bool C2UartDriver::connect(const std::string portname,int32_t baud)
{
    std::cerr << "connect to "<<portname<<" baud:"<<baud<< std::endl;
    try {
        sp_.setPort(portname);
        sp_.setBaudrate(baud);  // 设置默认波特率
        sp_.setBytesize(serial::bytesize_t::eightbits);
        sp_.setStopbits(serial::stopbits_t::stopbits_one);
        sp_.setParity(serial::parity_t::parity_none);
        sp_.setFlowcontrol(serial::flowcontrol_t::flowcontrol_none);
        // sp_.setBytesize(serial::bytesize_t::eightbits);
        // sp_.setStopbits(serial::stopbits_t::stopbits_one);
        // sp_.setParity(serial::parity_t::parity_none);
        // sp_.setFlowcontrol(serial::flowcontrol_t::flowcontrol_none);

        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        to.inter_byte_timeout = 0;
        sp_.setTimeout(to);
        
        sp_.open();
    }
    catch (const std::exception& e) {
        std::cerr << "e.what(): "<< std::endl;
        std::cerr << e.what() << std::endl;
        is_connected_ = false;
        return false;
    }

    is_connected_ = sp_.isOpen();
    return is_connected_;
}

void C2UartDriver::disconnect()
{
    if (is_capturing_ == true) {
        getScanData(0);
    }

    if (is_connected_ == true) {
        sp_.close();
        std::cout << "Scanner serial port closed!" << std::endl;
    }

    is_connected_ = false;
    is_capturing_ = false;
}

void C2UartDriver::ClearBuf()
{
     uint8_t rx_buf[65535] = {0,};
     //clean serial port given buffer
    try {
        sp_.read(rx_buf, 65535);
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        //return false;
    }
}


bool C2UartDriver::getBaudrate()
{
    uint8_t log_state = 0;
    bool result = false;
    uint8_t rx_buf[65535] = {0,};

    std::cout << "Scanner serial port initialization..." << std::endl;
    if (sp_.available() > 0) {
        for (std::size_t i=0; i<sizeof(baudrate)/4; i++) {
            try {
                sp_.setBaudrate(baudrate[i]);
            }
            catch (const std::exception& e) {
                std::cerr << e.what() << std::endl;
                return false;
            }
            
            if (getScanData(0) == false) {
                return false;
            }
        }
    }

    //clean serial port given buffer
    try {
        sp_.read(rx_buf, 65535);
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return false;
    }

    for (std::size_t i=0; i<sizeof(baudrate)/sizeof(uint32_t); i++) {
        try {
            sp_.setBaudrate(baudrate[i]);
        }
        catch (const std::exception& e) {
            std::cerr << e.what() << std::endl;
            return false;
        }

        if (login(log_state) == true) {
            std::cout << "Baudrate is: " << std::dec << baudrate[i]  << " bps" << std::endl;
            result = true;
            break;
        }
    }

    if (result == false) {
        std::cerr << "No available baudrate!" << std::endl;
    }
    else if (result == true && log_state == 0) {
        result = false;
    }
    
    return result;
}

bool C2UartDriver::getDeviceState(uint8_t &state)
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

    try {
        if (sp_.write(tx_buf, cmd_length) != cmd_length) {
            std::cerr << "Serial port write failed(getDeviceState)!" << std::endl;
            return false;
        }
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return false;
    }
    
    try {
        uint8_t rx_buf[50] = {0,};
        std::size_t len = sp_.read(rx_buf, 50);
        if (len <= 0) {
            std::cerr << "Serial port read timeout(getDeviceState)!" << std::endl;
            return false;
        }

        check_sum = 0;
        for (std::size_t i=0; i<len-1; i++) {
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

            for (std::size_t i=0; i<len; i++) {
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
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return false;
    }
}

bool C2UartDriver::getDeviceName(std::string &name)
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

    try {
        if (sp_.write(tx_buf, cmd_length) != cmd_length) {
            std::cerr << "Serial port write failed(getDeviceName)!" << std::endl;
            return false;
        }
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return false;
    }

    try {
        uint8_t rx_buf[50] = {0,};
        size_t len = sp_.read(rx_buf, 50);

        if (len <= 0) {
            std::cerr << "Serial port read timeout(getDeviceName)!" << std::endl;
            return false;
        }

        check_sum = 0;
        for (std::size_t i=0; i<len-1; i++) {
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

            for (std::size_t i=0; i<len; i++) {
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
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return false;
    }
}

bool C2UartDriver::getFirmwareVersion(uint32_t &version)
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

    try {
        if (sp_.write(tx_buf, cmd_length) != cmd_length) {
            std::cerr << "Serial port write failed(getFirmwareVersion)!" << std::endl;
            return false;
        }
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return false;
    }
    
    try {
        uint8_t rx_buf[50] = {0,};
        size_t len = sp_.read(rx_buf, 50);

        if (len <= 0) {
            std::cerr << "Serial port read timeout(getFirmwareVersion)!" << std::endl;
            return false;
        }

        check_sum = 0;
        for (std::size_t i=0; i<len-1; i++) {
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

            for (std::size_t i=0; i<len; i++) {
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
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return false;
    }
}

bool C2UartDriver::getSerialNumber(uint32_t &sn)
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

    try {
        if (sp_.write(tx_buf, cmd_length) != cmd_length) {
            std::cerr << "Serial port write failed(getSerialNumber)!" << std::endl;
            return false;
        }
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return false;
    }
    
    try {
        uint8_t rx_buf[50] = {0,};
        size_t len = sp_.read(rx_buf, 50);

        if (len <= 0) {
            std::cerr << "Serial port read timeout(getSerialNumber)!" << std::endl;
            return false;
        }

        check_sum = 0;

        for (std::size_t i=0; i<len-1; i++) {
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

            for (std::size_t i=0; i<len; i++) {
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
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return false;
    }
}

bool C2UartDriver::login(uint8_t &state)
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

    try {
        if (sp_.write(tx_buf, cmd_length) != cmd_length) {
            std::cerr << "Serial port write failed(login)!" << std::endl;
            return false;
        }
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return false;
    }
    sleep(2);
    try {
        uint8_t rx_buf[50] = {0,};
        size_t len = sp_.read(rx_buf, 50);

        if (len <= 0) {
            std::cerr << "Serial port read timeout(login)!" << std::endl;
            return false;
        }

        check_sum = 0;
        for (std::size_t i=0; i<len-1; i++) {
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

            for (std::size_t i=0; i<len; i++) {
                std::cout << " " << std::hex << (int)rx_buf[i];
            }

            std::cout << std::endl;
        }
        else {
            std::cout << "Log in success." << std::endl;
        }

        return result;
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return false;
    }
}

bool C2UartDriver::getScanAngle(int32_t &start_angle, int32_t &stop_angle)
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

    try {
        if (sp_.write(tx_buf, cmd_length) != cmd_length) {
            std::cerr << "Serial port write failed!" << std::endl;
            return false;
        }
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return false;
    }

    try {
        uint8_t rx_buf[50] = {0,};
        size_t len = sp_.read(rx_buf, 50);

        if (len <= 0) {
            std::cerr << "Serial port read timeout!" << std::endl;
            return false;
        }

        check_sum = 0;
        for (std::size_t i=0; i<len-1; i++) {
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

            for (std::size_t i=0; i<len; i++) {
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
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return false;
    }
}

bool C2UartDriver::setReflectivityNormalization(uint8_t normalizationFactor)
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

     for (std::size_t i=0; i<cmd_length-1; i++) {
        check_sum += tx_buf[i];
    }

    tx_buf[cmd_length-1] = check_sum;

    try {
        if (sp_.write(tx_buf, cmd_length) != cmd_length) {
            std::cerr << "Serial port write failed!" << std::endl;
            return false;
        }
    }
    catch(const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return false;
    }

    try {
        uint8_t rx_buf[50] = {0,};
        size_t len = sp_.read(rx_buf, 50);

        if (len <= 0) {
            std::cerr << "Serial port read timeout!" << std::endl;
            return false;
        }

        check_sum = 0;

        for (std::size_t i=0; i<len-1; i++) {
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
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return false;
    }
}

bool C2UartDriver::setScanAngle(int32_t start_angle, int32_t stop_angle)
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

    try {
        if (sp_.write(tx_buf, cmd_length) != cmd_length) {
            std::cerr << "Serial port write failed(setScanAngle)!" << std::endl;
            return false;
        }
    }
    catch(const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return false;
    }

    try {
        uint8_t rx_buf[50] = {0,};
        size_t len = sp_.read(rx_buf, 50);

        if (len <= 0) {
            std::cerr << "Serial port read timeout(setScanAngle)!" << std::endl;
            return false;
        }

        check_sum = 0;

        for (std::size_t i=0; i<len-1; i++) {
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

            for (std::size_t i=0; i<len; i++) {
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
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return false;
    }
}

bool C2UartDriver::setScanConfig(uint16_t frequency, uint16_t resolution)
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

    try {
        if (sp_.write(tx_buf, cmd_length) != cmd_length) {
            std::cerr << "Serial port write failed(setScanConfig)!" << std::endl;
            return false;
        }
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return false;
    }
    
    try {
        uint8_t rx_buf[50] = {0,};
        size_t len = sp_.read(rx_buf, 50);

        if (len <= 0) {
            std::cerr << "Serial port read timeout(setScanConfig)!" << std::endl;
            return false;
        }

        check_sum = 0;

        for (std::size_t i=0; i<len-1; i++) {
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

            for (std::size_t i=0; i<len; i++) {
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
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return false;
    }
}

bool C2UartDriver::getMeasureRange(float &range_min, float &range_max)
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

    try {
        if (sp_.write(tx_buf, cmd_length) != cmd_length) {
            std::cerr << "Serial port write failed!" << std::endl;
            return false;
        }
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return false;
    }

    try {
        uint8_t rx_buf[50] = {0,};
        size_t len = sp_.read(rx_buf, 50);

        if (len <= 0) {
            std::cerr << "Serial port read timeout!" << std::endl;
            return false;
        }

        check_sum = 0;
        for (std::size_t i=0; i<len-1; i++) {
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

            for (std::size_t i=0; i<len; i++) {
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
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return false;
    }
}

bool C2UartDriver::getScanData(uint8_t state)
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

    try {
        if (sp_.write(tx_buf, cmd_length) != cmd_length) {
            std::cerr << "Serial port write failed(getScanData)!" << std::endl;
            return false;
        }
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
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

void C2UartDriver::socketThreadFunc()
{
    // 设置超时时间为 2000 毫秒
    // 创建一个 serial::Timeout 对象
    serial::Timeout timeout = serial::Timeout::simpleTimeout(2000);

    // 设置超时时间
    sp_.setTimeout(timeout);

    while (!stop_socket_thread_) {
        uint8_t rx_buf[2048];
        size_t len = sp_.read(rx_buf, sizeof(rx_buf));
        if (len <= 0) {
            std::cerr << "Serial port read timeout(scandata)!" << std::endl;
            continue;
        }

        std::lock_guard<std::mutex> lock(data_mutex_);
        data.insert(data.end(), rx_buf, rx_buf + len);

        if (data.size() >= 60000) {
            data.clear();
            std::cerr << "Too many data in receiver queue, dropping data!" << std::endl;
        }
    }
}
bool C2UartDriver::scanDataReceiver()
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
	//std::cerr << "cloud_num: "<<+cloud_num<<" packet_idx "<<+packet_idx<<"packet_size:"<<+packet_size<< '\n';
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
                for (std::size_t i=0; i<pkt_points; i++) {
                uint16_t range = (((uint16_t)data[22+i*4] << 8) & 0xff00) + ((uint16_t)data[23+i*4] & 0x00ff);
                uint16_t intensity = (((uint16_t)data[24+i*4] << 8) & 0xff00) + ((uint16_t)data[25+i*4] & 0x00ff);

                scan_data_.ranges_data.push_back(range);
                scan_data_.intensities_data.push_back(intensity);
            }
            }             

             /* 2.6 最后一包 → 整圈完成 */
        if ((points_index + pkt_points )== scan_points && current_ok_) {
                    struct timeval tv;
                    gettimeofday(&tv, nullptr);
                    scan_data_.recv_last_sec = tv.tv_sec;
                    scan_data_.recv_last_nsec= tv.tv_usec*1000;

             scan_data_buf_.push_back(scan_data_);
            scan_data_ = ScanData();  // 清空，准备下一圈
                    

                    is_full_ = true;
        }            
            data.erase(data.begin(), data.begin()+packet_size);
            byte_count_ = 0;
        }       
    
    return true;
}

}