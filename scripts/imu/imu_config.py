#!/usr/bin/env python3
"""
DM-IMU-L1 配置脚本
用于通过 USB 串口配置 IMU 的输出设置
"""
import serial
import time
import sys


def send_command(ser, cmd_hex, description):
    """发送配置命令到 IMU"""
    cmd_bytes = bytes.fromhex(cmd_hex.replace(" ", ""))
    print(f"发送: {description}")
    print(f"  命令: {cmd_hex}")
    ser.write(cmd_bytes)
    time.sleep(0.1)  # 等待 IMU 处理


def main():
    port = "/dev/ttyACM0"
    baudrate = 921600

    if len(sys.argv) > 1:
        port = sys.argv[1]

    print(f"正在连接 IMU: {port} @ {baudrate}")

    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(0.5)  # 等待串口稳定

        print("\n=== 开始配置 IMU ===\n")

        # 1. 进入设置模式
        send_command(ser, "AA 06 01 0D", "进入设置模式")

        # 2. 开启加速度输出
        send_command(ser, "AA 01 14 0D", "开启加速度输出")

        # 3. 开启角速度输出
        send_command(ser, "AA 01 15 0D", "开启角速度输出")

        # 4. 开启欧拉角输出
        send_command(ser, "AA 01 16 0D", "开启欧拉角输出")

        # 5. 关闭四元数输出（可选，如果不需要的话）
        # send_command(ser, 'AA 01 07 0D', '关闭四元数输出')

        # 6. 保存参数（重要！否则掉电后会丢失设置）
        send_command(ser, "AA 03 01 0D", "保存参数到 Flash")

        # 7. 退出设置模式，进入正常模式
        send_command(ser, "AA 06 00 0D", "进入正常模式")

        print("\n=== IMU 配置完成 ===")

        ser.close()

    except serial.SerialException as e:
        print(f"错误: 无法打开串口 {port}")
        print(f"详细信息: {e}")
        print("\n请检查:")
        print("  1. IMU 是否已连接")
        print("  2. 串口设备名称是否正确（ls /dev/ttyACM*）")
        print("  3. 是否有权限访问串口（sudo chmod 666 /dev/ttyACM0）")
        sys.exit(1)
    except Exception as e:
        print(f"未知错误: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
