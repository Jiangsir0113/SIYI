import serial
import struct
import time

# ---------------------------- CRC16 校验表 ----------------------------
crc16_tab = [
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
]


# ---------------------------- 核心功能函数 ----------------------------
def crc16_cal(data):
    """计算CRC16校验值"""
    crc = 0
    for byte in data:
        temp = (crc >> 8) & 0xFF
        crc = ((crc << 8) & 0xFFFF) ^ crc16_tab[(byte ^ temp) & 0xFF]
    return crc


def send_gimbal_control(ser, yaw, pitch):
    """发送云台控制指令"""
    # 构建基础命令
    cmd = bytearray([
        0x55, 0x66,  # STX
        0x01,  # CTRL (需要ACK)
        0x04, 0x00,  # Data_len
        0x00, 0x00,  # SEQ
        0x0E  # CMD_ID
    ])

    # 添加角度数据（小端序）
    cmd.extend(struct.pack('<h', int(yaw * 10)))  # 单位0.1度
    cmd.extend(struct.pack('<h', int(pitch * 10)))  # 示例：-90° -> 0xFC7C

    # 计算并附加CRC
    crc = crc16_cal(cmd)
    cmd.extend(struct.pack('<H', crc))

    # 发送指令
    print(f"[控制指令] 发送: {cmd.hex()}")
    ser.write(cmd)

    # 读取ACK（可选）
    ack = ser.read(10)
    if ack:
        print(f"[ACK响应] 接收: {ack.hex()}")
    else:
        print("[警告] 未收到ACK")


def request_attitude(ser):
    """请求并解析姿态信息（修复版）"""
    # 清空缓冲区残留数据
    ser.reset_input_buffer()

    # 构建请求指令
    req_cmd = bytearray([
        0x55, 0x66,  # STX
        0x01,  # CTRL
        0x00, 0x00,  # Data_len
        0x00, 0x00,  # SEQ
        0x0D  # CMD_ID
    ])

    # 计算CRC并附加
    crc = crc16_cal(req_cmd)
    req_cmd.extend(struct.pack('<H', crc))

    # 发送请求
    print(f"[姿态请求] 发送: {req_cmd.hex()}")
    ser.write(req_cmd)

    # 动态读取完整数据包
    packet = bytearray()
    start_time = time.time()
    while time.time() - start_time < ser.timeout:
        # 逐字节读取寻找STX
        b = ser.read(1)
        if not b:
            continue
        packet += b

        # 检查是否找到STX
        if len(packet) >= 2:
            if packet[0] == 0x55 and packet[1] == 0x66:
                # 读取剩余字节
                remaining = 22 - len(packet)
                packet += ser.read(remaining)
                break

    # 校验数据完整性
    if len(packet) != 22:
        raise ValueError(f"数据包长度错误: 预期22字节，实际{len(packet)}字节")

    print(f"[姿态数据] 原始: {packet.hex()}")

    # 解析数据
    try:
        stx = struct.unpack('<H', packet[0:2])[0]
        if stx != 0x6655:
            raise ValueError(f"STX校验失败: 收到{stx:#06x}，期望0x6655")

        data_len = struct.unpack('<H', packet[3:5])[0]
        if data_len != 12:
            raise ValueError(f"数据长度错误: 收到{data_len}，期望12")

        # 解析角度（小端序）
        yaw, pitch, roll, *_ = struct.unpack('<6h', packet[8:20])
        yaw = round(yaw / 10.0, 1)
        if pitch <= -90:
            pitch = -round(pitch / 10.0, 1) # 符号处理
        else:
            pitch = round(pitch / 10.0 - 180, 1)
        roll = round(roll / 10.0, 1)

        print("\n------ 当前姿态 ------")
        print(f"偏航角(Yaw): {yaw}°")
        print(f"俯仰角(Pitch): {pitch}°")
        print(f"横滚角(Roll): {roll}°\n")

    except Exception as e:
        print(f"[解析错误] {str(e)}")
        print("原始数据包:", packet.hex())


# ---------------------------- 主程序 ----------------------------
if __name__ == '__main__':
    # 初始化串口
    ser = serial.Serial('COM15', 115200, timeout=2)
    print(f"已连接串口: {ser.name}")

    try:
        # 示例控制
        print("\n=== 测试控制：Yaw=0°, Pitch=-90° ===")
        send_gimbal_control(ser, 0.0, -2)
        time.sleep(2.5)  # 等待云台运动

        print("\n=== 请求当前姿态 ===")
        request_attitude(ser)

    except Exception as e:
        print(f"操作失败: {str(e)}")
    finally:
        ser.close()
        print("串口已关闭")
