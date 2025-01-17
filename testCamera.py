import serial
import struct


def parse_gimbal_data(byte_data):
    """
    解析云台相机姿态数据
    """
    # 校验数据长度
    if len(byte_data) != 22:
        raise ValueError(f"数据长度不正确，预期22字节，实际{len(byte_data)}字节。")

    # 解析STX
    stx = struct.unpack('<H', byte_data[0:2])[0]
    if stx != 0x6655:
        raise ValueError(f"无效的STX: {stx:#06x}")

    # 解析CTRL字段
    ctrl = byte_data[2]

    # 解析Data_len
    data_len = struct.unpack('<H', byte_data[3:5])[0]
    if data_len != 12:
        raise ValueError(f"无效的Data_len: {data_len}")

    # 解析SEQ
    seq = struct.unpack('<H', byte_data[5:7])[0]

    # 解析Command ID
    command_id = byte_data[7]

    # 解析Data
    payload = byte_data[8:20]
    yaw, pitch, roll, yaw_vel, pitch_vel, roll_vel = struct.unpack('<hhhhhh', payload)

    # 转换为实际角度和速率
    yaw = round(yaw / 10.0, 1)
    pitch = round(pitch / 10.0, 1)
    roll = round(roll / 10.0, 1)

    # 打印解析结果
    print(f"STX: 0x{stx:04X}")
    print(f"CTRL: {ctrl}")
    print(f"Data Length: {data_len}")
    print(f"SEQ: {seq}")
    print(f"Command ID: 0x{command_id:02X}")
    print(f"Yaw (偏航角): {yaw}°")
    print(f"Pitch (俯仰角): {pitch}°")
    print(f"Roll (滚转角): {roll}°")
    print(f"Yaw Velocity (偏航角速率): {yaw_vel}°/s")
    print(f"Pitch Velocity (俯仰角速率): {pitch_vel}°/s")
    print(f"Roll Velocity (滚转角速率): {roll_vel}°/s")


if __name__ == '__main__':
    ser = serial.Serial('COM15', 115200, timeout=1)

    # request = b'\x55\x66\x01\x02\x00\x00\x00\x07\x64\x64\x3d\xcf'  # 云台转向 100 100
    # request1 = b'\x55\x66\x01\x01\x00\x00\x00\x08\x01\xd1\x12'  # 一键回中
    # request2 = b'\x55\x66\x01\x04\x00\x00\x00\x0e\x00\x00\xff\xa6\x3b\x11'
    request3 = b'\x55\x66\x01\x00\x00\x00\x00\x0d\xe8\x05'
    # ser.write(request1)
    # ser.write(request2)
    ser.write(request3)
    response = ser.read(22)
    print(response)
    # 解析数据
    parse_gimbal_data(response)
