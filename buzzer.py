import serial, time

PORT = "/dev/ttyACM0"
BAUD = 19200
DEVICE_ID = 1

PID_COMMAND = 10
CMD_BUZZER_ON = 80
CMD_BUZZER_OFF = 81
PID_READ_TARGET = 17

MID_PC = 172
MID_MOTOR = 183

ser = serial.Serial(PORT, BAUD, timeout=1.0)

def checksum(data: bytes) -> int:
    return (-sum(data)) & 0xFF

def make_frame(data: bytes) -> bytes:
    return data + bytes([checksum(data)])

def get_info():
    frame = bytes([
        MID_MOTOR,
        MID_PC,
        DEVICE_ID,
        4,
        1,
        PID_READ_TARGET
    ])
    ser.write(make_frame(frame))
    data = ser.read(50)
    for i, b in enumerate(data):
        print(f'[{i:02}] 0x{b:02X} ({b:3d})')

def set_buzzer_on():
    frame = bytes([
        MID_MOTOR,
        MID_PC,
        DEVICE_ID,
        PID_COMMAND,
        1,
        CMD_BUZZER_ON
    ])

    ser.write(make_frame(frame))

def set_buzzer_off():
    frame = bytes([
        MID_MOTOR,
        MID_PC,
        DEVICE_ID,
        PID_COMMAND,
        1,
        CMD_BUZZER_OFF
    ])

    ser.write(make_frame(frame))

try:
    get_info()

finally:
    ser.flush()