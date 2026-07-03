import serial
import time
import msvcrt

PORT = 'COM3'
BAUDRATE = 9600

ser = serial.Serial(PORT, BAUDRATE, timeout=1)
time.sleep(2)

ON_CMD     = bytes([0xA0, 0x01, 0x01, 0xA2])
OFF_CMD    = bytes([0xA0, 0x01, 0x00, 0xA1])
STATUS_CMD = bytes([0xFF])
toggle = False

def relay_on():
    ser.write(ON_CMD)

def relay_off():
    ser.write(OFF_CMD)

def relay_status():
    ser.write(STATUS_CMD)
    time.sleep(0.2)
    response = ser.read(ser.in_waiting or 1)
    text = response.decode('utf-8', errors='ignore').strip()
    print(text)
    return response

try:
    while True:
        if msvcrt.kbhit():
            key = msvcrt.getch()

            if key == b'\x1b':          # ESC → 종료
                print("종료합니다.")
                break

            elif key.lower() == b's':   # 's' 또는 'S' → 상태 확인
                relay_status()

            else:                        # 그 외 모든 키 → 토글
                toggle = not toggle
                if toggle:
                    relay_on()
                    print(">> Relay ON")
                else:
                    relay_off()
                    print(">> Relay OFF")

        time.sleep(0.05)
finally:
    ser.close()