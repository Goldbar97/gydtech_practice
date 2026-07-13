import time
import serial

from evdev import InputDevice, ecodes


# =========================================================
# Configuration
# =========================================================
PORT = "/dev/ttyACM0"
BAUD = 19200
DEVICE_ID = 1

# 자신의 키보드 경로로 변경
KEYBOARD_DEVICE = "/dev/input/by-id/usb-Logitech_USB_Receiver-event-kbd"

# 키보드 독점 사용 여부
ENABLE_GRAB = False


ADVANCE_SPEED = 400

# 제자리 회전
TURN_SPEED = 200

# 주행 중 감속량
STEERING_SPEED = 200

# 가속도
ACCELERATION_MOVE = 33
ACCELERATION_STOP = 150


SEND_HZ = 20


MID_PC = 172
MID_MOTOR = 183
PID_OPEN_VELOCITY = 208


# =========================================================
# Motor Packet
# =========================================================
def checksum(data: bytes) -> int:

    return (-sum(data)) & 0xFF


def make_frame(left: int, right: int):

    left = max(-1023, min(1023, left))
    right = max(-1023, min(1023, right))

    data = (
        bytes([1])
        + left.to_bytes(2, "little", signed=True)
        + bytes([1])
        + right.to_bytes(2, "little", signed=True)
        + bytes([0])
    )

    frame = bytes([
        MID_MOTOR,
        MID_PC,
        DEVICE_ID,
        PID_OPEN_VELOCITY,
        len(data)
    ]) + data

    return frame + bytes([checksum(frame)])


# =========================================================
# Utility
# =========================================================

def approach(current, target, step):

    if current < target:

        current += step

        if current > target:
            current = target

    elif current > target:

        current -= step

        if current < target:
            current = target

    return current


# =========================================================
# Keyboard
# =========================================================

class Keyboard:

    def __init__(self):

        self.dev = InputDevice(KEYBOARD_DEVICE)

        if ENABLE_GRAB:
            self.dev.grab()

        self.keys = {
            ecodes.KEY_W: False,
            ecodes.KEY_A: False,
            ecodes.KEY_S: False,
            ecodes.KEY_D: False,
        }


    def update(self):

        while True:

            event = self.dev.read_one()

            if event is None:
                break

            if event.type != ecodes.EV_KEY:
                continue

            pressed = event.value != 0

            if event.code in self.keys:

                self.keys[event.code] = pressed

            elif event.code == ecodes.KEY_SPACE and pressed:

                for key in self.keys:
                    self.keys[key] = False


    def pressed(self, key):

        return self.keys[key]


    def release(self):

        if ENABLE_GRAB:
            self.dev.ungrab()


# =========================================================
# Driving
# =========================================================

def calculate_target(keys):

    forward = 0
    turn = 0


    if keys.pressed(ecodes.KEY_W):

        forward = ADVANCE_SPEED

    elif keys.pressed(ecodes.KEY_S):

        forward = -ADVANCE_SPEED


    #
    # 현재 정상 동작하는 방향 유지
    #

    if keys.pressed(ecodes.KEY_A):

        turn = -1

    elif keys.pressed(ecodes.KEY_D):

        turn = 1


    target_left = forward
    target_right = forward


    # ----------------------------
    # Moving
    # ----------------------------

    if forward != 0:

        if turn > 0:

            if forward > 0:

                target_left -= STEERING_SPEED

            else:

                target_right += STEERING_SPEED


        elif turn < 0:

            if forward > 0:

                target_right -= STEERING_SPEED

            else:

                target_left += STEERING_SPEED


    # ----------------------------
    # Pivot
    # ----------------------------

    else:

        if turn > 0:

            target_left = -TURN_SPEED
            target_right = TURN_SPEED

        elif turn < 0:

            target_left = TURN_SPEED
            target_right = -TURN_SPEED


    return target_left, target_right

# =========================================================
# Motor Controller
# =========================================================

class MotorController:

    def __init__(self, serial_port):

        self.ser = serial_port

        self.current_left = 0
        self.current_right = 0


    def update(self, target_left, target_right, stop_mode):

        if stop_mode:
            accel = ACCELERATION_STOP
        else:
            accel = ACCELERATION_MOVE


        self.current_left = approach(
            self.current_left,
            target_left,
            accel
        )


        self.current_right = approach(
            self.current_right,
            target_right,
            accel
        )


    def send(self):

        self.ser.write(
            make_frame(
                int(self.current_left),
                int(self.current_right)
            )
        )
    
    def buzzer_on(self):
        frame = bytes([
            MID_MOTOR,
            MID_PC,
            DEVICE_ID,
            10,
            1,
            80
        ])
        self.ser.write(frame + bytes([checksum(frame)]))


    def stop(self):

        self.current_left = 0
        self.current_right = 0

        self.send()


# =========================================================
# Status
# =========================================================

def print_status(keys,
                 target_left,
                 target_right,
                 motor):

    print(
        f"\r"
        f"W:{int(keys.pressed(ecodes.KEY_W))} "
        f"A:{int(keys.pressed(ecodes.KEY_A))} "
        f"S:{int(keys.pressed(ecodes.KEY_S))} "
        f"D:{int(keys.pressed(ecodes.KEY_D))} "
        f"| "
        f"Target({target_left:4d},{target_right:4d}) "
        f"Motor({int(motor.current_left):4d},{int(motor.current_right):4d})",
        end="",
        flush=True
    )


# =========================================================
# Help
# =========================================================

def print_help():

    print("-----------------------------------")
    print("W : Forward")
    print("S : Backward")
    print("A : Left")
    print("D : Right")
    print("SPACE : Stop")
    print("Ctrl+C : Quit")
    print("-----------------------------------")

# =========================================================
# Main
# =========================================================

def main():

    keyboard = Keyboard()

    send_period = 1.0 / SEND_HZ

    with serial.Serial(PORT, BAUD, timeout=0.1) as ser:

        motor = MotorController(ser)

        print_help()

        try:

            while True:

                loop_start = time.monotonic()

                # -------------------------------------
                # 키보드 이벤트 처리
                # -------------------------------------

                keyboard.update()


                # -------------------------------------
                # 목표 속도 계산
                # -------------------------------------

                target_left, target_right = calculate_target(
                    keyboard
                )


                # -------------------------------------
                # 입력이 하나도 없는가?
                # -------------------------------------

                stop_mode = not (
                    keyboard.pressed(ecodes.KEY_W)
                    or keyboard.pressed(ecodes.KEY_A)
                    or keyboard.pressed(ecodes.KEY_S)
                    or keyboard.pressed(ecodes.KEY_D)
                )


                # -------------------------------------
                # 가속도 적용
                # -------------------------------------

                motor.update(
                    target_left,
                    target_right,
                    stop_mode
                )


                # -------------------------------------
                # 모터 전송
                # -------------------------------------

                motor.send()

                # -------------------------------------
                # 상태 출력
                # -------------------------------------

                print_status(
                    keyboard,
                    target_left,
                    target_right,
                    motor
                )


                # -------------------------------------
                # 20Hz 유지
                # -------------------------------------

                elapsed = time.monotonic() - loop_start

                if elapsed < send_period:

                    time.sleep(send_period - elapsed)


        except KeyboardInterrupt:

            print("\nStopping...")


        finally:

            try:

                motor.stop()

                ser.flush()

            except Exception:
                pass

            keyboard.release()

            print("Motor stopped.")


# =========================================================

if __name__ == "__main__":

    main()