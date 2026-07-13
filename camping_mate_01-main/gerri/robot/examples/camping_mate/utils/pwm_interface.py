import time

# sysfs 경로 (보드에 따라 pwmchip 번호가 다를 수 있음!)
PWM_LEFT_CHIP = "/sys/class/pwm/pwmchip2"
PWM_CH_LEFT = PWM_LEFT_CHIP + "/pwm0"

PWM_RIGHT_CHIP = "/sys/class/pwm/pwmchip3"
PWM_CH_RIGHT = PWM_RIGHT_CHIP + "/pwm0"

def write_sysfs(path, value):
    with open(path, 'w') as f:
        f.write(str(value))

def init_pwm(period_ns=100000):
    # export (pwm0 활성화)
    for chip in [PWM_LEFT_CHIP, PWM_RIGHT_CHIP]:
        try:
            write_sysfs(chip + "/export", 0)
        except:
            pass  # 이미 export 된 경우 예외 무시
    
    time.sleep(0.1)

    # period, duty, enable 초기화
    for pwm in [PWM_CH_LEFT, PWM_CH_RIGHT]:
        write_sysfs(pwm + "/period", period_ns)
        write_sysfs(pwm + "/duty_cycle", 0)
        write_sysfs(pwm + "/enable", 1)

def set_duty(target, duty_ratio):
    # duty_ratio: 0.0 ~ 1.0
    period = int(open(target + "/period").read().strip())
    if duty_ratio <= 1.0:
        duty = int(period * duty_ratio)
    else:
        duty = int(period * (duty_ratio / 100.0))
        
    duty = min(duty, period)
    write_sysfs(target + "/duty_cycle", duty)
    print(f"{target + 'duty_cycle'} written {duty}")
    

def stop_pwm():
    # 비활성화 및 unexport
    for pwm in [PWM_CH_LEFT, PWM_CH_RIGHT]:
        write_sysfs(pwm + "/enable", 0)
    for chip in [PWM_LEFT_CHIP, PWM_RIGHT_CHIP]:
        write_sysfs(chip + "/unexport", 0)

# 사용 예시
if __name__ == "__main__":
    init_pwm(1000000)  # 20ms (50Hz → 서보 모드 예시)
    try:
        for d in [0.2, 0.5, 0.8]:
            print(f"Set Left/Right Duty: {d:.1f}")
            set_duty(PWM_CH_LEFT, d)
            set_duty(PWM_CH_RIGHT, d)
            time.sleep(2)
    finally:
        stop_pwm()
