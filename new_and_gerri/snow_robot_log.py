# -*- coding: utf-8 -*-
"""snow_robot 운행 로그 / 장애 감시 모듈 (공용 라이브러리 수정 없이 snow_robot 단에서만 동작).

하는 일
  1) print() 출력(공용 라이브러리 포함)을 가로채 시각이 붙은 파일 로그로 저장
       <robot.py 폴더>/snow_robot_logs/snow_robot_YYYY-MM-DD.log        전체
       <robot.py 폴더>/snow_robot_logs/snow_robot_error_YYYY-MM-DD.log  WARNING 이상만
     날짜별 파일이다. 오늘 파일이 이미 있으면 이어서 쓰고, 없으면 새로 만든다.
     자정이 지나면 실행 중에도 다음 날 파일로 자동 전환한다.
     콘솔 출력은 그대로 유지된다. /joy 메시지 줄은 파일에서 제외하고 개수만 센다.
  2) 죽은 스레드 / 처리 안 된 예외를 전부 기록 (sys / threading / unraisable hook, SIGTERM)
       -> 카메라 캡처 스레드, 모터 TX 스레드 등이 조용히 죽는 경우를 잡는다.
  3) 네트워크 감시: 서버로 TCP 연결 시간을 측정해 끊김/복구/지연을 기록
  4) 헬스 체크: 모터 TX 스레드, 짐벌 스레드, 시리얼 포트, 카메라 프레임, 온도, 메모리
       정상 -> 이상 / 이상 -> 복구 전환 시점에만 기록하고, 10초마다 [HB] 한 줄 요약을 남긴다.

환경변수
  SNOW_LOG_DIR      로그 폴더 (기본: snow_robot_robot.py와 같은 폴더의 snow_robot_logs/)
  SNOW_LOG_KEEP_DAYS 로그 보관 일수 (기본 30, 0이면 삭제 안 함)
  SNOW_LOG_DROP     파일 로그에서 제외할 줄 정규식 (기본: axes 와 buttons 가 같이 있는 줄)
  SNOW_NET_TARGET   네트워크 감시 대상 host:port (기본 175.126.123.199:443)
  SNOW_HB_PERIOD    [HB] 요약 주기 초 (기본 10)
"""

from __future__ import annotations

import atexit
import logging
import logging.handlers
import os
import re
import signal
import socket
import sys
import threading
import time
from typing import Callable, Dict, Optional, Tuple

log = logging.getLogger("snow")

_stats_lock = threading.Lock()
_stats = {"joy_total": 0, "joy_last": 0.0, "dropped": 0}
_rate_lock = threading.Lock()
_rate_last: Dict[str, float] = {}
_setup_done = False


# ----------------------------------------------------------------------
# 공용 유틸
# ----------------------------------------------------------------------
def mark_joy():
    """/joy 메시지가 들어올 때마다 호출 (SnowRobotBaseController.receive_message)."""
    with _stats_lock:
        _stats["joy_total"] += 1
        _stats["joy_last"] = time.time()


def log_every(key: str, interval: float, level: int, msg: str, **kwargs) -> bool:
    """같은 key의 로그를 interval초에 한 번만 남긴다 (반복 에러로 로그가 넘치는 것 방지)."""
    now = time.time()
    with _rate_lock:
        if now - _rate_last.get(key, 0.0) < interval:
            return False
        _rate_last[key] = now
    log.log(level, msg, **kwargs)
    return True


class DailyFileHandler(logging.Handler):
    """날짜별 로그 파일: <prefix>_YYYY-MM-DD.log (로컬 시간 기준).

    오늘 파일이 있으면 이어 쓰고(append), 없으면 새로 만든다. 날짜가 바뀌면 자동으로 새 파일로 전환한다.
    전원이 갑자기 꺼져도 남도록 기록마다 flush한다. keep_days보다 오래된 파일은 전환 때 삭제한다.
    """

    _NAME_RE = re.compile(r"_(\d{4}-\d{2}-\d{2})\.log$")

    def __init__(self, directory: str, prefix: str, keep_days: int = 30, level=logging.NOTSET):
        super().__init__(level)
        self.directory = directory
        self.prefix = prefix
        self.keep_days = keep_days
        self._day = None
        self._fh = None

    def _path(self, day: str) -> str:
        return os.path.join(self.directory, f"{self.prefix}_{day}.log")

    def _cleanup(self):
        if self.keep_days <= 0:
            return
        limit = time.time() - self.keep_days * 86400
        try:
            for name in os.listdir(self.directory):
                if not name.startswith(self.prefix + "_"):
                    continue
                m = self._NAME_RE.search(name)
                if not m or name[len(self.prefix) + 1:-4] != m.group(1):
                    continue  # prefix가 다른 파일(snow_robot_error_ 등)과 섞이지 않게 정확히 일치할 때만
                path = os.path.join(self.directory, name)
                if os.path.getmtime(path) < limit:
                    os.remove(path)
        except Exception:
            pass

    def _switch(self, day: str):
        if self._fh is not None:
            try:
                self._fh.close()
            except Exception:
                pass
        self._fh = open(self._path(day), "a", encoding="utf-8")
        self._day = day
        self._cleanup()

    def emit(self, record):
        try:
            day = time.strftime("%Y-%m-%d")
            if day != self._day or self._fh is None:
                self._switch(day)
            self._fh.write(self.format(record) + "\n")
            self._fh.flush()
        except Exception:
            self.handleError(record)

    def close(self):
        try:
            if self._fh is not None:
                self._fh.close()
        finally:
            self._fh = None
            super().close()


class _NoTee(logging.Filter):
    """Tee에서 온 줄은 콘솔에 이미 찍혔으므로 콘솔 핸들러에서는 제외."""

    def filter(self, record):
        return not getattr(record, "from_tee", False)


class _Tee:
    """sys.stdout / sys.stderr 대체. 원래 스트림으로 그대로 내보내고 줄 단위로 로그에도 남긴다."""

    WARN_RE = re.compile(r"too slow|disconnect|no response|timeout|timed out|lost|unreachable", re.I)
    ERR_RE = re.compile(r"error|failed|fail\b|exception|traceback|fatal", re.I)

    def __init__(self, orig, is_err: bool, drop_re):
        self._orig = orig
        self._is_err = is_err
        self._drop = drop_re
        self._buf = ""
        self._lock = threading.Lock()

    def write(self, s):
        try:
            self._orig.write(s)
        except Exception:
            pass
        try:
            with self._lock:
                self._buf += s
                lines = []
                while "\n" in self._buf:
                    line, self._buf = self._buf.split("\n", 1)
                    lines.append(line)
            for line in lines:
                self._emit(line)
        except Exception:
            pass
        return len(s)

    def _emit(self, line: str):
        line = line.rstrip("\r")
        if not line.strip():
            return
        if self._drop is not None and self._drop.search(line):
            with _stats_lock:
                _stats["dropped"] += 1
            return
        if self._is_err:
            level = logging.WARNING
        elif self.ERR_RE.search(line):
            level = logging.ERROR
        elif self.WARN_RE.search(line):
            level = logging.WARNING
        else:
            level = logging.INFO
        log.log(level, line, extra={"from_tee": True})

    def flush(self):
        try:
            self._orig.flush()
        except Exception:
            pass

    def __getattr__(self, name):
        return getattr(self._orig, name)


# ----------------------------------------------------------------------
# 설정
# ----------------------------------------------------------------------
def setup_logging() -> str:
    """로그 설정. main() 맨 앞에서 한 번 호출. 로그 폴더 경로를 돌려준다."""
    global _setup_done
    # 기본 위치: 이 파일(= snow_robot_robot.py와 같은 폴더) 옆의 snow_robot_logs/
    here = os.path.dirname(os.path.abspath(__file__))
    log_dir = os.getenv("SNOW_LOG_DIR", os.path.join(here, "snow_robot_logs"))
    try:
        os.makedirs(log_dir, exist_ok=True)
        test = os.path.join(log_dir, ".w")
        open(test, "w").close()
        os.remove(test)
    except Exception:
        # 쓰기 권한이 없으면 홈 폴더로 대체
        log_dir = os.path.join(os.path.expanduser("~"), "snow_robot_logs")
        os.makedirs(log_dir, exist_ok=True)
    if _setup_done:
        return log_dir
    _setup_done = True

    fmt = logging.Formatter(
        "%(asctime)s.%(msecs)03d [%(levelname)-7s] %(threadName)s: %(message)s", "%Y-%m-%d %H:%M:%S"
    )
    log.setLevel(logging.DEBUG)
    log.propagate = False

    try:
        keep_days = int(os.getenv("SNOW_LOG_KEEP_DAYS", "30"))
    except ValueError:
        keep_days = 30

    fh = DailyFileHandler(log_dir, "snow_robot", keep_days, logging.DEBUG)
    fh.setFormatter(fmt)
    log.addHandler(fh)

    eh = DailyFileHandler(log_dir, "snow_robot_error", keep_days, logging.WARNING)
    eh.setFormatter(fmt)
    log.addHandler(eh)

    orig_out, orig_err = sys.stdout, sys.stderr
    ch = logging.StreamHandler(orig_out)  # Tee 설치 전의 진짜 stdout
    ch.setFormatter(fmt)
    ch.setLevel(logging.INFO)
    ch.addFilter(_NoTee())
    log.addHandler(ch)

    drop_pat = os.getenv("SNOW_LOG_DROP", r"axes.*buttons|buttons.*axes")
    try:
        drop_re = re.compile(drop_pat) if drop_pat else None
    except re.error:
        drop_re = None
    sys.stdout = _Tee(orig_out, False, drop_re)
    sys.stderr = _Tee(orig_err, True, None)

    def _excepthook(tp, val, tb):
        if issubclass(tp, KeyboardInterrupt):
            sys.__excepthook__(tp, val, tb)
            return
        log.critical("처리되지 않은 예외로 프로세스 종료", exc_info=(tp, val, tb))

    def _thread_hook(args):
        if args.exc_type is SystemExit:
            return
        name = args.thread.name if args.thread else "?"
        log.error(f"스레드 '{name}' 예외로 종료됨 (이 스레드의 기능이 멈춤)",
                  exc_info=(args.exc_type, args.exc_value, args.exc_traceback))

    def _unraisable(u):
        log.error(f"unraisable: {u.err_msg} obj={u.object!r}",
                  exc_info=(u.exc_type, u.exc_value, u.exc_traceback))

    sys.excepthook = _excepthook
    threading.excepthook = _thread_hook
    sys.unraisablehook = _unraisable

    def _sigterm(signum, frame):
        log.warning(f"signal {signum} 수신 -> 종료")
        raise SystemExit(0)

    try:
        signal.signal(signal.SIGTERM, _sigterm)
    except Exception:
        pass  # 메인 스레드가 아니면 무시

    atexit.register(lambda: log.info("프로세스 종료 (atexit)"))
    log.info("=" * 60)
    log.info(f"snow_robot 시작 pid={os.getpid()} python={sys.version.split()[0]} log_dir={log_dir}")
    return log_dir


# ----------------------------------------------------------------------
# 네트워크 감시
# ----------------------------------------------------------------------
class NetMonitor(threading.Thread):
    """서버로 TCP 연결을 시도해 도달 가능 여부와 지연(RTT)을 측정한다.

    연결 거부(ConnectionRefused)는 '서버 호스트까지는 도달'이므로 정상으로 본다.
    """

    def __init__(self, target: str, interval=2.0, timeout=1.5, down_after=3, slow_ms=500):
        super().__init__(name="NetMonitor", daemon=True)
        host, _, port = target.rpartition(":")
        self.host, self.port = host, int(port)
        self.interval, self.timeout = interval, timeout
        self.down_after, self.slow_ms = down_after, slow_ms
        self.up = True
        self.rtt_ms: Optional[float] = None
        self._fails = 0
        self._down_since = 0.0

    def run(self):
        log.info(f"[NET] 감시 시작 target={self.host}:{self.port}")
        while True:
            t0 = time.time()
            err = None
            try:
                s = socket.create_connection((self.host, self.port), timeout=self.timeout)
                s.close()
            except ConnectionRefusedError:
                pass
            except OSError as e:
                err = e
            rtt = (time.time() - t0) * 1000.0

            if err is None:
                self.rtt_ms = rtt
                if self._fails >= self.down_after and not self.up:
                    log.warning(f"[NET] UP 복구 (끊김 {time.time() - self._down_since:.0f}s, rtt={rtt:.0f}ms)")
                self.up = True
                self._fails = 0
                if rtt > self.slow_ms:
                    log_every("net_slow", 10.0, logging.WARNING, f"[NET] 지연 큼 rtt={rtt:.0f}ms")
            else:
                self.rtt_ms = None
                self._fails += 1
                if self._fails == 1:
                    self._down_since = t0
                if self._fails == self.down_after and self.up:
                    self.up = False
                    log.error(f"[NET] DOWN {self.host}:{self.port} 연속 {self._fails}회 실패: {err}")
            time.sleep(max(0.0, self.interval - (time.time() - t0)))


# ----------------------------------------------------------------------
# 헬스 모니터
# ----------------------------------------------------------------------
def _read_temp_c() -> Optional[float]:
    best = None
    try:
        base = "/sys/class/thermal"
        for d in os.listdir(base):
            if d.startswith("thermal_zone"):
                try:
                    v = int(open(os.path.join(base, d, "temp")).read().strip()) / 1000.0
                    best = v if best is None else max(best, v)
                except Exception:
                    pass
    except Exception:
        pass
    return best


def _read_mem_avail_mb() -> Optional[float]:
    try:
        for line in open("/proc/meminfo"):
            if line.startswith("MemAvailable:"):
                return int(line.split()[1]) / 1024.0
    except Exception:
        pass
    return None


class HealthMonitor(threading.Thread):
    """check 함수들을 1초마다 돌려 상태가 바뀔 때만 기록하고, period마다 [HB] 요약을 남긴다."""

    def __init__(self, net: Optional[NetMonitor], period: float = 10.0):
        super().__init__(name="HealthMonitor", daemon=True)
        self.net = net
        self.period = period
        self.checks: Dict[str, Callable[[], Tuple[bool, str]]] = {}
        self.extra: Dict[str, Callable[[], str]] = {}
        self._state: Dict[str, bool] = {}

    def add_check(self, name: str, fn: Callable[[], Tuple[bool, str]]):
        self.checks[name] = fn

    def add_summary(self, name: str, fn: Callable[[], str]):
        self.extra[name] = fn

    def run(self):
        last_hb = 0.0
        last_joy_total = 0
        last_joy_t = time.time()
        while True:
            for name, fn in list(self.checks.items()):
                try:
                    ok, detail = fn()
                except Exception as e:
                    ok, detail = False, f"check 예외: {e}"
                prev = self._state.get(name, True)
                if not ok and prev:
                    log.error(f"[HEALTH] {name} 이상: {detail}")
                elif ok and not prev:
                    log.warning(f"[HEALTH] {name} 복구")
                self._state[name] = ok

            now = time.time()
            if now - last_hb >= self.period:
                with _stats_lock:
                    total, last, dropped = _stats["joy_total"], _stats["joy_last"], _stats["dropped"]
                rate = (total - last_joy_total) / max(now - last_joy_t, 1e-6)
                last_joy_total, last_joy_t = total, now
                age = "-" if last == 0 else f"{now - last:.1f}s"
                net = "-"
                if self.net is not None:
                    net = ("up" if self.net.up else "DOWN") + (
                        f"/{self.net.rtt_ms:.0f}ms" if self.net.rtt_ms is not None else "/-")
                temp, mem = _read_temp_c(), _read_mem_avail_mb()
                try:
                    load = os.getloadavg()[0]
                except Exception:
                    load = -1
                parts = [
                    f"net={net}",
                    f"joy={rate:.1f}/s(last {age})",
                    f"temp={'-' if temp is None else f'{temp:.0f}C'}",
                    f"mem={'-' if mem is None else f'{mem:.0f}MB'}",
                    f"load={load:.1f}",
                ]
                for name, fn in self.extra.items():
                    try:
                        parts.append(f"{name}={fn()}")
                    except Exception:
                        parts.append(f"{name}=?")
                bad = [n for n, ok in self._state.items() if not ok]
                if bad:
                    parts.append("BAD=" + ",".join(bad))
                log.info("[HB] " + " ".join(parts))
                last_hb = now
            time.sleep(1.0)


def start_monitors(sub_controller=None, cameras: Optional[dict] = None,
                   ports: Optional[Dict[str, str]] = None) -> HealthMonitor:
    """네트워크 감시 + 헬스 체크 시작.

    sub_controller : SnowRobotSubController (모터 TX 스레드 / 짐벌 스레드 확인)
    cameras        : {"front_cam": SnowCamera, ...}  (health() 메서드가 있는 것만)
    ports          : {"motor": "/dev/ttyACM0", "gimbal": "/dev/ttyUSB0"}  존재 여부 확인
    """
    net = None
    target = os.getenv("SNOW_NET_TARGET", "175.126.123.199:443")
    if target:
        net = NetMonitor(target)
        net.start()

    hm = HealthMonitor(net, period=float(os.getenv("SNOW_HB_PERIOD", "10")))

    for name, path in (ports or {}).items():
        if path:
            hm.add_check(f"port:{name}", lambda p=path: (os.path.exists(p), f"{p} 장치 없음 (USB 분리/전원?)"))

    if sub_controller is not None:
        motion = getattr(sub_controller, "motion", None)
        if motion is not None:
            def _motor():
                if getattr(motion, "drive", None) is None:
                    return False, f"모터 시리얼 미연결 ({getattr(motion, 'hardware_error', None)})"
                th = getattr(motion, "_thread", None)
                if th is None or not th.is_alive():
                    return False, "모터 TX 스레드 죽음 -> 모터 명령이 안 나감"
                return True, ""
            hm.add_check("motor_tx", _motor)

            def _motor_resp():
                d = getattr(motion, "drive", None)
                if d is None or not hasattr(d, "feedback_health"):
                    return True, ""
                return d.feedback_health()
            hm.add_check("motor_resp", _motor_resp)
            def _motor_sum():
                d = getattr(motion, "drive", None)  # 재연결 중 None이 될 수 있어 한 번만 읽는다
                return d.feedback_summary() if d is not None else "DISCONNECTED"
            hm.add_summary("motor", _motor_sum)

        if getattr(sub_controller, "_gimbal_enabled", False):
            def _gimbal():
                th = getattr(sub_controller, "_gimbal_thread", None)
                if th is None or not th.is_alive():
                    return False, "짐벌 스레드 죽음"
                if getattr(sub_controller, "gimbal", None) is None:
                    return False, "짐벌 연결 안 됨 (재연결 시도 중)"
                return True, ""
            hm.add_check("gimbal", _gimbal)

    for cname, cam in (cameras or {}).items():
        if not hasattr(cam, "health"):
            continue

        def _cam(c=cam):
            h = c.health()
            if not h["alive"]:
                return False, f"캡처 스레드 죽음 (자동 재시작 시도 중, 재시작 {h['restarts']}회)"
            if h["age"] is None or h["age"] > 2.0:
                return False, f"프레임 없음 (마지막 프레임 {h['age']})"
            return True, ""
        hm.add_check(f"cam:{cname}", _cam)

        def _camsum(c=cam):
            h = c.health()
            a = "-" if h["age"] is None else f"{h['age']:.2f}s"
            return f"{a}/r{h['restarts']}"
        hm.add_summary(f"cam_{cname}", _camsum)

    def _temp():
        t = _read_temp_c()
        return (t is None or t < 80.0), f"온도 {t}C (쓰로틀링 위험)"
    hm.add_check("temp", _temp)

    def _mem():
        m = _read_mem_avail_mb()
        return (m is None or m > 200.0), f"가용 메모리 {m:.0f}MB"
    hm.add_check("mem", _mem)

    hm.start()
    return hm
