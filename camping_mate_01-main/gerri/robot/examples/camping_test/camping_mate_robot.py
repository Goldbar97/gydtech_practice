#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import time
from pathlib import Path


def _bootstrap_repo_root():
    """
    어디서 실행하든(_and_, gerri import) 깨지지 않게 repo root를 sys.path 맨 앞에 추가.
    조건: root 아래에 '_and_' 폴더와 'gerri' 폴더가 함께 존재.
    """
    here = Path(__file__).resolve()
    for p in [here.parent] + list(here.parents):
        if (p / "_and_").is_dir() and (p / "gerri").is_dir():
            sys.path.insert(0, str(p))
            return
    # fallback (그래도 못 찾으면, 상위로 꽤 올라가서 한번 넣어줌)
    sys.path.insert(0, str(here.parents[4]))


_bootstrap_repo_root()

from _and_.and_robot import AdaptiveNetworkDaemon
from gerri.robot.examples.camping_test.camping_mate_config import ROBOT_INFO, VIDEO_INFO, AUDIO_INFO
from gerri.robot.examples.camping_test.camping_mate_base_controller import CampingMateBaseController
from gerri.robot.examples.camping_test.camping_mate_sub_controller import CampingMateSubController


def main():
    daemon = AdaptiveNetworkDaemon(
        robot_info=ROBOT_INFO,
        network="ketirtc",
        command="command",
        video_info=VIDEO_INFO,
        audio_info=AUDIO_INFO,
    )
    daemon.connect()

    base = CampingMateBaseController(ROBOT_INFO)
    sub = CampingMateSubController()

    base.sub_controller = sub
    sub.base_controller = base

    base.connect()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            base.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    main()



