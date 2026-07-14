# -*- coding: utf-8 -*-
from pubsub import pub

from gerri.robot.function.manipulator_function import ManipulatorFunction
from gerri.robot.function.mobile_function import MobileFunction
from gerri.robot.status_manager import StatusManager


class CampingMateBaseController(ManipulatorFunction, MobileFunction):
    def __init__(self, robot_info, **params):
        self.robot_info = robot_info
        self.robot_id = robot_info["id"]
        self.robot_category = robot_info["category"]
        self.robot_model = robot_info["model"]
        self.sub_controller = None

        pub.subscribe(self.receive_message, "receive_message")

    def receive_message(self, message):
        # 기존 기능 유지
        ManipulatorFunction.receive_message(self, message=message)
        MobileFunction.receive_message(self, message=message)

        if not isinstance(message, dict):
            return

        topic = message.get("topic")
        value = message.get("value", {})
        option = message.get("option")
        target = message.get("target")

        # ✅ rubberneck 키/조이스틱 입력은 target 없이 오는 경우가 많아서,
        #    /joy, move 류는 target 상관없이 항상 라우팅한다.
        if topic in ("/joy", "joy", "/move", "move", "/open", "open", "/stop", "stop"):
            if self.sub_controller:
                try:
                    self.sub_controller.on_message(topic=topic, value=value, option=option, raw=message)
                except Exception as e:
                    print(f"[camping_mate] on_message error: {e}")

        # 기존 target 기반 호출도 유지
        if target is not None and (target in self.robot_id or target == "all"):
            try:
                if topic == "hello_universe":
                    self.sub_controller.hello_universe(value)
                else:
                    self.send_message(
                        {"topic": "callback_" + str(topic), "value": "callback", "target": "all"}
                    )
            except AttributeError as e:
                print(f"❌ Controller does not support topic '{topic}': {e}")
            except Exception as e:
                print(f"❌ Error processing topic '{topic}': {e}")

    def send_message(self, message):
        pub.sendMessage("send_message", message=message)

    def connect(self):
        self.sub_controller.connect()
        self.status_manager = StatusManager(self.robot_info, self.sub_controller)

    def disconnect(self):
        try:
            self.sub_controller.disconnect()
        finally:
            pass



