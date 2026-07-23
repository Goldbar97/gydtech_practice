from gerri.interface.joy_handler.core.joystick_handler import JoyMessageHandler
from gerri.interface.joy_handler.protocols.rubbber_neck_joystick import RubberNeckJoyStickProtocol
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..camping_mate_controller import CampingMateController

BUTTONS = {
    "A": 0, "B": 1, "X": 2, "Y": 3,
    "LB": 4, "RB": 5, "NONE_BUTTON1": 6, "NONE_BUTTON2": 7,
    "BACK": 8, "START": 9, "C1": 10, "C2": 11,
    "FORWARD": 12, "BACKWARD": 13, "LEFT": 14, "RIGHT": 15,
    "PASS1": 16
}

AXES = {
    "AXIS_1_X": 0, "AXIS_1_Y": 1, "AXIS_2_X": 2, "AXIS_2_Y": 3,
    "LT": 4, "RT": 5,
}

class RubberNeckJoyStickHandler(JoyMessageHandler):
    def __init__(self,sub_controller=None):
        super().__init__(buttons=BUTTONS, axes=AXES, freq=0.05, psudo_release_event=True)
        self.sub_controller:CampingMateController = sub_controller
        
        self.robot_type = self.sub_controller.status.robot_type.get("category").lower() if self.sub_controller else "mobile"
        self.register_callback(self.handle_joy)

        self.last_message = ""

        self._prev_edge_state = {}

    def _rising_edge(self, name: str, currently_pressed: bool):
        prev = self._prev_edge_state.get(name, False)
        self._prev_edge_state[name] = currently_pressed
        return currently_pressed and not prev

    def safty_call(self, method_name:str):
        try:
            getattr(self.sub_controller, method_name)()
        except AttributeError:
            message = f"'{method_name}' method is not implemented in {self.robot_type} sub_controller"
            if message != self.last_message:
                print(message)
                self.last_message = message

    def handle_joy(self,handler: RubberNeckJoyStickProtocol):
        vx = 0.0
        vy = 0.0
        vth = 0.0

        roll = 0.0
        pitch = 0.0

        # Movement control
        if handler.AXIS_1_Y.is_pressed():
            if handler.AXIS_1_Y.get_value() != 0.0:
                vx = handler.AXIS_1_Y.get_value()

        if handler.AXIS_1_X.is_pressed():
            if handler.AXIS_1_X.get_value() != 0.0:
                vy = handler.AXIS_1_X.get_value()

        if handler.AXIS_2_X.is_pressed():
            if handler.AXIS_2_X.get_value() != 0.0:
                vth = handler.AXIS_2_X.get_value()

        if vx != 0.0 or vy != 0.0 or vth != 0.0:
            self.sub_controller.move(vx=vx,vy=vy,vth=vth)

        if self._rising_edge("B", handler.B.is_pressed()) or self._rising_edge("C1", handler.C1.is_pressed()):
            self.sub_controller.slower()

        if self._rising_edge("Y", handler.Y.is_pressed()) or self._rising_edge("C2", handler.C2.is_pressed()):
            self.sub_controller.faster()

        if handler.RT.is_pressed():
            pass
 
        if handler.LT.is_pressed():
            pass

        # Lease Control
        if handler.C1.is_released():
            pass
        if handler.C2.is_released():
            pass

        # Stance
        if handler.A.is_released():
            pass
        if handler.X.is_released():
            pass

        # Mission Control
        if handler.START.is_released():
            pass
        if handler.BACK.is_released():
            pass
        if handler.Y.is_released():
            pass
        
        # Operation Mode
        if handler.LB.is_released():
            pass
        if handler.RB.is_released():
            pass
        