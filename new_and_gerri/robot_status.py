class RobotStatus:
    def __init__(self, robot_id=None, model=None, category=None):
        self.robot_id = robot_id
        self.robot_type = {
            "category": category,
            "model": model,
            "footprint": None,
            "size": {
                "width": None,
                "length": None,
                "height": None
            }
        }

        # ZYX
        self.pose = {
            "position": {
                "x": None,
                "y": None,
                "z": None
            },
            "orientation": {
                "yaw": None,
                "pitch": None,
                "roll": None,
            },
            "2d": {
                "x": None,
                "y": None,
                "th": None
            }
        }

        self.velocity = {
            "linear": {
                "x": None,
                "y": None,
                "z": None
            },
            "angular": {
                "x": None,
                "y": None,
                "z": None
            },
            "2d": {
                "vx": None,
                "vy": None,
                "vth": None,
            }
        }
        self.battery_state = {
            "voltage": None,
            "current": None,
            "capacity": None,
            "percentage": None,
            "temperature": None,
            "now_charging": None
        }
        self.joint_state = {
            "name": None,
            "position": None,
            "velocity": None,
            "effort": None
        }
        self.path_plan = {
            "global": None,
            "local": None,
        }
        self.sensor = {
            "ultrasonic": None,
            "laser": None
        }
        self.map = {
            "name": None,
            "origin": None,
            "size": None,
            "resolution": None
        }

    def model_dump(self):
        return {
            "robot_id": self.robot_id,
            "robot_type": self.robot_type,
            "pose": self.pose,
            "velocity": self.velocity,
            "battery_state": self.battery_state,
            "joint_state": self.joint_state,
            "path_plan": self.path_plan,
            "sensor": self.sensor,
            "map": self.map,
        }