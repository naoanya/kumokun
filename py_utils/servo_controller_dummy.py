import json
import time


class ServoControllerDummy:
    """Simulated servo controller for testing without serial hardware.

    Provides the same high-level API as `ServoController` but does not open
    any serial port. Positions are stored in-memory (IDs 1..18, neutral 7500).
    """

    def __init__(self):
        self.is_connected = False
        self.read_timeout = 0.5
        # positions keyed by physical id (1..18)
        self.positions = {i: 7500 for i in range(1, 19)}

    def list_ports(self):
        return ["SIMULATED_PORT"]

    def connect(self, port, baudrate=115200):
        self.is_connected = True
        return True, "Connected (simulated)"

    def disconnect(self):
        self.is_connected = False
        return True

    # Lightweight single-line command helpers
    def set_pos_light(self, servo_id, pos):
        self.positions[int(servo_id)] = int(pos)
        return json.dumps({"status": "OK", "command": "XS", "id": int(servo_id), "pos": int(pos)}), None

    def get_pos_light(self, servo_id):
        val = self.positions.get(int(servo_id), 7500)
        return json.dumps({"status": "OK", "command": "XG", "id": int(servo_id), "pos": val, "feedback": val}), None

    def free_light(self, servo_id):
        val = self.positions.get(int(servo_id), 7500)
        return json.dumps({"status": "OK", "command": "XF", "id": int(servo_id), "pos": val}), None

    def set_pos(self, servo_id, pos):
        self.positions[int(servo_id)] = int(pos)
        return json.dumps({"status": "OK", "command": "SETPOS", "id": int(servo_id), "pos": int(pos)}), None

    def get_pos(self, servo_id):
        val = self.positions.get(int(servo_id), 7500)
        return json.dumps({"status": "OK", "command": "GETPOS", "id": int(servo_id), "pos": val, "feedback": val}), None

    def free(self, servo_id):
        val = self.positions.get(int(servo_id), 7500)
        return json.dumps({"status": "OK", "command": "FREE", "id": int(servo_id), "pos": val}), None

    # Bulk commands
    def set_all_pos(self, positions):
        if len(positions) != 18:
            return None, "Must provide exactly 18 positions"
        for i, v in enumerate(positions, start=1):
            self.positions[i] = int(v)
        responses = [json.dumps({"status": "OK", "command": "AS", "id": i, "pos": self.positions[i], "feedback": self.positions[i]}) for i in range(1, 19)]
        return responses, None

    def get_all_pos(self):
        responses = [json.dumps({"status": "OK", "command": "AG", "id": i, "pos": self.positions[i], "feedback": self.positions[i]}) for i in range(1, 19)]
        return responses, None

    def free_all(self):
        # In simulation 'free' does not change positions, just returns current
        responses = [json.dumps({"status": "OK", "command": "AF", "id": i, "pos": self.positions[i], "feedback": self.positions[i]}) for i in range(1, 19)]
        return responses, None

    # Parameter and EEPROM helpers (simulate OK)
    def set_stretch(self, servo_id, val):
        return json.dumps({"status": "OK"}), None

    def get_stretch(self, servo_id):
        return json.dumps({"status": "OK", "value": 0}), None

    def set_speed(self, servo_id, val):
        return json.dumps({"status": "OK"}), None

    def get_speed(self, servo_id):
        return json.dumps({"status": "OK", "value": 0}), None

    def set_current(self, servo_id, val):
        return json.dumps({"status": "OK"}), None

    def get_current(self, servo_id):
        return json.dumps({"status": "OK", "value": 0}), None

    def set_temp(self, servo_id, val):
        return json.dumps({"status": "OK"}), None

    def get_temp(self, servo_id):
        return json.dumps({"status": "OK", "value": 25}), None

    def read_eeprom(self, servo_id):
        return json.dumps({"status": "OK", "id": int(servo_id)}), None

    def write_eeprom(self, servo_id):
        return json.dumps({"status": "OK", "id": int(servo_id)}), None

    def dump_eeprom(self):
        return json.dumps({"status": "OK", "dump": {}}), None

    def ehelp(self):
        return json.dumps({"status": "OK", "help": []}), None

    def eget(self, field):
        return json.dumps({"status": "OK", "field": field, "value": ""}), None

    def eset(self, field, value):
        return json.dumps({"status": "OK", "field": field, "value": value}), None
