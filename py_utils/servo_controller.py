import serial
import serial.tools.list_ports
import time
from collections import deque


class ServoController:
    """ICS servo controller communication class"""
    
    def __init__(self):
        self.ser = None
        self.is_connected = False
        self.read_timeout = 0.5
        self.rx_buffer = deque(maxlen=1000)
        
    def list_ports(self):
        """Get a list of available serial ports"""
        ports = []
        for port, desc, hwid in serial.tools.list_ports.comports():
            ports.append(f"{port} - {desc}")
        return ports if ports else ["No ports available"]
    
    def connect(self, port, baudrate=115200):
        """Connect to the serial port"""
        try:
            # Extract COM port name from provided string
            com_port = port.split(' ')[0] if ' ' in port else port
            self.ser = serial.Serial(
                port=com_port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_EVEN,  # ICS protocol requirement
                stopbits=serial.STOPBITS_ONE,
                timeout=self.read_timeout
            )
            self.is_connected = True
            return True, "Connected successfully"
        except Exception as e:
            self.is_connected = False
            return False, f"Connection failed: {str(e)}"
    
    def disconnect(self):
        """Disconnect from the serial port"""
        if self.ser:
            self.ser.close()
            self.is_connected = False
            return True
        return False
    
    def send_command(self, cmd_str):
        """Send a command"""
        if not self.is_connected:
            return False, "Not connected"
        
        try:
            self.ser.write((cmd_str + '\n').encode())
            self.ser.flush()
            return True, "Sent"
        except Exception as e:
            return False, f"Send error: {str(e)}"
    
    def read_response(self, timeout=1.0):
        """Read a single-line response"""
        if not self.is_connected:
            return None
        
        start_time = time.time()
        response = ""
        
        while time.time() - start_time < timeout:
            if not self.is_connected:
                return None
            
            try:
                if self.ser.in_waiting > 0:
                    char = self.ser.read(1).decode('utf-8', errors='ignore')
                    if char == '\n':
                        return response.strip()
                    response += char
            except Exception as e:
                return None
            #time.sleep(0.0001)
        
        return None
    
    def send_and_receive(self, cmd_str, timeout=1.0):
        """Send a command and read a single-line response"""
        sent_ok, msg = self.send_command(cmd_str)
        if not sent_ok:
            return None, msg
        
        response = self.read_response(timeout)
        if response is None:
            return None, "Timeout or Read Error"
        return response, None

    def send_and_receive_lines(self, cmd_str, lines_expected, timeout=2.0):
        """Send a command and read multiple response lines"""
        sent_ok, msg = self.send_command(cmd_str)
        if not sent_ok:
            return None, msg
        
        responses = []
        start_time = time.time()
        
        while len(responses) < lines_expected:
            remaining = timeout - (time.time() - start_time)
            if remaining <= 0:
                break
            
            resp = self.read_response(timeout=remaining)
            if resp:
                responses.append(resp)
                # Check for global error on first line (e.g. invalid parameters)
                if len(responses) == 1 and '"status":"NG"' in resp and ('"reason":"invalid_parameters"' in resp or '"reason":"unknown_command"' in resp):
                    break
            else:
                break
        
        if not responses:
            return None, "Timeout or Read Error"
            
        return responses, None

    # --- Position Control Commands ---
    
    # --- Bulk Control Commands ---
    def set_all_pos(self, positions):
        """Set positions for all servos (AS)"""
        if len(positions) != 18:
            return None, "Must provide exactly 18 positions"
        pos_str = " ".join(map(str, positions))
        return self.send_and_receive_lines(f"AS {pos_str}", 18, timeout=4.0)

    def get_all_pos(self):
        """Get positions of all servos (AG)"""
        return self.send_and_receive_lines("AG", 18, timeout=4.0)

    def free_all(self):
        """Release torque on all servos (AF)"""
        return self.send_and_receive_lines("AF", 18, timeout=4.0)

    # --- Individual Control Commands ---
    def set_pos_light(self, servo_id, pos):
        return self.send_and_receive(f"XS {servo_id} {pos}")

    def get_pos_light(self, servo_id):
        return self.send_and_receive(f"XG {servo_id}")

    def free_light(self, servo_id):
        return self.send_and_receive(f"XF {servo_id}")

    def set_pos(self, servo_id, pos):
        return self.send_and_receive(f"SETPOS {servo_id} {pos}")

    def get_pos(self, servo_id):
        return self.send_and_receive(f"GETPOS {servo_id}")

    def free(self, servo_id):
        return self.send_and_receive(f"FREE {servo_id}")

    # --- Parameter Commands ---
    def set_stretch(self, servo_id, val):
        return self.send_and_receive(f"STRETCH {servo_id} {val}")

    def get_stretch(self, servo_id):
        return self.send_and_receive(f"GSTRETCH {servo_id}")

    def set_speed(self, servo_id, val):
        return self.send_and_receive(f"SPEED {servo_id} {val}")

    def get_speed(self, servo_id):
        return self.send_and_receive(f"GSPEED {servo_id}")

    def set_current(self, servo_id, val):
        return self.send_and_receive(f"CURRENT {servo_id} {val}")

    def get_current(self, servo_id):
        return self.send_and_receive(f"GCURRENT {servo_id}")

    def set_temp(self, servo_id, val):
        return self.send_and_receive(f"TEMP {servo_id} {val}")

    def get_temp(self, servo_id):
        return self.send_and_receive(f"GTEMP {servo_id}")

    # --- EEPROM Commands ---
    def read_eeprom(self, servo_id):
        return self.send_and_receive(f"READEPROM {servo_id}", timeout=2.0)

    def write_eeprom(self, servo_id):
        return self.send_and_receive(f"WRITEEPROM {servo_id}", timeout=2.0)

    def dump_eeprom(self):
        return self.send_and_receive("DUMP")

    def ehelp(self):
        return self.send_and_receive("EHELP")

    def eget(self, field):
        return self.send_and_receive(f"EGET {field}")

    def eset(self, field, value):
        return self.send_and_receive(f"ESET {field} {value}")