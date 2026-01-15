#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ICS Servo Controller GUI Application (Tkinter Version)
GUI application to operate the ICS servo controller on Raspberry Pi Pico
"""

import json
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
from collections import deque
from datetime import datetime
from servo_controller import ServoController

# Add py_utils to sys.path so local imports work
current_dir = os.path.dirname(os.path.abspath(__file__))
py_utils_path = os.path.join(os.path.dirname(current_dir), 'py_utils')
if py_utils_path not in sys.path:
    sys.path.append(py_utils_path)

class ServoControllerGUI:
    """Servo Controller GUI (Tkinter)"""
    
    def __init__(self, root):
        self.root = root
        self.root.title("ICS Servo Controller GUI")
        self.root.geometry("1000x700")
        
        self.controller = ServoController()
        self.selected_servo_id = 1
        self.log_messages = deque(maxlen=100)
        
        # Window settings
        style = ttk.Style()
        style.theme_use('clam')
        
        self.setup_ui()
        self.update_port_list()
    
    def log(self, msg):
        """Append a log message"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_msg = f"[{timestamp}] {msg}"
        self.log_messages.append(log_msg)
        self.status_label.config(text=log_msg)
        return log_msg
    
    def setup_ui(self):
        """Initial UI setup"""
        # Main frame
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Title
        title = ttk.Label(main_frame, text="ICS Servo Controller", font=("Arial", 16, "bold"))
        title.pack(pady=10)
        
        # Notebook (tabs)
        self.notebook = ttk.Notebook(main_frame)
        self.notebook.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # Add tabs
        self.setup_connection_tab()
        self.setup_position_tab()
        self.setup_parameters_tab()
        self.setup_eeprom_tab()
        self.setup_terminal_tab()
        self.setup_help_tab()
        
        # Status bar
        status_frame = ttk.Frame(main_frame)
        status_frame.pack(fill=tk.X, pady=5)
        self.status_label = ttk.Label(status_frame, text="Ready", relief=tk.SUNKEN)
        self.status_label.pack(side=tk.LEFT, fill=tk.X, expand=True)
    
    def setup_connection_tab(self):
        """Configure the Connection tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Connection")
        
        frame = ttk.LabelFrame(tab, text="Serial Port Configuration", padding=10)
        frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Port selection
        ttk.Label(frame, text="Port:").grid(row=0, column=0, sticky=tk.W, padx=5, pady=5)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(frame, textvariable=self.port_var, width=40, state='readonly')
        self.port_combo.grid(row=0, column=1, padx=5, pady=5)
        
        ttk.Button(frame, text="Refresh Ports", command=self.update_port_list).grid(row=0, column=2, padx=5, pady=5)
        
        # Baud rate selection
        ttk.Label(frame, text="Baud Rate:").grid(row=1, column=0, sticky=tk.W, padx=5, pady=5)
        self.baud_var = tk.StringVar(value="115200")
        baud_combo = ttk.Combobox(frame, textvariable=self.baud_var, 
                                  values=["115200", "625000", "1250000"], width=40, state='readonly')
        baud_combo.grid(row=1, column=1, padx=5, pady=5)
        
        # Connection buttons
        button_frame = ttk.Frame(frame)
        button_frame.grid(row=2, column=0, columnspan=3, sticky=tk.W, padx=5, pady=10)
        
        ttk.Button(button_frame, text="Connect", command=self.connect_serial).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Disconnect", command=self.disconnect_serial).pack(side=tk.LEFT, padx=5)
        
        self.connection_status = ttk.Label(button_frame, text="Status: Disconnected", foreground="red")
        self.connection_status.pack(side=tk.LEFT, padx=20)
        
        # Servo ID configuration
        id_frame = ttk.LabelFrame(tab, text="Servo ID Configuration", padding=10)
        id_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(id_frame, text="Select Servo ID:").grid(row=0, column=0, sticky=tk.W, padx=5, pady=5)
        self.servo_id_var = tk.IntVar(value=1)
        id_spin = ttk.Spinbox(id_frame, from_=1, to=31, textvariable=self.servo_id_var, width=5)
        id_spin.grid(row=0, column=1, sticky=tk.W, padx=5, pady=5)
        
        ttk.Button(id_frame, text="Set Default ID", command=self.set_servo_id).grid(row=0, column=2, padx=5, pady=5)
    
    def setup_position_tab(self):
        """Configure the Position Control tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Position Control")
        
        # Lightweight command frame
        light_frame = ttk.LabelFrame(tab, text="Quick Position Control (Lightweight Commands)", padding=10)
        light_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(light_frame, text="Position (0-16383):").pack(anchor=tk.W, padx=5, pady=5)
        
        pos_frame = ttk.Frame(light_frame)
        pos_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.pos_slider = ttk.Scale(pos_frame, from_=0, to=16383, orient=tk.HORIZONTAL)
        self.pos_slider.set(8192)
        self.pos_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
        self.pos_display = ttk.Label(pos_frame, text="8192", width=6)
        self.pos_display.pack(side=tk.LEFT, padx=5)
        
        self.pos_slider.configure(command=self.update_pos_display)
        
        button_frame = ttk.Frame(light_frame)
        button_frame.pack(fill=tk.X, padx=5, pady=10)
        ttk.Button(button_frame, text="XS - Set Position", command=self.cmd_xs).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="XG - Get Position", command=self.cmd_xg).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="XF - Free (Release)", command=self.cmd_xf).pack(side=tk.LEFT, padx=5)
        
        # Standard commands frame
        std_frame = ttk.LabelFrame(tab, text="Standard Commands", padding=10)
        std_frame.pack(fill=tk.X, padx=5, pady=5)
        
        button_frame = ttk.Frame(std_frame)
        button_frame.pack(fill=tk.X, padx=5, pady=10)
        ttk.Button(button_frame, text="SETPOS", command=self.cmd_setpos).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="GETPOS", command=self.cmd_getpos).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="FREE", command=self.cmd_free).pack(side=tk.LEFT, padx=5)
        
        # Output frame
        output_frame = ttk.LabelFrame(tab, text="Output", padding=10)
        output_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        self.position_output = scrolledtext.ScrolledText(output_frame, height=10, width=80, state=tk.DISABLED)
        self.position_output.pack(fill=tk.BOTH, expand=True)
    
    def setup_parameters_tab(self):
        """Configure the Parameters tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Parameters")
        
        # Stretch
        stretch_frame = ttk.Frame(tab)
        stretch_frame.pack(fill=tk.X, padx=5, pady=5)
        ttk.Label(stretch_frame, text="Stretch (1-127):").pack(side=tk.LEFT, padx=5)
        self.stretch_var = tk.IntVar(value=60)
        ttk.Spinbox(stretch_frame, from_=1, to=127, textvariable=self.stretch_var, width=5).pack(side=tk.LEFT, padx=5)
        ttk.Button(stretch_frame, text="Set", command=self.cmd_set_stretch).pack(side=tk.LEFT, padx=5)
        ttk.Button(stretch_frame, text="Get", command=self.cmd_get_stretch).pack(side=tk.LEFT, padx=5)
        
        # Speed
        speed_frame = ttk.Frame(tab)
        speed_frame.pack(fill=tk.X, padx=5, pady=5)
        ttk.Label(speed_frame, text="Speed (1-127):").pack(side=tk.LEFT, padx=5)
        self.speed_var = tk.IntVar(value=60)
        ttk.Spinbox(speed_frame, from_=1, to=127, textvariable=self.speed_var, width=5).pack(side=tk.LEFT, padx=5)
        ttk.Button(speed_frame, text="Set", command=self.cmd_set_speed).pack(side=tk.LEFT, padx=5)
        ttk.Button(speed_frame, text="Get", command=self.cmd_get_speed).pack(side=tk.LEFT, padx=5)
        
        # Current
        current_frame = ttk.Frame(tab)
        current_frame.pack(fill=tk.X, padx=5, pady=5)
        ttk.Label(current_frame, text="Current (1-63):").pack(side=tk.LEFT, padx=5)
        self.current_var = tk.IntVar(value=30)
        ttk.Spinbox(current_frame, from_=1, to=63, textvariable=self.current_var, width=5).pack(side=tk.LEFT, padx=5)
        ttk.Button(current_frame, text="Set", command=self.cmd_set_current).pack(side=tk.LEFT, padx=5)
        ttk.Button(current_frame, text="Get", command=self.cmd_get_current).pack(side=tk.LEFT, padx=5)
        
        # Temperature
        temp_frame = ttk.Frame(tab)
        temp_frame.pack(fill=tk.X, padx=5, pady=5)
        ttk.Label(temp_frame, text="Temp Limit (1-127):").pack(side=tk.LEFT, padx=5)
        self.temp_var = tk.IntVar(value=70)
        ttk.Spinbox(temp_frame, from_=1, to=127, textvariable=self.temp_var, width=5).pack(side=tk.LEFT, padx=5)
        ttk.Button(temp_frame, text="Set", command=self.cmd_set_temp).pack(side=tk.LEFT, padx=5)
        ttk.Button(temp_frame, text="Get", command=self.cmd_get_temp).pack(side=tk.LEFT, padx=5)
        
        # Output frame
        output_frame = ttk.LabelFrame(tab, text="Output", padding=10)
        output_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        self.parameters_output = scrolledtext.ScrolledText(output_frame, height=10, width=80, state=tk.DISABLED)
        self.parameters_output.pack(fill=tk.BOTH, expand=True)
    
    def setup_eeprom_tab(self):
        """Configure the EEPROM tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="EEPROM")
        
        # Bulk operations
        button_frame = ttk.Frame(tab)
        button_frame.pack(fill=tk.X, padx=5, pady=5)
        ttk.Button(button_frame, text="Read EEPROM", command=self.cmd_readeprom).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Write EEPROM", command=self.cmd_writeeprom).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Dump EEPROM", command=self.cmd_dump).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="EHELP", command=self.cmd_ehelp).pack(side=tk.LEFT, padx=5)
        
        # Field editing
        field_frame = ttk.Frame(tab)
        field_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(field_frame, text="Field Name:").pack(side=tk.LEFT, padx=5)
        fields = ['stretch', 'speed', 'punch', 'deadband', 'damping', 'protection',
                 'templimit', 'currentlimit', 'response', 'useroffset', 'id',
                 'pullupper', 'pulllower', 'charstretch1', 'charstretch2', 'charstretch3',
                 'rotmode', 'slave', 'reverse', 'free', 'pwminh', 'baudrate']
        self.eeprom_field_var = tk.StringVar(value='stretch')
        field_combo = ttk.Combobox(field_frame, textvariable=self.eeprom_field_var, 
                                   values=fields, width=20, state='readonly')
        field_combo.pack(side=tk.LEFT, padx=5)
        
        ttk.Label(field_frame, text="Value:").pack(side=tk.LEFT, padx=5)
        self.eeprom_value_var = tk.StringVar()
        ttk.Entry(field_frame, textvariable=self.eeprom_value_var, width=10).pack(side=tk.LEFT, padx=5)
        ttk.Button(field_frame, text="Get", command=self.cmd_eget).pack(side=tk.LEFT, padx=5)
        ttk.Button(field_frame, text="Set", command=self.cmd_eset).pack(side=tk.LEFT, padx=5)
        
        # Output frame
        output_frame = ttk.LabelFrame(tab, text="Output", padding=10)
        output_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        self.eeprom_output = scrolledtext.ScrolledText(output_frame, height=15, width=80, state=tk.DISABLED)
        self.eeprom_output.pack(fill=tk.BOTH, expand=True)
    
    def setup_terminal_tab(self):
        """Configure the Terminal tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Terminal")
        
        cmd_frame = ttk.Frame(tab)
        cmd_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(cmd_frame, text="Enter command:").pack(side=tk.LEFT, padx=5)
        self.terminal_input = ttk.Entry(cmd_frame, width=60)
        self.terminal_input.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
        self.terminal_input.bind('<Return>', lambda e: self.cmd_terminal_send())
        
        ttk.Button(cmd_frame, text="Send", command=self.cmd_terminal_send).pack(side=tk.LEFT, padx=5)
        ttk.Button(cmd_frame, text="Clear Log", command=self.clear_terminal_log).pack(side=tk.LEFT, padx=5)
        
        # Output frame
        output_frame = ttk.Frame(tab)
        output_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        self.terminal_output = scrolledtext.ScrolledText(output_frame, height=20, width=80, state=tk.DISABLED)
        self.terminal_output.pack(fill=tk.BOTH, expand=True)
    
    def setup_help_tab(self):
        """Configure the Help tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Help")
        
        help_text = """ICS Servo Controller GUI - Command Reference

Lightweight Commands (Fast Communication):
  XS <id> <pos>  - Set position (returns: {"id":1,"pos":8192})
  XG <id>        - Get position (returns: {"id":1,"pos":8192})
  XF <id>        - Free servo   (returns: {"id":1,"pos":0})

Standard Position Commands:
  SETPOS <id> <pos> - Set position with full feedback
  GETPOS <id>       - Get current position
  FREE <id>         - Release servo

Parameter Commands:
  STRETCH <id> <val>  - Set stretch parameter
  GSTRETCH <id>       - Get stretch parameter
  SPEED <id> <val>    - Set speed parameter
  GSPEED <id>         - Get speed parameter
  CURRENT <id> <val>  - Set current limit
  GCURRENT <id>       - Get current limit
  TEMP <id> <val>     - Set temperature limit
  GTEMP <id>          - Get temperature limit

EEPROM Commands:
  READEPROM <id>      - Read EEPROM from servo
  WRITEEPROM <id>     - Write EEPROM to servo
  DUMP                - Display EEPROM hex dump
  EGET <field>        - Get EEPROM field value
  ESET <field> <val>  - Set EEPROM field value
  EHELP               - Show EEPROM field names

Position Range: 0-16383
Servo ID Range: 1-31

For more help, type "HELP" in the terminal."""
        
        text_widget = scrolledtext.ScrolledText(tab, height=25, width=80, state=tk.DISABLED)
        text_widget.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        text_widget.config(state=tk.NORMAL)
        text_widget.insert(tk.END, help_text)
        text_widget.config(state=tk.DISABLED)
    
    # Port-related methods
    def update_port_list(self):
        """Update the list of serial ports"""
        ports = self.controller.list_ports()
        self.port_combo['values'] = ports
        if ports and ports[0] != "No ports available":
            self.port_combo.current(0)
    
    def connect_serial(self):
        """Establish serial connection"""
        port = self.port_var.get()
        baud = int(self.baud_var.get())
        
        if not port:
            messagebox.showerror("Error", "Please select a port")
            return
        
        ok, msg = self.controller.connect(port, baud)
        if ok:
            self.connection_status.config(text="Status: Connected", foreground="green")
            self.log(msg)
        else:
            messagebox.showerror("Connection Error", msg)
            self.log(msg)
    
    def disconnect_serial(self):
        """Disconnect serial connection"""
        self.controller.disconnect()
        self.connection_status.config(text="Status: Disconnected", foreground="red")
        self.log("Disconnected")
    
    def set_servo_id(self):
        """Set the selected servo ID"""
        self.selected_servo_id = self.servo_id_var.get()
        self.log(f"Servo ID set to {self.selected_servo_id}")
    
    def update_pos_display(self, value):
        """Update the position slider display"""
        self.pos_display.config(text=str(int(float(value))))
    
    # Command execution methods
    def show_command_result(self, cmd_str, response, err, output_widget):
        """Show command execution results"""
        if err == "Not connected":
             messagebox.showerror("Error", "Not connected")
             return

        output = self.log(f"Command: {cmd_str}")
        
        if err:
            output += f"\nError: {err}"
        else:
            output += f"\nResponse: {response}" if response else "\n(No response)"
            try:
                if response and response.startswith('{'):
                    data = json.loads(response)
                    output += f"\n[Parsed] {json.dumps(data, indent=2)}"
            except:
                pass
        
        output_widget.config(state=tk.NORMAL)
        output_widget.insert(tk.END, output + "\n\n")
        output_widget.see(tk.END)
        output_widget.config(state=tk.DISABLED)
    
    # Position control commands
    def cmd_xs(self):
        pos = int(self.pos_slider.get())
        response, err = self.controller.set_pos_light(self.selected_servo_id, pos)
        self.show_command_result(f"XS {self.selected_servo_id} {pos}", response, err, self.position_output)
    
    def cmd_xg(self):
        response, err = self.controller.get_pos_light(self.selected_servo_id)
        self.show_command_result(f"XG {self.selected_servo_id}", response, err, self.position_output)
    
    def cmd_xf(self):
        response, err = self.controller.free_light(self.selected_servo_id)
        self.show_command_result(f"XF {self.selected_servo_id}", response, err, self.position_output)
    
    def cmd_setpos(self):
        pos = int(self.pos_slider.get())
        response, err = self.controller.set_pos(self.selected_servo_id, pos)
        self.show_command_result(f"SETPOS {self.selected_servo_id} {pos}", response, err, self.position_output)
    
    def cmd_getpos(self):
        response, err = self.controller.get_pos(self.selected_servo_id)
        self.show_command_result(f"GETPOS {self.selected_servo_id}", response, err, self.position_output)
    
    def cmd_free(self):
        response, err = self.controller.free(self.selected_servo_id)
        self.show_command_result(f"FREE {self.selected_servo_id}", response, err, self.position_output)
    
    # Parameter commands
    def cmd_set_stretch(self):
        val = self.stretch_var.get()
        response, err = self.controller.set_stretch(self.selected_servo_id, val)
        self.show_command_result(f"STRETCH {self.selected_servo_id} {val}", response, err, self.parameters_output)
    
    def cmd_get_stretch(self):
        response, err = self.controller.get_stretch(self.selected_servo_id)
        self.show_command_result(f"GSTRETCH {self.selected_servo_id}", response, err, self.parameters_output)
    
    def cmd_set_speed(self):
        val = self.speed_var.get()
        response, err = self.controller.set_speed(self.selected_servo_id, val)
        self.show_command_result(f"SPEED {self.selected_servo_id} {val}", response, err, self.parameters_output)
    
    def cmd_get_speed(self):
        response, err = self.controller.get_speed(self.selected_servo_id)
        self.show_command_result(f"GSPEED {self.selected_servo_id}", response, err, self.parameters_output)
    
    def cmd_set_current(self):
        val = self.current_var.get()
        response, err = self.controller.set_current(self.selected_servo_id, val)
        self.show_command_result(f"CURRENT {self.selected_servo_id} {val}", response, err, self.parameters_output)
    
    def cmd_get_current(self):
        response, err = self.controller.get_current(self.selected_servo_id)
        self.show_command_result(f"GCURRENT {self.selected_servo_id}", response, err, self.parameters_output)
    
    def cmd_set_temp(self):
        val = self.temp_var.get()
        response, err = self.controller.set_temp(self.selected_servo_id, val)
        self.show_command_result(f"TEMP {self.selected_servo_id} {val}", response, err, self.parameters_output)
    
    def cmd_get_temp(self):
        response, err = self.controller.get_temp(self.selected_servo_id)
        self.show_command_result(f"GTEMP {self.selected_servo_id}", response, err, self.parameters_output)
    
    # EEPROM commands
    def cmd_readeprom(self):
        response, err = self.controller.read_eeprom(self.selected_servo_id)
        self.show_command_result(f"READEPROM {self.selected_servo_id}", response, err, self.eeprom_output)
    
    def cmd_writeeprom(self):
        response, err = self.controller.write_eeprom(self.selected_servo_id)
        self.show_command_result(f"WRITEEPROM {self.selected_servo_id}", response, err, self.eeprom_output)
    
    def cmd_dump(self):
        response, err = self.controller.dump_eeprom()
        self.show_command_result("DUMP", response, err, self.eeprom_output)
    
    def cmd_ehelp(self):
        response, err = self.controller.ehelp()
        self.show_command_result("EHELP", response, err, self.eeprom_output)
    
    def cmd_eget(self):
        field = self.eeprom_field_var.get()
        response, err = self.controller.eget(field)
        self.show_command_result(f"EGET {field}", response, err, self.eeprom_output)
    
    def cmd_eset(self):
        field = self.eeprom_field_var.get()
        value = self.eeprom_value_var.get()
        if not value:
            messagebox.showerror("Error", "Please enter a value")
            return
        response, err = self.controller.eset(field, value)
        self.show_command_result(f"ESET {field} {value}", response, err, self.eeprom_output)
    
    # Terminal commands
    def cmd_terminal_send(self):
        cmd = self.terminal_input.get().strip()
        if not cmd:
            return
        
        if not self.controller.is_connected:
            messagebox.showerror("Error", "Not connected")
            return
        
        response, err = self.controller.send_and_receive(cmd)
        self.show_command_result(f">> {cmd}", response, err, self.terminal_output)
        
        self.terminal_input.delete(0, tk.END)
    
    def clear_terminal_log(self):
        """Clear the terminal log"""
        self.terminal_output.config(state=tk.NORMAL)
        self.terminal_output.delete(1.0, tk.END)
        self.terminal_output.config(state=tk.DISABLED)
        self.log_messages.clear()


def main():
    """Main entry point"""
    root = tk.Tk()
    app = ServoControllerGUI(root)
    
    def on_closing():
        if app.controller.is_connected:
            app.controller.disconnect()
        root.destroy()
    
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()


if __name__ == '__main__':
    main()
