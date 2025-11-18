"""
EID GUI - Ground Command Display
Right panel showing TLM value displays organized by subsystem
Modern UI with improved visual design - 6U CubeSat at 520km LEO
"""
import tkinter as tk
from tkinter import ttk, scrolledtext, font
from telemetry_generator import spacecraft
import threading


class EIDGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("EID - Ground Command Display | 6U CubeSat @ 520km LEO")
        self.root.geometry("900x900")
        
        # Modern color scheme
        self.colors = {
            'bg_main': '#1e1e1e',          # Dark background
            'bg_panel': '#2d2d30',         # Panel background
            'bg_tlm_good': '#0e7c0e',      # Green for nominal
            'bg_tlm_caution': '#f1c40f',   # Yellow for caution
            'bg_tlm_warning': '#e74c3c',   # Red for warning
            'fg_main': '#ffffff',          # White text
            'fg_secondary': '#cccccc',     # Light gray text
            'accent': '#007acc',           # Blue accent
            'button_nominal': '#0e7c0e',   # Green buttons
            'button_response': '#f39c12',  # Orange buttons
        }
        
        self.root.configure(bg=self.colors['bg_main'])
        
        # Start the spacecraft simulator if not already started
        if not spacecraft.running:
            spacecraft.start()
        
        # TLM displays organized by subsystem - dict of subsystem -> list of labels
        self.subsystem_labels = {}
        
        # Define which telemetry to show for each subsystem
        self.subsystem_telemetry = {
            'orbit': ['altitude', 'velocity', 'latitude', 'longitude', 'eclipse', 'sun_angle'],
            'eps': ['mode', 'battery_voltage', 'battery_soc', 'battery_current', 'battery_temp', 
                    'solar_power', 'bus_voltage', 'bus_current'],
            'adcs': ['mode', 'enabled', 'roll', 'pitch', 'yaw', 'roll_rate', 'pitch_rate', 'yaw_rate', 'rws_rpm_x', 'rws_rpm_y', 'rws_rpm_z'],
            'comm': ['mode', 'tx_power', 'frequency', 'data_rate', 'temp', 'packets_tx', 'packets_rx', 'tx_time_remaining'],
            'tcs': ['temp_avi', 'temp_eps', 'temp_external', 'heater_avi_on', 'heater_eps_on', 'radiator_coating'],
            'prop': ['system_enabled', 'propellant_mass', 'tank_pressure', 'tank_temp', 'valve_open', 'total_delta_v'],
            'avi': ['cpu_load', 'cpu_temp', 'memory_usage', 'uptime', 'boot_count', 'watchdog_ok'],
            'cdh': ['cmd_count', 'tlm_count', 'error_count', 'storage_used', 'storage_total'],
            'ttc': ['link_status', 'signal_strength', 'uplink_rate', 'downlink_rate', 'pass_elevation', 'doppler_shift']
        }
        
        # Map subsystems to their relevant commands
        self.subsystem_commands = {
            'adcs': {
                'system': 'Attitude Determination & Control',
                'commands': [
                    (" Nadir Pointing", "ADCS_NADIR"),
                    ("☀️ Sun Pointing", "ADCS_SUN_POINT"),
                    ("📍 Inertial Hold", "ADCS_INERTIAL"),
                    ("🔄 Detumble Mode", "ADCS_DETUMBLE"),
                    ("🎯 Reset Attitude", "ADCS_RESET"),
                    ("⚡ ADCS On", "ADCS_ON"),
                    ("⏸️ ADCS Off", "ADCS_OFF"),
                ]
            },
            'eps': {
                'system': 'Electrical Power System',
                'commands': [
                    ("📊 Nominal Mode", "EPS_MODE_NOMINAL"),
                    ("🔋 Low Power Mode", "EPS_MODE_LOW_POWER"),
                    ("🔬 Science Mode", "EPS_MODE_SCIENCE"),
                    ("�️ Safe Mode", "EPS_MODE_SAFE"),
                    ("🔄 EPS Reset", "EPS_RESET"),
                ]
            },
            'comm': {
                'system': 'Communications',
                'commands': [
                    ("📡 TX Mode", "COMM_TX"),
                    ("� RX Mode", "COMM_RX"),
                    ("⏸️ Standby", "COMM_STANDBY"),
                    ("⚡ High Power (30dBm)", "COMM_POWER_30"),
                    ("� Med Power (27dBm)", "COMM_POWER_27"),
                    ("💤 Low Power (20dBm)", "COMM_POWER_20"),
                    ("� COMM Reset", "COMM_RESET"),
                ]
            },
            'prop': {
                'system': 'Propulsion',
                'commands': [
                    ("⚡ Propulsion On", "PROP_ON"),
                    ("⏹️ Propulsion Off", "PROP_OFF"),
                    ("⏸️ Standby", "PROP_STANDBY"),
                    ("🔓 Open Valve", "PROP_VALVE_OPEN"),
                    ("🔒 Close Valve", "PROP_VALVE_CLOSE"),
                ]
            },
            'tcs': {
                'system': 'Thermal Control',
                'commands': [
                    ("🔥 AVI Heater On", "TCS_HEATER_AVI_ON"),
                    ("❄️ AVI Heater Off", "TCS_HEATER_AVI_OFF"),
                    ("🔥 EPS Heater On", "TCS_HEATER_EPS_ON"),
                    ("❄️ EPS Heater Off", "TCS_HEATER_EPS_OFF"),
                    ("🔥 PROP Heater On", "TCS_HEATER_PROP_ON"),
                    ("❄️ PROP Heater Off", "TCS_HEATER_PROP_OFF"),
                ]
            },
            'avi': {
                'system': 'Avionics',
                'commands': [
                    ("� Reboot Computer", "AVI_REBOOT"),
                    ("⚡ Power Cycle", "AVI_POWER_CYCLE"),
                ]
            },
            'cdh': {
                'system': 'Command & Data Handling',
                'commands': [
                    ("� Status Check", "CDH_STATUS"),
                ]
            },
            'ttc': {
                'system': 'TT&C Ground Interface',
                'commands': [
                    ("� Link Check", "TTC_STATUS"),
                ]
            },
            'orbit': {
                'system': 'Orbital Parameters',
                'commands': [
                    ("� Orbit Status", "ORBIT_STATUS"),
                ]
            }
        }
        
        self.create_widgets()
        self.update_data()
        
    def create_widgets(self):
        # Configure custom fonts
        self.font_title = font.Font(family='Segoe UI', size=11, weight='bold')
        self.font_subsystem = font.Font(family='Segoe UI', size=10, weight='bold')
        self.font_value = font.Font(family='Consolas', size=9, weight='bold')
        self.font_label = font.Font(family='Segoe UI', size=8)
        self.font_button = font.Font(family='Segoe UI', size=9, weight='bold')
        
        # Main container with modern styling
        main_frame = tk.Frame(self.root, bg=self.colors['accent'], padx=2, pady=2)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)
        
        inner_frame = tk.Frame(main_frame, bg=self.colors['bg_main'])
        inner_frame.pack(fill=tk.BOTH, expand=True)
        
        # Title label with modern styling
        title_label = tk.Label(inner_frame, text="⚡ EID | 6U CubeSat @ 520km LEO", 
                              font=self.font_title, 
                              bg=self.colors['bg_panel'], 
                              fg=self.colors['fg_main'],
                              pady=8)
        title_label.pack(fill=tk.X, padx=10, pady=(10, 5))
        
        # Create scrollable frame for telemetry subsystems
        canvas_container = tk.Frame(inner_frame, bg=self.colors['bg_main'])
        canvas_container.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Create canvas with scrollbar
        canvas = tk.Canvas(canvas_container, bg=self.colors['bg_main'], highlightthickness=0)
        scrollbar = ttk.Scrollbar(canvas_container, orient="vertical", command=canvas.yview)
        scrollable_frame = tk.Frame(canvas, bg=self.colors['bg_main'])
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Create subsystem sections
        subsystem_order = ['orbit', 'eps', 'adcs', 'comm', 'tcs', 'prop', 'avi', 'cdh', 'ttc']
        subsystem_names = {
            'orbit': '🛰️ ORBIT',
            'eps': '⚡ EPS - Electrical Power',
            'adcs': '🎯 ADCS - Attitude Control',
            'comm': '📡 COMM - Communications',
            'tcs': '🌡️ TCS - Thermal Control',
            'prop': '🚀 PROP - Propulsion',
            'avi': '💻 AVI - Avionics',
            'cdh': '📊 CDH - Command & Data',
            'ttc': '📶 TT&C - Tracking/Telemetry'
        }
        
        for subsystem in subsystem_order:
            if subsystem not in self.subsystem_telemetry:
                continue
                
            # Subsystem header
            header_frame = tk.Frame(scrollable_frame, bg=self.colors['accent'], height=2)
            header_frame.pack(fill=tk.X, pady=(10, 0))
            
            subsystem_label = tk.Label(scrollable_frame, 
                                      text=subsystem_names.get(subsystem, subsystem.upper()),
                                      font=self.font_subsystem,
                                      bg=self.colors['bg_panel'],
                                      fg=self.colors['fg_main'],
                                      anchor='w',
                                      padx=10, pady=5)
            subsystem_label.pack(fill=tk.X, pady=(0, 3))
            
            # Telemetry grid for this subsystem
            tlm_grid = tk.Frame(scrollable_frame, bg=self.colors['bg_main'])
            tlm_grid.pack(fill=tk.X, padx=5, pady=(0, 5))
            
            # Create telemetry displays (4 per row)
            telemetry_keys = self.subsystem_telemetry[subsystem]
            self.subsystem_labels[subsystem] = []
            
            for idx, tlm_key in enumerate(telemetry_keys):
                row = idx // 4
                col = idx % 4
                
                # Create display
                display_frame = tk.Frame(tlm_grid, bg=self.colors['bg_panel'], relief=tk.FLAT, bd=0)
                display_frame.grid(row=row, column=col, padx=3, pady=3, sticky='nsew')
                
                # Configure grid weights for equal distribution
                tlm_grid.columnconfigure(col, weight=1)
                
                tlm_label = tk.Label(display_frame, 
                                   text=f"{tlm_key}\n--",
                                   font=self.font_value,
                                   bg=self.colors['bg_tlm_good'], 
                                   fg='white',
                                   height=3, width=14,
                                   relief=tk.RAISED, bd=2,
                                   cursor='hand2',
                                   wraplength=110,
                                   justify=tk.CENTER)
                tlm_label.pack(fill=tk.BOTH, expand=True, padx=1, pady=1)
                
                # Bind click event
                tlm_label.bind('<Button-1>', 
                             lambda e, sub=subsystem, key=tlm_key: self.open_subsystem_commands(sub, key))
                
                self.subsystem_labels[subsystem].append((tlm_key, tlm_label))
        
        # Note about clicking telemetry
        note_label = tk.Label(inner_frame, 
                             text="💡 C&W Traffic Light System Active | Click any TLM display for subsystem commands",
                             font=self.font_label,
                             bg=self.colors['bg_main'], 
                             fg=self.colors['fg_secondary'],
                             justify=tk.CENTER)
        note_label.pack(pady=5)
        
        # Bottom section - EVR Log and Quick Commands
        bottom_frame = tk.Frame(inner_frame, bg=self.colors['bg_main'])
        bottom_frame.pack(fill=tk.BOTH, expand=False, padx=10, pady=5)
        
        # EVR Log (left side)
        evr_frame = tk.LabelFrame(bottom_frame, text="📋 EVR Log",
                                 font=self.font_title,
                                 bg=self.colors['bg_panel'], 
                                 fg=self.colors['fg_main'],
                                 relief=tk.FLAT, bd=2)
        evr_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        
        self.evr_text = scrolledtext.ScrolledText(
            evr_frame,
            height=8,
            width=50,
            font=('Consolas', 8),
            bg=self.colors['bg_panel'],
            fg=self.colors['fg_main'],
            state=tk.DISABLED
        )
        self.evr_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Quick Commands (right side)
        cmd_frame = tk.LabelFrame(bottom_frame, text="⚙️ Quick Commands",
                                 font=self.font_title,
                                 bg=self.colors['bg_panel'], 
                                 fg=self.colors['fg_main'],
                                 relief=tk.FLAT, bd=2)
        cmd_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=False)
        
        # Create command buttons
        cmd_grid = tk.Frame(cmd_frame, bg=self.colors['bg_panel'])
        cmd_grid.pack(padx=5, pady=5)
        
        commands = [
            ("🌍 Nadir\nPoint", "ADCS_NADIR", self.colors['button_nominal']),
            ("☀️ Sun\nPoint", "ADCS_SUN_POINT", self.colors['button_nominal']),
            ("🔬 Science\nMode", "EPS_MODE_SCIENCE", self.colors['button_nominal']),
            ("🔋 Low Pwr\nMode", "EPS_MODE_LOW_POWER", self.colors['button_nominal']),
            ("📊 Nominal\nMode", "EPS_MODE_NOMINAL", self.colors['button_nominal']),
            ("⚡ ADCS\nOn", "ADCS_ON", self.colors['button_nominal']),
            ("🛡️ Safe\nMode", "SAFE_MODE", self.colors['button_response']),
            ("🔄 System\nReset", "SYS_RESET", self.colors['button_response'])
        ]
        
        for i, (label, cmd, color) in enumerate(commands):
            row = i // 4
            col = i % 4
            btn = tk.Button(cmd_grid, text=label, 
                          command=lambda c=cmd: self.execute_command(c),
                          bg=color, 
                          fg='white',
                          font=self.font_button,
                          width=10, height=2,
                          relief=tk.RAISED,
                          bd=2,
                          activebackground=self.colors['accent'],
                          cursor='hand2')
            btn.grid(row=row, column=col, padx=3, pady=3)
            
    def execute_command(self, command):
        """Execute a command with error handling"""
        try:
            spacecraft.send_command(command)
        
            # Log command to experiment manager if active
            try:
                from experiment_manager import experiment_manager
                if experiment_manager and experiment_manager.is_running:
                    experiment_manager.log_command(command)
            except:
                pass  # Experiment manager not active
        except Exception as e:
            print(f"Error executing command: {e}")
        
    def open_subsystem_commands(self, subsystem, tlm_key):
        """Open subsystem-specific command window"""
        # Get command set for this subsystem
        if subsystem not in self.subsystem_commands:
            self.open_anomaly_window()
            return
            
        command_info = self.subsystem_commands[subsystem]
        system_name = command_info['system']
        commands = command_info['commands']
        
        # Get current telemetry value
        telemetry = spacecraft.get_telemetry()
        subsystem_data = telemetry.get(subsystem, {})
        current_value = subsystem_data.get(tlm_key, "N/A")
        if isinstance(current_value, float):
            value_str = f"{current_value:.2f}"
        else:
            value_str = str(current_value)
        
        # Create subsystem command window
        cmd_window = tk.Toplevel(self.root)
        cmd_window.title(f"⚙️ {system_name}")
        cmd_window.geometry("450x550")
        cmd_window.configure(bg=self.colors['bg_main'])
        
        # Title frame
        title_frame = tk.Frame(cmd_window, bg=self.colors['bg_panel'], pady=10)
        title_frame.pack(fill=tk.X, padx=10, pady=10)
        
        tk.Label(title_frame, text=f"⚙️ {system_name}",
                font=self.font_title, 
                bg=self.colors['bg_panel'],
                fg=self.colors['fg_main']).pack()
        
        tk.Label(title_frame, text=f"{tlm_key.replace('_', ' ').title()}: {value_str}",
                font=('Segoe UI', 10), 
                bg=self.colors['bg_panel'],
                fg=self.colors['fg_secondary']).pack(pady=(5, 0))
        
        # Divider
        divider = tk.Frame(cmd_window, bg=self.colors['accent'], height=2)
        divider.pack(fill=tk.X, padx=10, pady=5)
        
        # Info label
        info_label = tk.Label(cmd_window,
                             text="Select a command to execute:",
                             font=('Segoe UI', 9),
                             bg=self.colors['bg_main'],
                             fg=self.colors['fg_secondary'])
        info_label.pack(pady=5)
        
        # Commands frame
        cmd_frame = tk.Frame(cmd_window, bg=self.colors['bg_main'])
        cmd_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Create buttons for each command
        for label, cmd in commands:
            btn = tk.Button(cmd_frame, text=label,
                          command=lambda c=cmd: self.execute_subsystem_command(c, cmd_window),
                          bg=self.colors['button_nominal'], 
                          fg='white',
                          font=self.font_button, 
                          width=35, height=2,
                          relief=tk.RAISED, bd=2,
                          activebackground=self.colors['accent'],
                          cursor='hand2')
            btn.pack(pady=4)
        
        # Divider before emergency
        divider2 = tk.Frame(cmd_frame, bg=self.colors['fg_secondary'], height=1)
        divider2.pack(fill=tk.X, pady=10)
        
        # Emergency safe mode
        safe_btn = tk.Button(cmd_frame, text="🛡️ SAFE MODE (Emergency)",
                           command=lambda: self.execute_subsystem_command("SAFE_MODE", cmd_window),
                           bg=self.colors['bg_tlm_warning'], 
                           fg='white',
                           font=self.font_button, 
                           width=35, height=2,
                           relief=tk.RAISED, bd=2,
                           activebackground='#c0392b',
                           cursor='hand2')
        safe_btn.pack(pady=4)
            
    def execute_subsystem_command(self, command, window):
        """Execute a subsystem command"""
        try:
            spacecraft.send_command(command)
        
            # Log command to experiment manager if active
            try:
                from experiment_manager import experiment_manager
                if experiment_manager and experiment_manager.is_running:
                    experiment_manager.log_command(command)
                    if not experiment_manager.anomaly_resolved_time:
                        experiment_manager.mark_anomaly_resolved()
            except:
                pass
            
            window.destroy()
        except Exception as e:
            print(f"Error executing command: {e}")
            window.destroy()
    
    def open_anomaly_window(self):
        """Open generic anomaly response command window (fallback)"""
        anomaly_window = tk.Toplevel(self.root)
        anomaly_window.title("🚨 Anomaly Response Commands")
        anomaly_window.geometry("420x380")
        anomaly_window.configure(bg=self.colors['bg_main'])
        
        # Title frame
        title_frame = tk.Frame(anomaly_window, bg=self.colors['bg_panel'], pady=10)
        title_frame.pack(fill=tk.X, padx=10, pady=10)
        
        tk.Label(title_frame, text="🚨 Generic Anomaly Response",
                font=self.font_title, 
                bg=self.colors['bg_panel'],
                fg=self.colors['fg_main']).pack()
        
        # Commands frame
        cmd_frame = tk.Frame(anomaly_window, bg=self.colors['bg_main'])
        cmd_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Generic anomaly response commands
        commands = [
            ("🛡️ Safe Mode", "SAFE_MODE"),
            ("⚡ Power Reset", "PWR_RESET"),
            ("🎯 Attitude Reset", "ATT_RESET"),
            ("🌡️ Thermal Reset", "THERM_RESET"),
            ("📡 Comm Reset", "COMM_RESET"),
            ("🔋 Battery Charge", "BATT_CHARGE"),
            ("⚠️ Emergency Shutdown", "EMERG_SHUTDOWN"),
            ("🔧 Diagnostic Mode", "DIAG_MODE")
        ]
        
        for label, cmd in commands:
            btn = tk.Button(cmd_frame, text=label,
                          command=lambda c=cmd: self.execute_anomaly_response(c, anomaly_window),
                          bg=self.colors['button_response'], 
                          fg='white',
                          font=self.font_button, 
                          width=25, height=2,
                          relief=tk.RAISED, bd=2,
                          activebackground=self.colors['bg_tlm_warning'],
                          cursor='hand2')
            btn.pack(pady=4)
            
    def execute_anomaly_response(self, command, window):
            """Execute an anomaly response command with error handling"""
            try:
                spacecraft.send_command(command)
            
                # Log command to experiment manager if active
                try:
                    from experiment_manager import experiment_manager
                    if experiment_manager and experiment_manager.is_running:
                        experiment_manager.log_command(command)
                        # If this is a resolution command, mark anomaly as resolved
                        if not experiment_manager.anomaly_resolved_time:
                            experiment_manager.mark_anomaly_resolved()
                except:
                    pass  # Experiment manager not active
                
                window.destroy()
            except Exception as e:
                print(f"Error executing anomaly response: {e}")
                window.destroy()
        
    def update_data(self):
        """Update all displays with latest data"""
        telemetry = spacecraft.get_telemetry()
        
        # Update TLM displays organized by subsystem
        for subsystem, label_list in self.subsystem_labels.items():
            subsystem_data = telemetry.get(subsystem, {})
            
            for tlm_key, label in label_list:
                if tlm_key in subsystem_data:
                    value = subsystem_data[tlm_key]
                    display_name = tlm_key.replace('_', ' ').title()
                    
                    # Determine color based on value (traffic light system)
                    color = self.get_tlm_color(subsystem, tlm_key, value)
                    
                    # Format value
                    if isinstance(value, float):
                        value_str = f"{value:.2f}"
                    elif isinstance(value, int):
                        value_str = str(value)
                    else:
                        value_str = str(value)
                    
                    label.config(text=f"{display_name}\n{value_str}", bg=color)
                else:
                    label.config(text=f"{tlm_key}\nN/A", bg='gray')
                
        # Update EVR log
        evr_log = spacecraft.get_evr_log()
        self.evr_text.config(state=tk.NORMAL)
        self.evr_text.delete(1.0, tk.END)
        
        # Show last 15 entries
        for entry in evr_log[-15:]:
            self.evr_text.insert(tk.END, entry + "\n")
            
        self.evr_text.see(tk.END)
        self.evr_text.config(state=tk.DISABLED)
        
        # Schedule next update (1 Hz)
        self.root.after(1000, self.update_data)
        
    def get_tlm_color(self, subsystem, key, value):
        """Get traffic light color based on telemetry value"""
        # Green = nominal, Yellow = caution, Red = warning
        
        # Only apply thresholds to numeric values
        if not isinstance(value, (int, float)):
            return self.colors['bg_tlm_good']
        
        # EPS (Electrical Power System)
        if subsystem == 'eps':
            if key == 'battery_voltage':
                if value < 10.0:
                    return self.colors['bg_tlm_warning']
                elif value < 10.8:
                    return self.colors['bg_tlm_caution']
                else:
                    return self.colors['bg_tlm_good']
                    
            elif key == 'battery_soc':
                if value < 20:
                    return self.colors['bg_tlm_warning']
                elif value < 40:
                    return self.colors['bg_tlm_caution']
                else:
                    return self.colors['bg_tlm_good']
                    
            elif key == 'battery_temp':
                if value > 45 or value < -10:
                    return self.colors['bg_tlm_warning']
                elif value > 40 or value < 0:
                    return self.colors['bg_tlm_caution']
                else:
                    return self.colors['bg_tlm_good']
                    
        # ADCS (Attitude Determination & Control)
        elif subsystem == 'adcs':
            if key in ['roll', 'pitch', 'yaw']:
                abs_val = abs(value)
                if abs_val > 10:
                    return self.colors['bg_tlm_warning']
                elif abs_val > 5:
                    return self.colors['bg_tlm_caution']
                else:
                    return self.colors['bg_tlm_good']
                    
            elif key in ['roll_rate', 'pitch_rate', 'yaw_rate']:
                abs_val = abs(value)
                if abs_val > 0.5:
                    return self.colors['bg_tlm_warning']
                elif abs_val > 0.1:
                    return self.colors['bg_tlm_caution']
                else:
                    return self.colors['bg_tlm_good']
                    
        # TCS (Thermal Control)
        elif subsystem == 'tcs':
            if key.startswith('temp_panel'):
                # External panel temps are cold
                if value > -10 or value < -100:
                    return self.colors['bg_tlm_warning']
                elif value > -30 or value < -95:
                    return self.colors['bg_tlm_caution']
                else:
                    return self.colors['bg_tlm_good']
            else:
                # Internal component temps
                if value > 50 or value < 0:
                    return self.colors['bg_tlm_warning']
                elif value > 45 or value < 5:
                    return self.colors['bg_tlm_caution']
                else:
                    return self.colors['bg_tlm_good']
                    
        # COMM (Communications)
        elif subsystem == 'comm':
            if key == 'rssi':
                if value < -100:
                    return self.colors['bg_tlm_warning']
                elif value < -95:
                    return self.colors['bg_tlm_caution']
                else:
                    return self.colors['bg_tlm_good']
                    
            elif key == 'pa_temp':
                if value > 60:
                    return self.colors['bg_tlm_warning']
                elif value > 55:
                    return self.colors['bg_tlm_caution']
                else:
                    return self.colors['bg_tlm_good']
                    
            elif key == 'link_margin':
                if value < 5:
                    return self.colors['bg_tlm_warning']
                elif value < 10:
                    return self.colors['bg_tlm_caution']
                else:
                    return self.colors['bg_tlm_good']
                    
        # PROP (Propulsion)
        elif subsystem == 'prop':
            if key == 'fuel_mass':
                if value < 0.020:
                    return self.colors['bg_tlm_warning']
                elif value < 0.050:
                    return self.colors['bg_tlm_caution']
                else:
                    return self.colors['bg_tlm_good']
                    
            elif key == 'fuel_pressure':
                if value < 100 or value > 250:
                    return self.colors['bg_tlm_warning']
                elif value < 150 or value > 220:
                    return self.colors['bg_tlm_caution']
                else:
                    return self.colors['bg_tlm_good']
                    
        # AVI (Avionics)
        elif subsystem == 'avi':
            if key in ['cpu_usage', 'memory_usage', 'storage_used']:
                if value > 95:
                    return self.colors['bg_tlm_warning']
                elif value > 85:
                    return self.colors['bg_tlm_caution']
                else:
                    return self.colors['bg_tlm_good']
                    
            elif key == 'cpu_temp':
                if value > 60:
                    return self.colors['bg_tlm_warning']
                elif value > 50:
                    return self.colors['bg_tlm_caution']
                else:
                    return self.colors['bg_tlm_good']
                    
        # ORBIT
        elif subsystem == 'orbit':
            if key == 'altitude':
                # 520km nominal
                if value < 510 or value > 530:
                    return self.colors['bg_tlm_caution']
                else:
                    return self.colors['bg_tlm_good']
                    
        # Default to green for everything else
        return self.colors['bg_tlm_good']
            
    def on_closing(self):
        """Clean shutdown"""
        self.root.destroy()


def main():
    root = tk.Tk()
    app = EIDGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()
