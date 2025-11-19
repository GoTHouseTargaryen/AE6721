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
            'orbit': ['altitude', 'velocity', 'latitude', 'longitude', 'eclipse', 'eclipse_fraction', 'beta_angle', 'sun_angle'],
            'eps': ['mode', 'faulted', 'solar_array_a_deployed', 'solar_array_b_deployed', 'battery_a_voltage', 'battery_a_soc', 
                    'battery_b_voltage', 'battery_b_soc', 'cross_charging_enabled', 'bus_voltage', 'bus_current'],
            'adcs': ['mode', 'enabled', 'faulted', 'voltage', 'current', 'roll', 'pitch', 'yaw', 'roll_rate', 'pitch_rate', 'yaw_rate', 'rws_rpm_x', 'rws_rpm_y', 'rws_rpm_z'],
            'comm': ['mode', 'faulted', 'voltage', 'current', 'tx_power', 'frequency', 'data_rate', 'temp', 'packets_tx', 'packets_rx', 'tx_time_remaining'],
            'tcs': ['faulted', 'voltage', 'current', 'temp_avi', 'temp_eps', 'heater_avi_on', 'heater_eps_on', 'heater_prop_on', 'radiator_deployed'],
            'prop': ['standby_mode', 'system_enabled', 'faulted', 'catalyst_heater_on', 'catalyst_temp', 'catalyst_ready', 'voltage_5v', 'current_5v', 'voltage_vbat', 'current_vbat', 'propellant_mass', 'propellant_type', 'tank_pressure', 'tank_temp', 'valve_open', 'total_delta_v'],
            'avi': ['cpu_load', 'cpu_temp', 'memory_usage', 'uptime', 'boot_count', 'watchdog_ok'],
            'cdh': ['faulted', 'voltage', 'current', 'cmd_count', 'tlm_count', 'error_count', 'storage_used', 'storage_total'],
            'ttc': ['faulted', 'voltage', 'current', 'link_status', 'signal_strength', 'uplink_rate', 'downlink_rate', 'pass_elevation', 'doppler_shift'],
            'star_tracker': ['enabled', 'faulted', 'voltage', 'current', 'init_time_remaining', 'num_stars_tracked', 'attitude_valid', 'attitude_accuracy', 'roll', 'pitch', 'yaw', 'sensor_temp'],
            'gps': ['enabled', 'faulted', 'voltage', 'current', 'num_satellites', 'position_valid', 'position_accuracy', 'velocity_accuracy', 'receiver_temp']
        }
        
        # Map subsystems to their relevant commands
        self.subsystem_commands = {
            'adcs': {
                'system': 'Attitude Determination & Control',
                'commands': [
                    ("🌍 Nadir Pointing", "ADCS_NADIR"),
                    ("☀️ Sun Pointing", "ADCS_SUN_POINT"),
                    ("📍 Inertial Hold", "ADCS_INERTIAL"),
                    ("🔄 Detumble Mode", "ADCS_DETUMBLE"),
                    ("🎯 Go To Attitude (R/P/Y)", "ADCS_GO_TO"),
                    ("🎯 Reset Attitude", "ADCS_RESET"),
                    ("💫 Desaturate Wheels", "ADCS_DESAT"),
                    ("⚡ ADCS On", "ADCS_ON"),
                    ("⏸️ ADCS Off", "ADCS_OFF"),
                    ("🔄 Power Cycle", "ADCS_POWER_CYCLE"),
                ]
            },
            'eps': {
                'system': 'Electrical Power System',
                'commands': [
                    ("📊 Nominal Mode", "EPS_MODE_NOMINAL"),
                    ("🔋 Low Power Mode", "EPS_MODE_LOW_POWER"),
                    ("🔬 Science Mode", "EPS_MODE_SCIENCE"),
                    ("🛡️ Safe Mode", "EPS_MODE_SAFE"),
                    ("🔄 EPS Reset", "EPS_RESET"),
                    ("🌞 Deploy Array A", "EPS_DEPLOY_ARRAY_A"),
                    ("🌛 Undeploy Array A", "EPS_UNDEPLOY_ARRAY_A"),
                    ("🌞 Deploy Array B", "EPS_DEPLOY_ARRAY_B"),
                    ("🌛 Undeploy Array B", "EPS_UNDEPLOY_ARRAY_B"),
                    ("🔗 Cross-Charge Enable", "EPS_CROSS_CHARGE_ENABLE"),
                    ("❌ Cross-Charge Disable", "EPS_CROSS_CHARGE_DISABLE"),
                    ("🔄 Power Cycle", "EPS_POWER_CYCLE"),
                ]
            },
            'comm': {
                'system': 'Communications',
                'commands': [
                    ("📡 TX Mode", "COMM_TX"),
                    ("📨 RX Mode", "COMM_RX"),
                    ("🔄 TX/RX Full Duplex", "COMM_TX_RX"),
                    ("⏸️ Standby", "COMM_STANDBY"),
                    ("⚡ High Power (30dBm)", "COMM_POWER_30"),
                    ("📶 Med Power (27dBm)", "COMM_POWER_27"),
                    ("💤 Low Power (20dBm)", "COMM_POWER_20"),
                    ("🔧 COMM Reset", "COMM_RESET"),
                    ("🔄 Power Cycle", "COMM_POWER_CYCLE"),
                ]
            },
            'prop': {
                'system': 'Propulsion - Heated Catalyst Monoprop',
                'commands': [
                    ("⚡ Standby (5V)", "PROP_STANDBY"),
                    ("🔋 System On (VBat)", "PROP_ON"),
                    ("⏹️ System Off", "PROP_OFF"),
                    ("🔥 Catalyst Heater On", "PROP_HEATER_ON"),
                    ("❄️ Catalyst Heater Off", "PROP_HEATER_OFF"),
                    ("🔓 Open Valve", "PROP_VALVE_OPEN"),
                    ("🔒 Close Valve", "PROP_VALVE_CLOSE"),
                    ("🔄 Power Cycle", "PROP_POWER_CYCLE"),
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
                    ("📡 Deploy Radiator", "TCS_DEPLOY_RADIATOR"),
                    ("📦 Undeploy Radiator", "TCS_UNDEPLOY_RADIATOR"),
                    ("🔄 Power Cycle", "TCS_POWER_CYCLE"),
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
                    ("🔄 Power Cycle", "CDH_POWER_CYCLE"),
                ]
            },
            'ttc': {
                'system': 'TT&C Ground Interface',
                'commands': [
                    ("📊 Link Check", "TTC_STATUS"),
                    ("🔄 Power Cycle", "TTC_POWER_CYCLE"),
                ]
            },
            'star_tracker': {
                'system': 'Star Tracker Navigation',
                'commands': [
                    ("⭐ Enable Tracker", "STAR_TRACKER_ON"),
                    ("⏸️ Disable Tracker", "STAR_TRACKER_OFF"),
                    ("🔄 Power Cycle", "STAR_TRACKER_POWER_CYCLE"),
                ]
            },
            'gps': {
                'system': 'GPS Navigation',
                'commands': [
                    ("📡 Enable GPS", "GPS_ON"),
                    ("⏸️ Disable GPS", "GPS_OFF"),
                    ("🔄 Power Cycle", "GPS_POWER_CYCLE"),
                ]
            },
            'orbit': {
                'system': 'Orbital Parameters',
                'commands': [
                    ("📍 Orbit Status", "ORBIT_STATUS"),
                ]
            },
            'system': {
                'system': 'System Controls',
                'commands': [
                    ("⚠️ Enable Fault Injection", "FAULT_INJECTION_ENABLE"),
                    ("✅ Disable Fault Injection", "FAULT_INJECTION_DISABLE"),
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
        subsystem_order = ['orbit', 'eps', 'adcs', 'star_tracker', 'gps', 'comm', 'tcs', 'prop', 'avi', 'cdh', 'ttc']
        subsystem_names = {
            'orbit': '🛰️ ORBIT',
            'eps': '⚡ EPS - Electrical Power',
            'adcs': '🎯 ADCS - Attitude Control',
            'star_tracker': '⭐ STAR TRACKER - Navigation',
            'gps': '📡 GPS - Navigation',
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
            
            # Create telemetry displays (7 per row)
            telemetry_keys = self.subsystem_telemetry[subsystem]
            self.subsystem_labels[subsystem] = []
            
            for idx, tlm_key in enumerate(telemetry_keys):
                row = idx // 7
                col = idx % 7
                
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
            ("⚡ ADCS\nOn", "ADCS_ON", self.colors['button_nominal']),
            ("🔌 PROP\n5V On", "PROP_STANDBY", self.colors['button_nominal']),
            ("☀️ Sun\nPoint", "ADCS_SUN_POINT", self.colors['button_nominal']),
            ("🌍 Nadir\nPoint", "ADCS_NADIR", self.colors['button_nominal']),
            ("🎯 Go To\nAttitude", "ADCS_GO_TO", self.colors['button_nominal']),
            ("📡 RX\nMode", "COMM_RX", self.colors['button_nominal']),
            ("📡 TX/RX\nMode", "COMM_TX_RX", self.colors['button_nominal']),
            ("🛡️ Safe\nMode", "SAFE_MODE", self.colors['button_response'])
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
            # Special handling for ADCS_GO_TO - prompt for roll, pitch, yaw
            if command == "ADCS_GO_TO":
                self.open_adcs_goto_dialog(self.root)
                return
            
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
        
        # Create scrollable container
        container = tk.Frame(cmd_window, bg=self.colors['bg_main'])
        container.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Create canvas with scrollbar
        canvas = tk.Canvas(container, bg=self.colors['bg_main'], highlightthickness=0)
        scrollbar = tk.Scrollbar(container, orient="vertical", command=canvas.yview)
        cmd_frame = tk.Frame(canvas, bg=self.colors['bg_main'])
        
        cmd_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=cmd_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        # Enable mousewheel scrolling
        def _on_mousewheel(event):
            try:
                canvas.yview_scroll(int(-1*(event.delta/120)), "units")
            except:
                pass  # Canvas no longer exists
        
        canvas.bind_all("<MouseWheel>", _on_mousewheel)
        
        # Unbind mousewheel when window closes
        def _on_close():
            canvas.unbind_all("<MouseWheel>")
            cmd_window.destroy()
        
        cmd_window.protocol("WM_DELETE_WINDOW", _on_close)
        
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
            # Special handling for ADCS_GO_TO - prompt for roll, pitch, yaw
            if command == "ADCS_GO_TO":
                self.open_adcs_goto_dialog(window)
                return
            
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
    
    def open_adcs_goto_dialog(self, parent_window):
        """Open dialog to input roll, pitch, yaw for ADCS_GO_TO command"""
        dialog = tk.Toplevel(self.root)
        dialog.title("🎯 Go To Attitude")
        dialog.geometry("350x280")
        dialog.configure(bg=self.colors['bg_main'])
        dialog.transient(parent_window)
        dialog.grab_set()
        
        # Title
        title_frame = tk.Frame(dialog, bg=self.colors['bg_panel'], pady=8)
        title_frame.pack(fill=tk.X, padx=10, pady=(10, 5))
        tk.Label(title_frame, text="🎯 Enter Target Attitude",
                font=self.font_title,
                bg=self.colors['bg_panel'],
                fg=self.colors['fg_main']).pack()
        
        # Input frame
        input_frame = tk.Frame(dialog, bg=self.colors['bg_main'])
        input_frame.pack(padx=20, pady=5, fill=tk.X)
        
        # Roll input
        tk.Label(input_frame, text="Roll (°):", font=('Segoe UI', 10),
                bg=self.colors['bg_main'], fg=self.colors['fg_main']).grid(row=0, column=0, sticky='e', padx=5, pady=5)
        roll_entry = tk.Entry(input_frame, font=('Segoe UI', 10), width=15)
        roll_entry.grid(row=0, column=1, padx=5, pady=5)
        roll_entry.insert(0, "0")
        
        # Pitch input
        tk.Label(input_frame, text="Pitch (°):", font=('Segoe UI', 10),
                bg=self.colors['bg_main'], fg=self.colors['fg_main']).grid(row=1, column=0, sticky='e', padx=5, pady=5)
        pitch_entry = tk.Entry(input_frame, font=('Segoe UI', 10), width=15)
        pitch_entry.grid(row=1, column=1, padx=5, pady=5)
        pitch_entry.insert(0, "0")
        
        # Yaw input
        tk.Label(input_frame, text="Yaw (°):", font=('Segoe UI', 10),
                bg=self.colors['bg_main'], fg=self.colors['fg_main']).grid(row=2, column=0, sticky='e', padx=5, pady=5)
        yaw_entry = tk.Entry(input_frame, font=('Segoe UI', 10), width=15)
        yaw_entry.grid(row=2, column=1, padx=5, pady=5)
        yaw_entry.insert(0, "0")
        
        # Error label (above buttons)
        error_label = tk.Label(dialog, text="", font=('Segoe UI', 9),
                              bg=self.colors['bg_main'], fg=self.colors['bg_tlm_warning'])
        error_label.pack(pady=(10, 5))
        
        # Button frame
        button_frame = tk.Frame(dialog, bg=self.colors['bg_main'])
        button_frame.pack(pady=(0, 15))
        
        def execute_goto():
            try:
                roll = float(roll_entry.get())
                pitch = float(pitch_entry.get())
                yaw = float(yaw_entry.get())
                
                command = f"ADCS_GO_TO {roll} {pitch} {yaw}"
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
                
                dialog.destroy()
            except ValueError:
                error_label.config(text="⚠️ Invalid input - enter numbers only")
        
        def cancel():
            dialog.destroy()
        
        # Execute button
        exec_btn = tk.Button(button_frame, text="Execute",
                            command=execute_goto,
                            bg=self.colors['button_nominal'],
                            fg='white',
                            font=self.font_button,
                            width=12,
                            relief=tk.RAISED, bd=2,
                            activebackground=self.colors['accent'],
                            cursor='hand2')
        exec_btn.pack(side=tk.LEFT, padx=5)
        
        # Cancel button
        cancel_btn = tk.Button(button_frame, text="Cancel",
                              command=cancel,
                              bg=self.colors['fg_secondary'],
                              fg='white',
                              font=self.font_button,
                              width=12,
                              relief=tk.RAISED, bd=2,
                              cursor='hand2')
        cancel_btn.pack(side=tk.LEFT, padx=5)
        
        # Focus on roll entry
        roll_entry.focus_set()
        roll_entry.select_range(0, tk.END)
    
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
        
        # Create scrollable container
        container = tk.Frame(anomaly_window, bg=self.colors['bg_main'])
        container.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Create canvas with scrollbar
        canvas = tk.Canvas(container, bg=self.colors['bg_main'], highlightthickness=0)
        scrollbar = tk.Scrollbar(container, orient="vertical", command=canvas.yview)
        cmd_frame = tk.Frame(canvas, bg=self.colors['bg_main'])
        
        cmd_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=cmd_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        # Enable mousewheel scrolling
        def _on_mousewheel(event):
            try:
                canvas.yview_scroll(int(-1*(event.delta/120)), "units")
            except:
                pass  # Canvas no longer exists
        
        canvas.bind_all("<MouseWheel>", _on_mousewheel)
        
        # Unbind mousewheel when window closes
        def _on_close():
            canvas.unbind_all("<MouseWheel>")
            anomaly_window.destroy()
        
        anomaly_window.protocol("WM_DELETE_WINDOW", _on_close)
        
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
                    
                    # Check for stale telemetry indicator (value ends with ' S')
                    if isinstance(value, str) and value.endswith(' S'):
                        # Stale telemetry - show last known value with 'S' indicator
                        base_value = value[:-2]  # Remove ' S' suffix
                        label.config(text=f"{display_name}\n{base_value} S", bg='gray', fg='orange')
                    elif value == 'S':
                        # Legacy stale format (for backward compatibility)
                        label.config(text=f"{display_name}\nS", bg='gray', fg='orange')
                    else:
                        # Determine color based on value (traffic light system)
                        color = self.get_tlm_color(subsystem, tlm_key, value, subsystem_data)
                        
                        # Format value
                        if isinstance(value, float):
                            value_str = f"{value:.2f}"
                        elif isinstance(value, int):
                            value_str = str(value)
                        else:
                            value_str = str(value)
                        
                        label.config(text=f"{display_name}\n{value_str}", bg=color, fg='white')
                else:
                    label.config(text=f"{tlm_key}\nN/A", bg='gray', fg='white')
                
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
        
    def get_tlm_color(self, subsystem, key, value, subsystem_data=None):
        """Get traffic light color based on telemetry value"""
        # Green = nominal, Yellow = caution, Red = warning
        
        # Special handling for faulted field
        if key == 'faulted':
            if value is True or value == 'True' or value == True:
                return self.colors['bg_tlm_warning']  # Red when faulted
            else:
                return self.colors['bg_tlm_good']  # Green when not faulted
        
        # Boolean values (except 'faulted' handled above) are always nominal
        if isinstance(value, bool):
            return self.colors['bg_tlm_good']
        
        # Only apply thresholds to numeric values
        if not isinstance(value, (int, float)):
            return self.colors['bg_tlm_good']
        
        # EPS (Electrical Power System)
        if subsystem == 'eps':
            if key == 'battery_voltage' or key == 'battery_a_voltage' or key == 'battery_b_voltage':
                if value < 10.2:
                    return self.colors['bg_tlm_warning']
                elif value < 10.5:
                    return self.colors['bg_tlm_caution']
                else:
                    return self.colors['bg_tlm_good']
                    
            elif key == 'battery_soc' or key == 'battery_a_soc' or key == 'battery_b_soc':
                if value < 15:
                    return self.colors['bg_tlm_warning']
                elif value < 20:
                    return self.colors['bg_tlm_caution']
                else:
                    return self.colors['bg_tlm_good']
                    
            elif key == 'battery_temp' or key == 'battery_a_temp' or key == 'battery_b_temp':
                if value > 50 or value < -15:
                    return self.colors['bg_tlm_warning']
                elif value > 45 or value < -10:
                    return self.colors['bg_tlm_caution']
                else:
                    return self.colors['bg_tlm_good']
                    
            elif key == 'battery_current' or key == 'battery_a_current' or key == 'battery_b_current':
                abs_val = abs(value)
                if abs_val > 3.0:
                    return self.colors['bg_tlm_warning']
                elif abs_val > 2.7:
                    return self.colors['bg_tlm_caution']
                else:
                    return self.colors['bg_tlm_good']
                    
        # ADCS (Attitude Determination & Control)
        elif subsystem == 'adcs':
            if key in ['roll', 'pitch', 'yaw']:
                # Mode-aware color checking for attitudes
                # Get current ADCS mode and determine target attitudes
                mode = subsystem_data.get('mode', 'DETUMBLE') if subsystem_data else 'DETUMBLE'
                
                # Determine target attitude based on current mode
                if mode == 'SUN_POINT':
                    # Sun pointing: -X axis (solar panels) points at sun
                    target_roll = 180.0
                    target_pitch = 0.0
                    target_yaw = 0.0
                elif mode == 'NADIR':
                    # Nadir pointing: +X axis points at Earth
                    target_roll = 0.0
                    target_pitch = -90.0
                    target_yaw = 0.0
                elif mode == 'FINE_HOLD':
                    # Fine hold: use commanded target attitudes
                    target_roll = subsystem_data.get('target_roll', 0.0) if subsystem_data else 0.0
                    target_pitch = subsystem_data.get('target_pitch', 0.0) if subsystem_data else 0.0
                    target_yaw = subsystem_data.get('target_yaw', 0.0) if subsystem_data else 0.0
                else:
                    # DETUMBLE, INERTIAL, or other modes: target is 0,0,0
                    target_roll = 0.0
                    target_pitch = 0.0
                    target_yaw = 0.0
                
                # Select appropriate target for this axis
                if key == 'roll':
                    target = target_roll
                elif key == 'pitch':
                    target = target_pitch
                else:  # yaw
                    target = target_yaw
                
                # Calculate error with angle wrapping (-180 to +180)
                error = (value - target + 180) % 360 - 180
                abs_error = abs(error)
                
                # Apply thresholds to error magnitude
                if abs_error > 10:
                    return self.colors['bg_tlm_warning']
                elif abs_error > 5:
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
                    
            elif key in ['rws_rpm_x', 'rws_rpm_y', 'rws_rpm_z']:
                abs_val = abs(value)
                if abs_val > 5500:
                    return self.colors['bg_tlm_warning']
                elif abs_val > 4500:
                    return self.colors['bg_tlm_caution']
                else:
                    return self.colors['bg_tlm_good']
                    
        # TCS (Thermal Control)
        elif subsystem == 'tcs':
            if key == 'temp_avi':
                if value > 55 or value < -5:
                    return self.colors['bg_tlm_warning']
                elif value > 50 or value < 0:
                    return self.colors['bg_tlm_caution']
                else:
                    return self.colors['bg_tlm_good']
                    
            elif key == 'temp_eps':
                if value > 50 or value < -15:
                    return self.colors['bg_tlm_warning']
                elif value > 45 or value < -10:
                    return self.colors['bg_tlm_caution']
                else:
                    return self.colors['bg_tlm_good']
                    
        # COMM (Communications)
        elif subsystem == 'comm':
            if key == 'temp':
                if value > 65:
                    return self.colors['bg_tlm_warning']
                elif value > 60:
                    return self.colors['bg_tlm_caution']
                else:
                    return self.colors['bg_tlm_good']
                    
        # PROP (Propulsion)
        elif subsystem == 'prop':
            if key == 'propellant_mass':
                if value < 0.01:
                    return self.colors['bg_tlm_warning']
                elif value < 0.05:
                    return self.colors['bg_tlm_caution']
                else:
                    return self.colors['bg_tlm_good']
                    
            elif key == 'tank_pressure':
                if value < 50 or value > 250:
                    return self.colors['bg_tlm_warning']
                elif value < 100 or value > 230:
                    return self.colors['bg_tlm_caution']
                else:
                    return self.colors['bg_tlm_good']
                    
            elif key == 'tank_temp':
                if value < -20 or value > 60:
                    return self.colors['bg_tlm_warning']
                elif value < -10 or value > 50:
                    return self.colors['bg_tlm_caution']
                else:
                    return self.colors['bg_tlm_good']
                    
            elif key == 'catalyst_temp':
                # Catalyst optimal temperature is 300-350°C
                if value < 250 or value > 400:
                    return self.colors['bg_tlm_warning']
                elif value < 300 or value > 350:
                    return self.colors['bg_tlm_caution']
                else:
                    return self.colors['bg_tlm_good']
                    
        # AVI (Avionics)
        elif subsystem == 'avi':
            if key in ['cpu_load', 'memory_usage']:
                if value > 95:
                    return self.colors['bg_tlm_warning']
                elif value > 85:
                    return self.colors['bg_tlm_caution']
                else:
                    return self.colors['bg_tlm_good']
                    
            elif key == 'cpu_temp':
                if value > 75:
                    return self.colors['bg_tlm_warning']
                elif value > 65:
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
                    
        # CDH (Command & Data Handling)
        elif subsystem == 'cdh':
            if key == 'storage_used':
                # Note: storage_used is an absolute value, need to calculate percentage
                # This is a simplified check - the actual system calculates percentage
                # For now, check if the value itself seems high (assuming max is around 100)
                if value > 95:
                    return self.colors['bg_tlm_warning']
                elif value > 85:
                    return self.colors['bg_tlm_caution']
                else:
                    return self.colors['bg_tlm_good']
                    
        # TTC (Tracking, Telemetry & Command)
        elif subsystem == 'ttc':
            if key == 'signal_strength':
                if value < -100:
                    return self.colors['bg_tlm_warning']
                elif value < -95:
                    return self.colors['bg_tlm_caution']
                else:
                    return self.colors['bg_tlm_good']
                    
        # Star Tracker (Precision Attitude Sensor)
        elif subsystem == 'star_tracker':
            if key == 'init_time_remaining':
                # Green when initialized (0), yellow when initializing
                if value > 0:
                    return self.colors['bg_tlm_caution']
                else:
                    return self.colors['bg_tlm_good']
                    
            elif key == 'num_stars_tracked':
                # Red if <2, yellow if 2-4, green if 5+
                if value < 2:
                    return self.colors['bg_tlm_warning']
                elif value < 5:
                    return self.colors['bg_tlm_caution']
                else:
                    return self.colors['bg_tlm_good']
                    
            elif key == 'attitude_accuracy':
                # Green if <5 arcsec, yellow if 5-15, red if >15 or 0 (invalid)
                if value == 0 or value > 15:
                    return self.colors['bg_tlm_warning']
                elif value > 5:
                    return self.colors['bg_tlm_caution']
                else:
                    return self.colors['bg_tlm_good']
                    
            elif key in ['roll', 'pitch', 'yaw']:
                # Use same thresholds as ADCS attitude
                abs_val = abs(value)
                if abs_val > 10:
                    return self.colors['bg_tlm_warning']
                elif abs_val > 5:
                    return self.colors['bg_tlm_caution']
                else:
                    return self.colors['bg_tlm_good']
                    
            elif key == 'sensor_temp':
                # Star tracker sensor temperature
                if value > 40 or value < -30:
                    return self.colors['bg_tlm_warning']
                elif value > 35 or value < -20:
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
