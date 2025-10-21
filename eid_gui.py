"""
EID GUI - Ground Command Display
Right panel showing TLM value displays, EVR log, and command buttons
Modern UI with improved visual design
"""
import tkinter as tk
from tkinter import ttk, scrolledtext, font
from telemetry_generator import spacecraft
import threading


class EIDGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("EID - Ground Command Display | 6U CubeSat")
        self.root.geometry("750x750")
        
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
        
        # TLM displays - we'll track which telemetry values to show
        self.tlm_labels = []
        self.selected_tlm_keys = [
            'altitude', 'velocity', 'latitude', 'longitude',
            'battery_voltage', 'battery_current', 'solar_array_current',
            'temperature_internal', 'temperature_external', 'pressure',
            'attitude_roll', 'attitude_pitch', 'attitude_yaw',
            'communication_signal', 'pressure'
        ]
        
        self.create_widgets()
        self.update_data()
        
    def create_widgets(self):
        # Configure custom fonts
        self.font_title = font.Font(family='Segoe UI', size=11, weight='bold')
        self.font_value = font.Font(family='Consolas', size=13, weight='bold')
        self.font_label = font.Font(family='Segoe UI', size=8)
        self.font_button = font.Font(family='Segoe UI', size=9, weight='bold')
        
        # Main container with modern styling
        main_frame = tk.Frame(self.root, bg=self.colors['accent'], padx=2, pady=2)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)
        
        inner_frame = tk.Frame(main_frame, bg=self.colors['bg_main'])
        inner_frame.pack(fill=tk.BOTH, expand=True)
        
        # Title label with modern styling
        title_label = tk.Label(inner_frame, text="⚡ EID | Ground Command Display", 
                              font=self.font_title, 
                              bg=self.colors['bg_panel'], 
                              fg=self.colors['fg_main'],
                              pady=8)
        title_label.pack(fill=tk.X, padx=10, pady=(10, 5))
        
        # TLM Value Display grid (3x5 grid of displays)
        tlm_grid_frame = tk.Frame(inner_frame, bg=self.colors['bg_main'])
        tlm_grid_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Create 3 rows of 5 TLM displays each
        for row in range(3):
            row_frame = tk.Frame(tlm_grid_frame, bg=self.colors['bg_main'])
            row_frame.pack(fill=tk.BOTH, expand=True, pady=3)
            
            for col in range(5):
                tlm_display = self.create_tlm_display(row_frame)
                tlm_display.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=3)
                
        # Note about C&W and TLM Display
        note_label = tk.Label(inner_frame, 
                             text="💡 C&W Traffic Light System Active | Click any TLM display for detailed response options",
                             font=self.font_label,
                             bg=self.colors['bg_main'], 
                             fg=self.colors['fg_secondary'],
                             justify=tk.CENTER)
        note_label.pack(pady=5)
        
        # Bottom section
        bottom_frame = tk.Frame(inner_frame, bg=self.colors['bg_main'])
        bottom_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # EVR Log (left side) with modern styling
        evr_frame = tk.LabelFrame(bottom_frame, text="📋 EVR Log",
                                 font=self.font_title,
                                 bg=self.colors['bg_panel'], 
                                 fg=self.colors['fg_main'],
                                 relief=tk.FLAT, bd=2)
        evr_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        
        self.evr_text = scrolledtext.ScrolledText(
            evr_frame,
            height=8,
            width=40,
            font=('Consolas', 8),
            bg=self.colors['bg_panel'],
            fg=self.colors['fg_main'],
            state=tk.DISABLED
        )
        self.evr_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Command buttons (right side) with modern styling
        cmd_frame = tk.LabelFrame(bottom_frame, text="⚙️ Quick Commands",
                                 font=self.font_title,
                                 bg=self.colors['bg_panel'], 
                                 fg=self.colors['fg_main'],
                                 relief=tk.FLAT, bd=2)
        cmd_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=False)
        
        # Create 2x4 grid of command buttons with nominal operations
        cmd_grid = tk.Frame(cmd_frame, bg=self.colors['bg_panel'])
        cmd_grid.pack(padx=5, pady=5)
        
        # Updated commands for nominal operations
        commands = [
            ("🎯 Nadir\nPoint", "CMD1", self.colors['button_nominal']),
            ("☀️ Sun\nPoint", "CMD2", self.colors['button_nominal']),
            ("🔬 Science\nMode", "CMD3", self.colors['button_nominal']),
            ("🔋 Low Pwr\nMode", "CMD4", self.colors['button_nominal']),
            ("✓ Clear\nCaut", "CMD5", self.colors['button_response']),
            ("💊 Health\nCheck", "CMD6", self.colors['button_nominal']),
            ("📊 Set\nDatarate", "CMD7", self.colors['button_nominal']),
            ("⏱️ WDT\nReset", "CMD8", self.colors['button_response'])
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
            
    def create_tlm_display(self, parent):
        """Create a single TLM value display box with modern styling"""
        display_frame = tk.Frame(parent, bg=self.colors['bg_panel'], relief=tk.FLAT, bd=0)
        
        # Create label that will show telemetry name and value
        tlm_label = tk.Label(display_frame, text="TLM Value\nDisplay",
                           font=('Consolas', 9, 'bold'),
                           bg=self.colors['bg_tlm_good'], 
                           fg='white',
                           height=4, width=14,
                           relief=tk.RAISED, bd=3,
                           cursor='hand2',
                           wraplength=110,
                           justify=tk.CENTER)
        tlm_label.pack(fill=tk.BOTH, expand=True, padx=1, pady=1)
        
        # Store reference to label
        self.tlm_labels.append(tlm_label)
        
        # Bind click event
        tlm_label.bind('<Button-1>', lambda e: self.open_anomaly_window())
        
        return display_frame
        
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
        
    def open_anomaly_window(self):
        """Open anomaly response command window with modern styling"""
        anomaly_window = tk.Toplevel(self.root)
        anomaly_window.title("🚨 Anomaly Response Commands")
        anomaly_window.geometry("420x380")
        anomaly_window.configure(bg=self.colors['bg_main'])
        
        # Title frame
        title_frame = tk.Frame(anomaly_window, bg=self.colors['bg_panel'], pady=10)
        title_frame.pack(fill=tk.X, padx=10, pady=10)
        
        tk.Label(title_frame, text="🚨 Anomaly Response Commands",
                font=self.font_title, 
                bg=self.colors['bg_panel'],
                fg=self.colors['fg_main']).pack()
        
        # Commands frame
        cmd_frame = tk.Frame(anomaly_window, bg=self.colors['bg_main'])
        cmd_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Anomaly response commands
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
        
        # Update TLM displays with traffic light system
        for i, label in enumerate(self.tlm_labels):
            if i < len(self.selected_tlm_keys):
                key = self.selected_tlm_keys[i]
                if key in telemetry:
                    value = telemetry[key]
                    display_name = key.replace('_', ' ').title()
                    
                    # Determine color based on value (traffic light system)
                    color = self.get_tlm_color(key, value)
                    
                    # Format value
                    if isinstance(value, float):
                        value_str = f"{value:.2f}"
                    else:
                        value_str = str(value)
                    
                    label.config(text=f"{display_name}\n{value_str}", bg=color)
                else:
                    label.config(text="TLM Value\nN/A", bg='gray')
            else:
                label.config(text="TLM Value\nDisplay", bg='lightgray')
                
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
        
    def get_tlm_color(self, key, value):
        """Get traffic light color based on telemetry value - uses spacecraft thresholds"""
        # Green = nominal, Yellow = caution, Red = warning
        # Access spacecraft's configurable thresholds
        thresholds = spacecraft.thresholds
        
        if key == 'battery_voltage':
            # 12V system thresholds
            if value < thresholds['battery_voltage_min']:
                return self.colors['bg_tlm_warning']  # Red
            elif value < thresholds['battery_voltage_caution']:
                return self.colors['bg_tlm_caution']  # Yellow
            else:
                return self.colors['bg_tlm_good']  # Green
                
        elif key in ['attitude_roll', 'attitude_pitch', 'attitude_yaw']:
            abs_val = abs(value)
            if abs_val > thresholds['attitude_warning']:
                return self.colors['bg_tlm_warning']  # Red
            elif abs_val > thresholds['attitude_caution']:
                return self.colors['bg_tlm_caution']  # Yellow
            else:
                return self.colors['bg_tlm_good']  # Green
                
        elif key == 'temperature_internal':
            # Celsius thresholds
            if value > thresholds['temperature_max'] or value < thresholds['temperature_min']:
                return self.colors['bg_tlm_warning']  # Red
            elif value > 35.0 or value < 5.0:
                return self.colors['bg_tlm_caution']  # Yellow
            else:
                return self.colors['bg_tlm_good']  # Green
                
        elif key == 'temperature_external':
            # External temps are cold in space (Celsius)
            if value > -10.0 or value < -90.0:
                return self.colors['bg_tlm_warning']  # Red
            elif value > -20.0 or value < -85.0:
                return self.colors['bg_tlm_caution']  # Yellow
            else:
                return self.colors['bg_tlm_good']  # Green
                
        elif key == 'communication_signal':
            if value < 70.0:
                return self.colors['bg_tlm_warning']  # Red
            elif value < 85.0:
                return self.colors['bg_tlm_caution']  # Yellow
            else:
                return self.colors['bg_tlm_good']  # Green
                
        elif key == 'altitude':
            # ISS orbit: 408km nominal
            if value < 400 or value > 416:
                return self.colors['bg_tlm_caution']  # Yellow
            else:
                return self.colors['bg_tlm_good']  # Green
                
        elif key == 'battery_current':
            # High discharge rate warning
            if value < -1.5:  # Discharging fast
                return self.colors['bg_tlm_warning']  # Red
            elif value < -1.0:
                return self.colors['bg_tlm_caution']  # Yellow
            else:
                return self.colors['bg_tlm_good']  # Green
                
        else:
            return self.colors['bg_tlm_good']  # Default to green
            
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
