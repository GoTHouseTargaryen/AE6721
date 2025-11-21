"""
Experiment Control GUI
Interface for managing experimental scenarios and viewing real-time metrics
"""
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import experiment_manager as exp_mod
from telemetry_generator import spacecraft
import threading
import time


class ExperimentGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Experiment Control Panel")
        self.root.geometry("600x700")
        self.root.configure(bg='#2d2d30')
        
        # Ensure spacecraft is started
        if not spacecraft.running:
            spacecraft.start()
        
        # Initialize experiment manager and hold direct reference
        self.exp_mgr = exp_mod.initialize_experiment_manager(spacecraft)
        
        # Define nominal procedure (can be customized)
        self.nominal_procedure = []
        
        # Set up window close handler
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        self.create_widgets()
        self.update_status()
        
    def create_widgets(self):
        # Title
        title = tk.Label(self.root, text="🔬 Experiment Control System", 
                        font=('Segoe UI', 14, 'bold'),
                        bg='#2d2d30', fg='white', pady=10)
        title.pack()
        
        # Researcher Input Frame
        input_frame = tk.LabelFrame(self.root, text="Researcher Inputs (Required)",
                                   font=('Segoe UI', 11, 'bold'),
                                   bg='#3c3c3c', fg='white', bd=2)
        input_frame.pack(fill=tk.X, padx=15, pady=10)
        
        # Subject Number
        tk.Label(input_frame, text="Subject Number:", 
                bg='#3c3c3c', fg='white', font=('Segoe UI', 10)).grid(row=0, column=0, padx=10, pady=5, sticky=tk.W)
        self.subject_number_var = tk.StringVar()
        subject_entry = tk.Entry(input_frame, textvariable=self.subject_number_var, 
                                font=('Segoe UI', 10), width=20, bg='#1e1e1e', fg='white', insertbackground='white')
        subject_entry.grid(row=0, column=1, padx=10, pady=5, sticky=tk.W)
        
        # Run Type
        tk.Label(input_frame, text="Run Type:", 
                bg='#3c3c3c', fg='white', font=('Segoe UI', 10)).grid(row=1, column=0, padx=10, pady=5, sticky=tk.W)
        self.run_type_var = tk.StringVar(value="Control Nominal")
        run_type_dropdown = ttk.Combobox(input_frame, textvariable=self.run_type_var, 
                                        font=('Segoe UI', 9), width=25, state='readonly')
        run_type_dropdown['values'] = ('Control Nominal', 'EID Nominal', 'Control Off-Nominal', 'EID Off-Nominal')
        run_type_dropdown.grid(row=1, column=1, padx=10, pady=5, sticky=tk.W)
        
        # Scenario Selection Frame
        scenario_frame = tk.LabelFrame(self.root, text="Scenario Configuration",
                                      font=('Segoe UI', 11, 'bold'),
                                      bg='#3c3c3c', fg='white', bd=2)
        scenario_frame.pack(fill=tk.X, padx=15, pady=10)
        
        # Scenario type info
        tk.Label(scenario_frame, text="Scenario Info:", 
                bg='#3c3c3c', fg='white', font=('Segoe UI', 10)).grid(row=0, column=0, padx=10, pady=5, sticky=tk.W)
        
        info_text = tk.Label(scenario_frame, 
                           text="• Nominal scenarios: 5 minutes, auto-launches appropriate GUI\n"
                                "• Off-Nominal scenarios: 10 minutes, 2 compound faults (3-5 min), auto-launches appropriate GUI\n"
                                "• Faults are auto-injected - no manual researcher intervention needed",
                           bg='#3c3c3c', fg='#aaaaaa', font=('Segoe UI', 9), justify=tk.LEFT)
        info_text.grid(row=0, column=1, columnspan=3, padx=5, pady=5, sticky=tk.W)
        
        # Nominal Procedure Input
        tk.Label(scenario_frame, text="Nominal Procedure (editable):", 
                bg='#3c3c3c', fg='white', font=('Segoe UI', 10)).grid(row=1, column=0, padx=10, pady=5, sticky=tk.NW)
        
        procedure_frame = tk.Frame(scenario_frame, bg='#3c3c3c')
        procedure_frame.grid(row=1, column=1, columnspan=2, padx=5, pady=5, sticky=tk.EW)
        
        self.procedure_text = scrolledtext.ScrolledText(procedure_frame, height=5, width=40,
                                                        font=('Consolas', 9),
                                                        bg='#1e1e1e', fg='white')
        self.procedure_text.pack()
        # Default nominal procedure commands - can be edited before starting experiment
        default_procedure = """# Full Nominal Operations Procedure
COMM_TX_RX
ADCS_ON
EPS_DEPLOY_ARRAY_A
EPS_DEPLOY_ARRAY_B
TCS_DEPLOY_RADIATOR
GPS_ON
STAR_TRACKER_ON
ADCS_SUN_POINT
PROP_STANDBY
PROP_ON
PROP_HEATER_ON
ADCS_DESAT
ADCS_GO_TO 50 -50 100
PROP_VALVE_OPEN
PROP_VALVE_CLOSE
ADCS_SUN_POINT
PROP_VALVE_CLOSE
PROP_HEATER_OFF
PROP_OFF"""
        self.procedure_text.insert('1.0', default_procedure)
        
        # Start/Stop Buttons
        button_frame = tk.Frame(scenario_frame, bg='#3c3c3c')
        button_frame.grid(row=2, column=0, columnspan=3, pady=10)
        
        self.start_btn = tk.Button(button_frame, text="▶ Start Experiment",
                                   command=self.start_experiment,
                                   bg='#0e7c0e', fg='white', font=('Segoe UI', 10, 'bold'),
                                   width=15, height=2, cursor='hand2')
        self.start_btn.grid(row=0, column=0, padx=5)
        
        self.stop_btn = tk.Button(button_frame, text="⏹ Stop Experiment",
                                  command=self.stop_scenario,
                                  bg='#e74c3c', fg='white', font=('Segoe UI', 10, 'bold'),
                                  width=15, height=2, cursor='hand2', state=tk.DISABLED)
        self.stop_btn.grid(row=0, column=1, padx=5)
        
        # Procedure Timing Frame (For Researcher Use)
        action_frame = tk.LabelFrame(self.root, text="Procedure Timing (Optional)",
                                     font=('Segoe UI', 11, 'bold'),
                                     bg='#3c3c3c', fg='white', bd=2)
        action_frame.pack(fill=tk.X, padx=15, pady=10)
        
        btn_frame = tk.Frame(action_frame, bg='#3c3c3c')
        btn_frame.pack(pady=10)
        
        tk.Button(btn_frame, text="⏱ Start Procedure",
                 command=self.start_procedure,
                 bg='#007acc', fg='white', font=('Segoe UI', 9, 'bold'),
                 width=18, height=2).grid(row=0, column=0, padx=5, pady=5)
        
        tk.Button(btn_frame, text="⏱ End Procedure",
                 command=self.end_procedure,
                 bg='#9b59b6', fg='white', font=('Segoe UI', 9, 'bold'),
                 width=18, height=2).grid(row=0, column=1, padx=5, pady=5)
        
        # Info label for procedure instructions
        procedure_info = tk.Label(action_frame, 
                                 text="Use these buttons to time when subject starts/completes the nominal procedure.\n"
                                      "Procedure is auto-loaded at experiment start. Anomalies are auto-detected.",
                                 font=('Segoe UI', 8, 'italic'),
                                 bg='#3c3c3c', fg='#cccccc', justify=tk.LEFT)
        procedure_info.pack(pady=5)
        
        # Status Display
        status_frame = tk.LabelFrame(self.root, text="Experiment Status",
                                     font=('Segoe UI', 11, 'bold'),
                                     bg='#3c3c3c', fg='white', bd=2)
        status_frame.pack(fill=tk.BOTH, expand=True, padx=15, pady=10)
        
        self.status_text = scrolledtext.ScrolledText(status_frame, height=15, width=60,
                                                     font=('Consolas', 9),
                                                     bg='#1e1e1e', fg='#00ff00',
                                                     state=tk.DISABLED)
        self.status_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Info Label
        info = tk.Label(self.root, 
                       text="📊 Experiment data is logged to JSON files in the project directory",
                       font=('Segoe UI', 8, 'italic'),
                       bg='#2d2d30', fg='#cccccc')
        info.pack(pady=5)
        
    def start_experiment(self):
        """Start the experiment and auto-launch appropriate GUI"""
        # Validate researcher inputs
        subject_number = self.subject_number_var.get().strip()
        run_type = self.run_type_var.get()
        
        if not subject_number:
            messagebox.showwarning("Missing Input", "Please enter a Subject Number.")
            return
        
        # Determine scenario type based on run type
        if "Nominal" in run_type:
            scenario_type = "nominal"
            duration = 5
        else:  # Off-Nominal
            scenario_type = "off-nominal"
            duration = 10
        
        # Parse nominal procedure from text box
        procedure_text = self.procedure_text.get('1.0', tk.END)
        procedures = [line.strip() for line in procedure_text.split('\n') 
                     if line.strip() and not line.strip().startswith('#')]
        
        if not procedures:
            messagebox.showwarning("No Procedure", 
                                 "Please define at least one nominal procedure command.")
            return
        
        # Start scenario with researcher inputs
        success = self.exp_mgr.start_scenario(scenario_type, duration, procedures, 
                                            subject_number=subject_number, 
                                            run_type=run_type)
        
        if success:
            self.start_btn.config(state=tk.DISABLED)
            self.stop_btn.config(state=tk.NORMAL)
            
            # Update procedure text box to show the actual loaded procedure
            self.procedure_text.delete('1.0', tk.END)
            self.procedure_text.insert('1.0', "# Loaded nominal procedure\n" + "\n".join(procedures))
            
            # Auto-launch appropriate GUI based on run type
            import subprocess
            import sys
            
            if "Control" in run_type:
                # Launch Control GUI
                subprocess.Popen([sys.executable, "control_gui.py"])
                gui_type = "Control GUI"
            else:  # EID
                # Launch EID GUI
                subprocess.Popen([sys.executable, "eid_gui.py"])
                gui_type = "EID GUI"
            
            # Auto-start procedure timing
            self.exp_mgr.start_procedure_timing()
            
            messagebox.showinfo("Experiment Started", 
                              f"{scenario_type.title()} experiment started ({duration} min)\n\n"
                              f"Subject: {subject_number}\n"
                              f"Run Type: {run_type}\n"
                              f"GUI Launched: {gui_type}\n"
                              f"Procedure: {len(procedures)} commands (auto-loaded)\n\n"
                              f"{'Off-nominal: 2 compound faults will inject at 3-5 min' if scenario_type == 'off-nominal' else 'Nominal: No faults'}")
        else:
            messagebox.showerror("Error", "Failed to start experiment. One may already be running.")
            
    def stop_scenario(self):
        """Stop the current experiment"""
        self.exp_mgr.stop_scenario()
        self.start_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)
        
        # Restore default procedure template
        self.procedure_text.delete('1.0', tk.END)
        default_procedure = """# Full Nominal Operations Procedure
COMM_TX_RX
ADCS_ON
EPS_DEPLOY_ARRAY_A
EPS_DEPLOY_ARRAY_B
TCS_DEPLOY_RADIATOR
GPS_ON
STAR_TRACKER_ON
ADCS_SUN_POINT
PROP_STANDBY
PROP_ON
PROP_HEATER_ON
ADCS_DESAT
ADCS_GO_TO 50 -50 100
PROP_VALVE_OPEN
PROP_VALVE_CLOSE
ADCS_SUN_POINT
PROP_VALVE_CLOSE
PROP_HEATER_OFF
PROP_OFF"""
        self.procedure_text.insert('1.0', default_procedure)
        
        messagebox.showinfo("Experiment Stopped", "Experiment has been stopped and data saved.")
    
    def start_procedure(self):
        """Start timing a nominal procedure"""
        if not self.exp_mgr.procedure_start_time:
            self.exp_mgr.start_procedure_timing()
            messagebox.showinfo("Procedure Timing Started", 
                              "Procedure timing started. Commands will be verified against loaded procedure.")
        else:
            messagebox.showinfo("Already Started", "Procedure timing was already started at experiment launch.")
    
    def end_procedure(self):
        """End timing a nominal procedure"""
        self.exp_mgr.end_procedure_timing()
        
        # Get summary info
        errors = len(self.exp_mgr.procedure_errors)
        steps = self.exp_mgr.expected_step
        total = len(self.exp_mgr.loaded_procedure) if self.exp_mgr.loaded_procedure else 0
        
        messagebox.showinfo("Procedure Completed", 
                          f"Procedure timing ended.\n\n"
                          f"Steps completed: {steps}/{total}\n"
                          f"Procedure errors: {errors}\n\n"
                          f"Full details logged to experiment file.")
        
    def update_status(self):
        """Update the status display"""
        try:
            if self.exp_mgr:
                status = self.exp_mgr.get_status()

                self.status_text.config(state=tk.NORMAL)
                self.status_text.delete('1.0', tk.END)
                self.status_text.insert('1.0', status)
                self.status_text.config(state=tk.DISABLED)
        except Exception as e:
            print(f"Error updating status: {e}")
        
        # Schedule next update
        self.root.after(1000, self.update_status)
    
    def on_closing(self):
        """Handle window close event - stop experiment if running"""
        if self.exp_mgr and self.exp_mgr.is_running:
            if messagebox.askokcancel("Quit", "Experiment is running. Stop experiment and close?"):
                self.exp_mgr.stop_scenario()
                self.root.destroy()
        else:
            self.root.destroy()


def main():
    root = tk.Tk()
    app = ExperimentGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
