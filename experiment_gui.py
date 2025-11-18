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
        
        self.create_widgets()
        self.update_status()
        
    def create_widgets(self):
        # Title
        title = tk.Label(self.root, text="🔬 Experiment Control System", 
                        font=('Segoe UI', 14, 'bold'),
                        bg='#2d2d30', fg='white', pady=10)
        title.pack()
        
        # Scenario Selection Frame
        scenario_frame = tk.LabelFrame(self.root, text="Scenario Configuration",
                                      font=('Segoe UI', 11, 'bold'),
                                      bg='#3c3c3c', fg='white', bd=2)
        scenario_frame.pack(fill=tk.X, padx=15, pady=10)
        
        # Scenario type
        tk.Label(scenario_frame, text="Scenario Type:", 
                bg='#3c3c3c', fg='white', font=('Segoe UI', 10)).grid(row=0, column=0, padx=10, pady=5, sticky=tk.W)
        
        self.scenario_var = tk.StringVar(value="nominal")
        tk.Radiobutton(scenario_frame, text="Nominal (5 min)", variable=self.scenario_var,
                      value="nominal", bg='#3c3c3c', fg='white', selectcolor='#0e7c0e',
                      font=('Segoe UI', 9)).grid(row=0, column=1, padx=5, pady=5)
        tk.Radiobutton(scenario_frame, text="Anomaly (10 min)", variable=self.scenario_var,
                      value="anomaly", bg='#3c3c3c', fg='white', selectcolor='#e74c3c',
                      font=('Segoe UI', 9)).grid(row=0, column=2, padx=5, pady=5)
        
        # Nominal Procedure Input
        tk.Label(scenario_frame, text="Nominal Procedure:", 
                bg='#3c3c3c', fg='white', font=('Segoe UI', 10)).grid(row=1, column=0, padx=10, pady=5, sticky=tk.NW)
        
        procedure_frame = tk.Frame(scenario_frame, bg='#3c3c3c')
        procedure_frame.grid(row=1, column=1, columnspan=2, padx=5, pady=5, sticky=tk.EW)
        
        self.procedure_text = scrolledtext.ScrolledText(procedure_frame, height=5, width=40,
                                                        font=('Consolas', 9),
                                                        bg='#1e1e1e', fg='white')
        self.procedure_text.pack()
        self.procedure_text.insert('1.0', "# Enter nominal procedure commands (one per line)\n# Example:\nATT_NADIR\nMODE_NOMINAL\nCLEAR_CAUTIONS")
        
        # Start/Stop Buttons
        button_frame = tk.Frame(scenario_frame, bg='#3c3c3c')
        button_frame.grid(row=2, column=0, columnspan=3, pady=10)
        
        self.start_btn = tk.Button(button_frame, text="▶ Start Scenario",
                                   command=self.start_scenario,
                                   bg='#0e7c0e', fg='white', font=('Segoe UI', 10, 'bold'),
                                   width=15, height=2, cursor='hand2')
        self.start_btn.grid(row=0, column=0, padx=5)
        
        self.stop_btn = tk.Button(button_frame, text="⏹ Stop Scenario",
                                  command=self.stop_scenario,
                                  bg='#e74c3c', fg='white', font=('Segoe UI', 10, 'bold'),
                                  width=15, height=2, cursor='hand2', state=tk.DISABLED)
        self.stop_btn.grid(row=0, column=1, padx=5)
        
        # Operator Action Buttons Frame
        action_frame = tk.LabelFrame(self.root, text="Operator Actions",
                                     font=('Segoe UI', 11, 'bold'),
                                     bg='#3c3c3c', fg='white', bd=2)
        action_frame.pack(fill=tk.X, padx=15, pady=10)
        
        btn_frame = tk.Frame(action_frame, bg='#3c3c3c')
        btn_frame.pack(pady=10)
        
        tk.Button(btn_frame, text="✓ Anomaly Detected",
                 command=self.mark_anomaly_detected,
                 bg='#f1c40f', fg='black', font=('Segoe UI', 9, 'bold'),
                 width=18, height=2).grid(row=0, column=0, padx=5, pady=5)
        
        tk.Button(btn_frame, text="✓ Anomaly Resolved",
                 command=self.mark_anomaly_resolved,
                 bg='#0e7c0e', fg='white', font=('Segoe UI', 9, 'bold'),
                 width=18, height=2).grid(row=0, column=1, padx=5, pady=5)
        
        tk.Button(btn_frame, text="⏱ Start Procedure",
                 command=self.start_procedure,
                 bg='#007acc', fg='white', font=('Segoe UI', 9, 'bold'),
                 width=18, height=2).grid(row=1, column=0, padx=5, pady=5)
        
        tk.Button(btn_frame, text="⏱ End Procedure",
                 command=self.end_procedure,
                 bg='#9b59b6', fg='white', font=('Segoe UI', 9, 'bold'),
                 width=18, height=2).grid(row=1, column=1, padx=5, pady=5)
        
        # Info label for procedure instructions
        procedure_info = tk.Label(action_frame, 
                                 text="Note: 'Start Procedure' loads procedure for verification.\n"
                                      "Test subject executes commands manually with paper procedure.",
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
        
    def start_scenario(self):
        """Start the selected scenario"""
        scenario_type = self.scenario_var.get()
        duration = 5 if scenario_type == "nominal" else 10
        
        # Parse nominal procedure from text box
        procedure_text = self.procedure_text.get('1.0', tk.END)
        procedures = [line.strip() for line in procedure_text.split('\n') 
                     if line.strip() and not line.strip().startswith('#')]
        
        if not procedures:
            messagebox.showwarning("No Procedure", 
                                 "Please define at least one nominal procedure command.")
            return
        
        # Start scenario
        success = self.exp_mgr.start_scenario(scenario_type, duration, procedures)
        
        if success:
            self.start_btn.config(state=tk.DISABLED)
            self.stop_btn.config(state=tk.NORMAL)
            messagebox.showinfo("Scenario Started", 
                              f"{scenario_type.title()} scenario started for {duration} minutes.\n"
                              f"Nominal procedure: {len(procedures)} commands")
        else:
            messagebox.showerror("Error", "Failed to start scenario. One may already be running.")
            
    def stop_scenario(self):
        """Stop the current scenario"""
        self.exp_mgr.stop_scenario()
        self.start_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)
        messagebox.showinfo("Scenario Stopped", "Scenario has been stopped and data saved.")
        
    def mark_anomaly_detected(self):
        """Mark that operator detected the anomaly"""
        self.exp_mgr.mark_anomaly_detected()
        messagebox.showinfo("Marked", "Anomaly detection time recorded.")
        
    def mark_anomaly_resolved(self):
        """Mark that anomaly was resolved"""
        self.exp_mgr.mark_anomaly_resolved()
        messagebox.showinfo("Marked", "Anomaly resolution time recorded.")
        
    def start_procedure(self):
        """Start timing a nominal procedure - loads procedure for verification"""
        self.exp_mgr.start_procedure_timing()
        messagebox.showinfo("Procedure Started", 
                          "Procedure timing started.\n\n"
                          "The test subject should now execute the procedure\n"
                          "they have in hand. Commands will be verified against\n"
                          "the loaded procedure in the background.")
        
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


def main():
    root = tk.Tk()
    app = ExperimentGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
