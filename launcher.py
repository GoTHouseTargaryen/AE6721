"""
Launcher for Ground Command Display System
Runs Control, EID, and Experiment GUIs in a single process so they share the same simulator instance
"""
import tkinter as tk
from tkinter import messagebox
import os
import sys

# Import GUIs to run them as Toplevel windows
try:
    from control_gui import ControlGUI
    from eid_gui import EIDGUI
    from experiment_gui import ExperimentGUI
except Exception as e:
    # Defer import errors to runtime button handlers to show messagebox errors
    ControlGUI = None
    EIDGUI = None
    ExperimentGUI = None


class LauncherGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Spacecraft Ground Command Display - Launcher")
        self.root.geometry("450x400")
        self.root.configure(bg='lightblue')
        
        # Track open windows
        self.windows = []
        
        self.create_widgets()
        
    def create_widgets(self):
        # Title
        title = tk.Label(self.root, text="Spacecraft Ground Command Display",
                        font=('Arial', 14, 'bold'), bg='lightblue', fg='darkblue')
        title.pack(pady=20)
        
        subtitle = tk.Label(self.root, text="LEO Spacecraft Telemetry System",
                          font=('Arial', 10), bg='lightblue', fg='darkblue')
        subtitle.pack(pady=5)
        
        # Buttons frame
        btn_frame = tk.Frame(self.root, bg='lightblue')
        btn_frame.pack(pady=30)
        
        # Launch both GUIs button
        launch_both_btn = tk.Button(btn_frame, text="Launch Both GUIs",
                                   command=self.launch_both,
                                   bg='green', fg='white',
                                   font=('Arial', 12, 'bold'),
                                   width=20, height=2)
        launch_both_btn.pack(pady=10)
        
        # Individual launch buttons
        control_btn = tk.Button(btn_frame, text="Launch Control GUI Only",
                              command=self.launch_control,
                              bg='blue', fg='white',
                              font=('Arial', 10),
                              width=20)
        control_btn.pack(pady=5)
        
        eid_btn = tk.Button(btn_frame, text="Launch EID GUI Only",
                          command=self.launch_eid,
                          bg='blue', fg='white',
                          font=('Arial', 10),
                          width=20)
        eid_btn.pack(pady=5)
        
        # Experiment control button
        exp_btn = tk.Button(btn_frame, text="Launch Experiment Control",
                          command=self.launch_experiment,
                          bg='purple', fg='white',
                          font=('Arial', 10),
                          width=20)
        exp_btn.pack(pady=5)
        
        # Test simulator button
        sim_btn = tk.Button(btn_frame, text="Test Simulator Only",
                          command=self.test_simulator,
                          bg='orange', fg='white',
                          font=('Arial', 10),
                          width=20)
        sim_btn.pack(pady=5)
        
        # Info label
        info_label = tk.Label(self.root, 
                             text="Close this window to stop all components",
                             font=('Arial', 8, 'italic'),
                             bg='lightblue', fg='gray')
        info_label.pack(side=tk.BOTTOM, pady=10)
        
    def launch_control(self):
        """Launch Control GUI as a Toplevel window"""
        try:
            if ControlGUI is None:
                from control_gui import ControlGUI as _CG
            else:
                _CG = ControlGUI
            win = tk.Toplevel(self.root)
            win.title("Control GUI")
            _CG(win)
            self.windows.append(win)
        except Exception as e:
            messagebox.showerror("Error", f"Failed to launch Control GUI: {e}")
            
    def launch_eid(self):
        """Launch EID GUI as a Toplevel window"""
        try:
            if EIDGUI is None:
                from eid_gui import EIDGUI as _EG
            else:
                _EG = EIDGUI
            win = tk.Toplevel(self.root)
            win.title("EID GUI")
            _EG(win)
            self.windows.append(win)
        except Exception as e:
            messagebox.showerror("Error", f"Failed to launch EID GUI: {e}")
            
    def launch_experiment(self):
        """Launch Experiment Control GUI as a Toplevel window"""
        try:
            if ExperimentGUI is None:
                from experiment_gui import ExperimentGUI as _XG
            else:
                _XG = ExperimentGUI
            win = tk.Toplevel(self.root)
            win.title("Experiment Control")
            _XG(win)
            self.windows.append(win)
        except Exception as e:
            messagebox.showerror("Error", f"Failed to launch Experiment Control: {e}")
            
    def launch_both(self):
        """Launch both GUIs"""
        self.launch_control()
        self.root.after(500, self.launch_eid)  # Slight delay between launches
        
    def test_simulator(self):
        """Test the simulator by opening a small window that prints EVR entries"""
        try:
            from telemetry_generator import spacecraft
            # Start simulator if needed
            if not spacecraft.running:
                spacecraft.start()

            win = tk.Toplevel(self.root)
            win.title("Simulator EVR Monitor")
            txt = tk.Text(win, height=20, width=80)
            txt.pack(fill=tk.BOTH, expand=True)

            def refresh():
                evr = spacecraft.get_evr_log()
                txt.delete('1.0', tk.END)
                for line in evr[-50:]:
                    txt.insert(tk.END, line + "\n")
                win.after(1000, refresh)

            refresh()
            self.windows.append(win)
        except Exception as e:
            messagebox.showerror("Error", f"Failed to open simulator monitor: {e}")
            
    def on_closing(self):
        """Clean shutdown of all windows and simulator"""
        try:
            from telemetry_generator import spacecraft
            spacecraft.stop()
        except Exception:
            pass
        # Destroy any open Toplevel windows
        for win in self.windows:
            try:
                win.destroy()
            except Exception:
                pass
        self.root.destroy()


def main():
    root = tk.Tk()
    app = LauncherGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()
