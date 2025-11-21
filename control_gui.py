"""
Control GUI - Ground Command Display
Left panel showing telemetry channels, EVR log, and caution & warning
Modern UI with improved visual design
"""
import tkinter as tk
from tkinter import ttk, scrolledtext, font
from telemetry_generator import spacecraft
import threading


class ControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Control | Ground Command Display | 6U CubeSat")
        self.root.geometry("900x750")  # Increased width for 2-column display
        
        # Modern color scheme (matching EID GUI)
        self.colors = {
            'bg_main': '#1e1e1e',          # Dark background
            'bg_panel': '#2d2d30',         # Panel background
            'bg_caution': '#f1c40f',       # Yellow for caution
            'bg_warning': '#e74c3c',       # Red for warning
            'fg_main': '#ffffff',          # White text
            'fg_secondary': '#cccccc',     # Light gray text
            'fg_caution': '#f39c12',       # Orange text
            'fg_warning': '#c0392b',       # Dark red text
            'accent': '#007acc',           # Blue accent
            'button_send': '#0e7c0e',      # Green send button
            'button_clear': '#f39c12',     # Orange clear button
        }
        
        self.root.configure(bg=self.colors['bg_main'])
        
        # Start the spacecraft simulator if not already running
        if not spacecraft.running:
            spacecraft.start()
        
        self.create_widgets()
        self.update_data()
        
    def create_widgets(self):
        # Configure custom fonts
        self.font_title = font.Font(family='Segoe UI', size=11, weight='bold')
        self.font_data = font.Font(family='Consolas', size=9)
        self.font_label = font.Font(family='Segoe UI', size=8)
        self.font_button = font.Font(family='Segoe UI', size=9, weight='bold')
        
        # Main container with modern styling
        main_frame = tk.Frame(self.root, bg=self.colors['accent'], padx=2, pady=2)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)
        
        inner_frame = tk.Frame(main_frame, bg=self.colors['bg_main'])
        inner_frame.pack(fill=tk.BOTH, expand=True)
        
        # Title label with modern styling
        title_label = tk.Label(inner_frame, text="📡 Control | Ground Command Display", 
                              font=self.font_title, 
                              bg=self.colors['bg_panel'], 
                              fg=self.colors['fg_main'],
                              pady=8)
        title_label.pack(fill=tk.X, padx=10, pady=(10, 5))
        
        # Telemetry Channels section (top half) with modern styling
        telemetry_frame = tk.LabelFrame(inner_frame, text="📊 Telemetry Channels",
                                       font=self.font_title,
                                       bg=self.colors['bg_panel'], 
                                       fg=self.colors['fg_main'],
                                       relief=tk.FLAT, bd=2)
        telemetry_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Create telemetry channel display with scrollbar
        self.telemetry_text = scrolledtext.ScrolledText(
            telemetry_frame, 
            height=15, 
            width=90,  # Increased width for 2-column display
            font=self.font_data,
            bg=self.colors['bg_panel'],
            fg=self.colors['fg_main'],
            insertbackground=self.colors['fg_main'],
            state=tk.DISABLED
        )
        self.telemetry_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Bottom section with EVR Log and Caution & Warning
        bottom_frame = tk.Frame(inner_frame, bg=self.colors['bg_main'])
        bottom_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # EVR Log (bottom left) with modern styling
        evr_frame = tk.LabelFrame(bottom_frame, text="📋 EVR Log", 
                                 font=self.font_title,
                                 bg=self.colors['bg_panel'], 
                                 fg=self.colors['fg_main'],
                                 relief=tk.FLAT, bd=2)
        evr_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        
        self.evr_text = scrolledtext.ScrolledText(
            evr_frame,
            height=10,
            width=25,
            font=self.font_data,
            bg=self.colors['bg_panel'],
            fg=self.colors['fg_main'],
            insertbackground=self.colors['fg_main'],
            state=tk.DISABLED
        )
        self.evr_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Caution & Warning (bottom right) with modern styling
        cw_frame = tk.LabelFrame(bottom_frame, text="⚠️ Caution & Warning",
                                font=self.font_title,
                                bg=self.colors['bg_panel'], 
                                fg=self.colors['fg_main'],
                                relief=tk.FLAT, bd=2)
        cw_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        self.cw_text = scrolledtext.ScrolledText(
            cw_frame,
            height=10,
            width=50,
            font=self.font_data,
            bg=self.colors['bg_panel'],
            fg=self.colors['fg_main'],
            insertbackground=self.colors['fg_main'],
            wrap=tk.WORD,
            state=tk.DISABLED
        )
        self.cw_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Command Window at bottom with modern styling
        cmd_frame = tk.LabelFrame(inner_frame, text="⌨️ Command Window",
                                 font=self.font_title,
                                 bg=self.colors['bg_panel'], 
                                 fg=self.colors['fg_main'],
                                 relief=tk.FLAT, bd=2)
        cmd_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # Help text
        help_label = tk.Label(cmd_frame, 
                             text="💡 Type commands (see COMMAND_REFERENCE.txt) | Press Enter or click Send",
                             font=self.font_label,
                             bg=self.colors['bg_panel'], 
                             fg=self.colors['fg_secondary'])
        help_label.pack(pady=(5, 2))
        
        cmd_entry_frame = tk.Frame(cmd_frame, bg=self.colors['bg_panel'])
        cmd_entry_frame.pack(fill=tk.X, padx=8, pady=8)
        
        self.cmd_entry = tk.Entry(cmd_entry_frame, 
                                  font=font.Font(family='Consolas', size=10),
                                  bg=self.colors['bg_main'],
                                  fg=self.colors['fg_main'],
                                  insertbackground=self.colors['accent'],
                                  relief=tk.SOLID,
                                  bd=2)
        self.cmd_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5))
        
        send_btn = tk.Button(cmd_entry_frame, text="▶ Send", 
                           command=self.send_command,
                           bg=self.colors['button_send'], 
                           fg='white',
                           font=self.font_button,
                           relief=tk.RAISED,
                           bd=2,
                           cursor='hand2',
                           activebackground=self.colors['accent'])
        send_btn.pack(side=tk.RIGHT, padx=2)
        
        clear_btn = tk.Button(cmd_entry_frame, text="🗑️ Clear Cautions", 
                            command=self.clear_cautions,
                            bg=self.colors['button_clear'], 
                            fg='white',
                            font=self.font_button,
                            relief=tk.RAISED,
                            bd=2,
                            cursor='hand2',
                            activebackground=self.colors['bg_caution'])
        clear_btn.pack(side=tk.RIGHT)
        
        self.cmd_entry.bind('<Return>', lambda e: self.send_command())
        
    def send_command(self):
        """Send command to spacecraft"""
        command = self.cmd_entry.get().strip()
        if command:
            try:
                # Send command and check if it was accepted
                queued = spacecraft.send_command(command)
                
                # Wait for command to be processed
                import time
                time.sleep(0.2)
                
                # Check execution result - verify it's the command we just sent
                success = queued and (spacecraft.last_command == command) and spacecraft.last_command_success
                
                # Log command result to experiment manager if active
                try:
                    from experiment_manager import experiment_manager
                    if experiment_manager and experiment_manager.is_running:
                        if not success:
                            experiment_manager.log_command(command, success=False, error_msg="Command rejected by spacecraft")
                        else:
                            experiment_manager.log_command(command)
                except Exception as e:
                    print(f"Error logging command to experiment manager: {e}")
                    pass  # Experiment manager not active
            except Exception as e:
                # Show a small message in EVR to avoid GUI crash
                try:
                    self.evr_text.config(state=tk.NORMAL)
                    self.evr_text.insert(tk.END, f"ERROR sending command: {e}\n")
                    self.evr_text.config(state=tk.DISABLED)
                except Exception:
                    pass
            finally:
                self.cmd_entry.delete(0, tk.END)
            
    def clear_cautions(self):
        """Clear all caution states"""
        spacecraft.send_command("CLEAR_CAUTIONS")
            
    def update_data(self):
        """Update all displays with latest data"""
        # Update telemetry channels
        channels = spacecraft.get_telemetry_channels()
        
        # Save scroll position
        telemetry_scroll_pos = self.telemetry_text.yview()
        
        self.telemetry_text.config(state=tk.NORMAL)
        self.telemetry_text.delete(1.0, tk.END)
        
        # Format as 2-column table
        # Calculate midpoint for splitting into two columns
        mid = (len(channels) + 1) // 2
        left_channels = channels[:mid]
        right_channels = channels[mid:]
        
        # Header
        self.telemetry_text.insert(tk.END, f"{'Telemetry Channel':<28} {'Data':>10}  |  {'Telemetry Channel':<28} {'Data':>10}\n")
        self.telemetry_text.insert(tk.END, "-" * 90 + "\n")
        
        # Display two columns side by side
        for i in range(max(len(left_channels), len(right_channels))):
            left_line = ""
            right_line = ""
            
            if i < len(left_channels):
                channel_name, data, is_stale = left_channels[i]
                stale_marker = " S" if is_stale else ""
                left_line = f"{channel_name:<28} {data:>10}{stale_marker}"
            else:
                left_line = " " * 38
            
            if i < len(right_channels):
                channel_name, data, is_stale = right_channels[i]
                stale_marker = " S" if is_stale else ""
                right_line = f"{channel_name:<28} {data:>10}{stale_marker}"
            
            self.telemetry_text.insert(tk.END, f"{left_line}  |  {right_line}\n")
        
        # Restore scroll position
        self.telemetry_text.yview_moveto(telemetry_scroll_pos[0])
        self.telemetry_text.config(state=tk.DISABLED)
        
        # Update EVR log
        evr_log = spacecraft.get_evr_log()
        
        # Save scroll position
        evr_scroll_pos = self.evr_text.yview()
        
        self.evr_text.config(state=tk.NORMAL)
        self.evr_text.delete(1.0, tk.END)
        
        # Show last 20 entries
        for entry in evr_log[-20:]:
            self.evr_text.insert(tk.END, entry + "\n")
        
        # Restore scroll position
        self.evr_text.yview_moveto(evr_scroll_pos[0])
        self.evr_text.config(state=tk.DISABLED)
        
        # Update caution & warning
        cautions, warnings = spacecraft.get_cautions_warnings()
        
        # Save scroll position
        cw_scroll_pos = self.cw_text.yview()
        
        self.cw_text.config(state=tk.NORMAL)
        self.cw_text.delete(1.0, tk.END)
        
        if warnings:
            self.cw_text.insert(tk.END, "*** WARNINGS ***\n", 'warning')
            for warning in warnings:
                self.cw_text.insert(tk.END, f"⚠ {warning}\n", 'warning')
            self.cw_text.insert(tk.END, "\n")
            
        if cautions:
            self.cw_text.insert(tk.END, "CAUTIONS\n", 'caution')
            for caution in cautions:
                self.cw_text.insert(tk.END, f"⚡ {caution}\n", 'caution')
                
        # Configure text tags for colors with modern styling
        self.cw_text.tag_config('warning', 
                               foreground=self.colors['bg_warning'], 
                               font=self.font_data.copy())
        self.cw_text.tag_config('caution', 
                               foreground=self.colors['bg_caution'], 
                               font=self.font_data)
        
        # Restore scroll position
        self.cw_text.yview_moveto(cw_scroll_pos[0])
        self.cw_text.config(state=tk.DISABLED)
        
        # Schedule next update (1 Hz)
        self.root.after(1000, self.update_data)
        
    def on_closing(self):
        """Clean shutdown"""
        self.root.destroy()


def main():
    root = tk.Tk()
    app = ControlGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()
