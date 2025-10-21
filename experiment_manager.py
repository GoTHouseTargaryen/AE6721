"""
Experiment Manager - Controlled Testing System
Manages nominal and anomaly scenarios for research data collection
Logs operator performance metrics including response times and command errors
"""
import time
import json
import random
from datetime import datetime
from threading import Thread, RLock


class ExperimentManager:
    def __init__(self, spacecraft):
        self.spacecraft = spacecraft
        self.is_running = False
        self.scenario_type = None  # 'nominal' or 'anomaly'
        self.scenario_thread = None
        
        # Timing and logging
        self.start_time = None
        self.anomaly_injection_time = None
        self.anomaly_detected_time = None
        self.anomaly_resolved_time = None
        self.procedure_start_time = None
        self.procedure_end_time = None
        
        # Command tracking
        self.command_log = []
        self.command_count = 0
        self.command_errors = []
        
        # Nominal procedure tracking
        self.nominal_procedure = []
        self.procedure_step = 0
        self.procedure_times = []
        
        # Data lock for thread safety
        self.lock = RLock()
        
        # Log file path
        self.log_file = None
        
    def start_scenario(self, scenario_type, duration_minutes, nominal_procedure=None):
        """
        Start an experimental scenario
        
        Args:
            scenario_type: 'nominal' or 'anomaly'
            duration_minutes: Duration in minutes (5 for nominal, 10 for anomaly)
            nominal_procedure: List of command strings defining the nominal procedure
        """
        if self.is_running:
            self.log_event("ERROR", "Scenario already running")
            return False
        
        self.scenario_type = scenario_type
        self.start_time = time.time()
        self.is_running = True
        self.command_count = 0
        self.command_errors = []
        self.procedure_step = 0
        self.procedure_times = []
        # reset timing markers from any previous run
        self.anomaly_injection_time = None
        self.anomaly_detected_time = None
        self.anomaly_resolved_time = None
        self.procedure_start_time = None
        self.procedure_end_time = None
        
        if nominal_procedure:
            self.nominal_procedure = nominal_procedure
        # schedule first automatic procedure step a few seconds after start
        self._procedure_interval_sec = 30
        self._next_procedure_time = self.start_time + 5 if self.nominal_procedure else None
        
        # Create log file with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file = f"experiment_log_{scenario_type}_{timestamp}.json"
        
        # Initialize log structure
        self.log_data = {
            'scenario_type': scenario_type,
            'start_time': datetime.now().isoformat(),
            'duration_minutes': duration_minutes,
            'nominal_procedure': self.nominal_procedure,
            'events': [],
            'commands': [],
            'metrics': {}
        }
        
        self.log_event("SCENARIO_START", f"Starting {scenario_type} scenario for {duration_minutes} minutes")
        
        # Start scenario thread
        self.scenario_thread = Thread(target=self._run_scenario, args=(duration_minutes,))
        self.scenario_thread.daemon = True
        self.scenario_thread.start()
        
        return True
        
    def _run_scenario(self, duration_minutes):
        """Run the scenario for the specified duration"""
        duration_seconds = duration_minutes * 60
        end_time = time.time() + duration_seconds
        
        # For anomaly scenario, schedule random anomaly injection
        if self.scenario_type == 'anomaly':
            # Inject anomaly between 2nd and 9th minute
            anomaly_delay = random.uniform(120, 540)  # 2-9 minutes in seconds
            anomaly_time = time.time() + anomaly_delay
            self.log_event("ANOMALY_SCHEDULED", f"Anomaly will occur in {anomaly_delay:.1f} seconds")
        else:
            anomaly_time = None
        
        # Monitor scenario
        while time.time() < end_time and self.is_running:
            current_time = time.time()
            
            # Auto-send nominal procedure steps on cadence
            if self.nominal_procedure and self._next_procedure_time and current_time >= self._next_procedure_time:
                with self.lock:
                    if self.procedure_step < len(self.nominal_procedure):
                        cmd = self.nominal_procedure[self.procedure_step]
                        try:
                            self.spacecraft.send_command(cmd)
                            self.log_command(cmd, success=True)
                            self.log_event("AUTO_PROCEDURE", f"Sent step {self.procedure_step + 1}/{len(self.nominal_procedure)}: {cmd}")
                        except Exception as e:
                            self.log_command(cmd, success=False, error_msg=str(e))
                        self.procedure_step += 1
                        self._next_procedure_time = current_time + self._procedure_interval_sec if self.procedure_step < len(self.nominal_procedure) else None

            # Check if it's time to inject anomaly
            if anomaly_time and current_time >= anomaly_time and not self.anomaly_injection_time:
                self._inject_anomaly()
                
            time.sleep(1)
        
        # Scenario complete
        self._end_scenario()
        
    def _inject_anomaly(self):
        """Inject a random anomaly into the spacecraft"""
        with self.lock:
            self.anomaly_injection_time = time.time()
            
            # Choose random anomaly type
            anomaly_types = [
                ('battery_low', 'Battery voltage dropping below threshold'),
                ('attitude_error', 'Attitude control system error - off-pointing'),
                ('thermal_warning', 'Thermal system warning - temperature out of range'),
                ('comm_degraded', 'Communication signal degraded'),
                ('power_fault', 'Power system fault detected')
            ]
            
            anomaly_type, description = random.choice(anomaly_types)
            
            self.log_event("ANOMALY_INJECTED", f"{anomaly_type}: {description}")
            
            # Trigger the anomaly in spacecraft
            if anomaly_type == 'battery_low':
                self.spacecraft.send_command("INJECT_ANOMALY_BATTERY")
            elif anomaly_type == 'attitude_error':
                self.spacecraft.send_command("INJECT_ANOMALY_ATTITUDE")
            elif anomaly_type == 'thermal_warning':
                self.spacecraft.send_command("INJECT_ANOMALY_THERMAL")
            elif anomaly_type == 'comm_degraded':
                self.spacecraft.send_command("INJECT_ANOMALY_COMM")
            elif anomaly_type == 'power_fault':
                self.spacecraft.send_command("INJECT_ANOMALY_POWER")
                
    def log_command(self, command, success=True, error_msg=None):
        """Log a command sent by the operator"""
        with self.lock:
            self.command_count += 1
            elapsed = time.time() - self.start_time if self.start_time else 0
            
            cmd_entry = {
                'timestamp': datetime.now().isoformat(),
                'elapsed_seconds': elapsed,
                'command_number': self.command_count,
                'command': command,
                'success': success,
                'error': error_msg
            }
            
            self.command_log.append(cmd_entry)
            self.log_data['commands'].append(cmd_entry)
            
            if not success:
                self.command_errors.append(cmd_entry)
                self.log_event("COMMAND_ERROR", f"Command '{command}' failed: {error_msg}")
            else:
                self.log_event("COMMAND_EXECUTED", f"Command #{self.command_count}: {command}")
                
    def log_event(self, event_type, description):
        """Log a general event"""
        with self.lock:
            elapsed = time.time() - self.start_time if self.start_time else 0
            
            event = {
                'timestamp': datetime.now().isoformat(),
                'elapsed_seconds': elapsed,
                'event_type': event_type,
                'description': description
            }
            
            self.log_data['events'].append(event)
            
            # Print to console for debugging
            print(f"[{elapsed:.1f}s] {event_type}: {description}")
            
    def mark_anomaly_detected(self):
        """Mark when operator detected the anomaly"""
        if self.anomaly_injection_time and not self.anomaly_detected_time:
            with self.lock:
                self.anomaly_detected_time = time.time()
                detection_delay = self.anomaly_detected_time - self.anomaly_injection_time
                self.log_event("ANOMALY_DETECTED", f"Operator detected anomaly after {detection_delay:.2f} seconds")
                
    def mark_anomaly_resolved(self):
        """Mark when anomaly was resolved"""
        if self.anomaly_injection_time and not self.anomaly_resolved_time:
            with self.lock:
                self.anomaly_resolved_time = time.time()
                
                if self.anomaly_detected_time:
                    response_time = self.anomaly_resolved_time - self.anomaly_detected_time
                    total_time = self.anomaly_resolved_time - self.anomaly_injection_time
                    self.log_event("ANOMALY_RESOLVED", 
                                 f"Anomaly resolved in {response_time:.2f}s (total: {total_time:.2f}s)")
                else:
                    total_time = self.anomaly_resolved_time - self.anomaly_injection_time
                    self.log_event("ANOMALY_RESOLVED", 
                                 f"Anomaly resolved in {total_time:.2f}s (no detection marked)")
                    
    def start_procedure_timing(self):
        """Start timing a nominal procedure"""
        with self.lock:
            self.procedure_start_time = time.time()
            self.log_event("PROCEDURE_START", "Nominal procedure started")
            
    def end_procedure_timing(self):
        """End timing a nominal procedure"""
        if self.procedure_start_time:
            with self.lock:
                self.procedure_end_time = time.time()
                procedure_duration = self.procedure_end_time - self.procedure_start_time
                self.procedure_times.append(procedure_duration)
                self.log_event("PROCEDURE_COMPLETE", f"Nominal procedure completed in {procedure_duration:.2f}s")
                self.procedure_start_time = None
                
    def _end_scenario(self):
        """End the scenario and calculate metrics"""
        with self.lock:
            if not self.is_running or not self.start_time:
                return
                
            self.is_running = False
            end_time = time.time()
            total_duration = end_time - self.start_time
            
            # Calculate metrics
            metrics = {
                'total_duration_seconds': total_duration,
                'total_commands': self.command_count,
                'command_errors': len(self.command_errors),
                'command_error_rate': len(self.command_errors) / max(1, self.command_count),
                'procedures_completed': len(self.procedure_times),
                'average_procedure_time': sum(self.procedure_times) / len(self.procedure_times) if self.procedure_times else 0,
            }
            
            if self.scenario_type == 'anomaly':
                if self.anomaly_injection_time and self.start_time:
                    metrics['anomaly_injection_time'] = self.anomaly_injection_time - self.start_time
                    
                if self.anomaly_detected_time and self.anomaly_injection_time:
                    metrics['anomaly_detection_delay'] = self.anomaly_detected_time - self.anomaly_injection_time
                    
                if self.anomaly_resolved_time and self.anomaly_injection_time:
                    if self.anomaly_detected_time:
                        metrics['anomaly_response_time'] = self.anomaly_resolved_time - self.anomaly_detected_time
                    metrics['anomaly_total_time'] = self.anomaly_resolved_time - self.anomaly_injection_time
                    
            self.log_data['metrics'] = metrics
            self.log_data['end_time'] = datetime.now().isoformat()
            
            # Save to JSON file
            if self.log_file:
                with open(self.log_file, 'w') as f:
                    json.dump(self.log_data, f, indent=2)
                
            self.log_event("SCENARIO_END", f"Scenario complete. Log saved to {self.log_file}")
            
    def stop_scenario(self):
        """Stop the current scenario"""
        if self.is_running:
            self.is_running = False
            self._end_scenario()
            
    def get_status(self):
        """Get current scenario status"""
        with self.lock:
            if not self.is_running or not self.start_time:
                return "No scenario running"
                
            elapsed = time.time() - self.start_time
            status = f"Scenario: {self.scenario_type}\n"
            status += f"Elapsed: {elapsed:.1f}s\n"
            status += f"Commands: {self.command_count}\n"
            status += f"Errors: {len(self.command_errors)}\n"
            
            if self.scenario_type == 'anomaly' and self.anomaly_injection_time:
                status += "Anomaly: ACTIVE\n"
                if self.anomaly_detected_time:
                    status += "Status: DETECTED\n"
                if self.anomaly_resolved_time:
                    status += "Status: RESOLVED\n"
                    
            return status


# Global experiment manager instance
experiment_manager = None


def initialize_experiment_manager(spacecraft):
    """Initialize the global experiment manager"""
    global experiment_manager
    experiment_manager = ExperimentManager(spacecraft)
    return experiment_manager
