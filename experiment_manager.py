"""
Experiment Manager - Controlled Testing System
Manages nominal and anomaly scenarios for research data collection
Logs operator performance metrics including response times and command errors
"""
import os
import time
import json
import csv
import random
from datetime import datetime
from threading import Thread, RLock


class ExperimentManager:
    def __init__(self, spacecraft):
        self.spacecraft = spacecraft
        self.is_running = False
        self.scenario_type = None  # 'nominal', 'anomaly', or 'experimental'
        self.scenario_thread = None
        
        # Researcher inputs
        self.subject_number = None
        self.run_type = None  # 'Nominal Control', 'Nominal EID', 'Off-Nominal Control', 'Off-Nominal EID'
        
        # Timing and logging
        self.start_time = None
        self.anomaly_injection_time = None
        self.anomaly_detected_time = None
        self.anomaly_resolved_time = None
        self.procedure_start_time = None
        self.procedure_end_time = None
        self.procedure_completion_time = None
        
        # Anomaly tracking
        self.injected_anomaly_type = None
        
        # Caution/Warning tracking
        self.cautions_warnings_log = []  # List of all cautions/warnings that occurred
        self.caution_warning_response_times = []  # Time to respond to each
        self.active_cautions_warnings = {}  # Currently active {issue: start_time}
        
        # Command tracking
        self.command_log = []
        self.command_count = 0
        self.command_errors = []
        self.off_procedure_commands = []  # Track commands that deviate from procedure
        
        # Nominal procedure tracking
        self.nominal_procedure = []
        self.procedure_step = 0
        self.procedure_times = []
        
        # Procedure verification (loaded but not auto-executed)
        self.loaded_procedure = []
        self.procedure_active = False
        self.procedure_errors = []
        self.expected_step = 0
        
        # Data lock for thread safety
        self.lock = RLock()
        
        # Log file path
        self.log_file = None
        
    def start_scenario(self, scenario_type, duration_minutes, nominal_procedure=None, subject_number=None, run_type=None):
        """
        Start an experimental scenario
        
        Args:
            scenario_type: 'nominal', 'anomaly', or 'experimental'
            duration_minutes: Duration in minutes (5 for nominal, 10 for anomaly/experimental)
            nominal_procedure: List of command strings defining the nominal procedure
            subject_number: Researcher-assigned subject identifier
            run_type: 'Nominal Control', 'Nominal EID', 'Off-Nominal Control', or 'Off-Nominal EID'
        """
        if self.is_running:
            self.log_event("ERROR", "Scenario already running")
            return False
        
        with self.lock:
            # Store researcher inputs
            self.subject_number = subject_number
            self.run_type = run_type
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
            # Load the procedure for verification
            self.loaded_procedure = nominal_procedure
            self.expected_step = 0
            self.procedure_errors = []
            self.procedure_active = True
            self.procedure_start_time = time.time()
        
        # Create Data folder and set up consolidated log files
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        os.makedirs("Data", exist_ok=True)
        
        # Use consolidated filenames that append data
        self.json_consolidated_file = "Data/Experiment_Data.json"
        self.csv_consolidated_file = "Data/Experiment_Data.csv"
        
        # Store this run's unique identifier for later reference
        self.run_timestamp = timestamp
        
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
        
        # For anomaly scenario, schedule random anomaly injection (2-9 minutes)
        if self.scenario_type == 'anomaly':
            anomaly_delay = random.uniform(120, 540)  # 2-9 minutes in seconds
            anomaly_time = time.time() + anomaly_delay
            self.log_event("ANOMALY_SCHEDULED", f"Anomaly will occur in {anomaly_delay:.1f} seconds")
        # For off-nominal scenario, schedule compound anomaly injection (3-5 minutes)
        elif self.scenario_type == 'off-nominal':
            anomaly_delay = random.uniform(180, 300)  # 3-5 minutes in seconds
            anomaly_time = time.time() + anomaly_delay
            self.log_event("COMPOUND_ANOMALY_SCHEDULED", f"Compound anomaly (2 simultaneous faults) will occur in {anomaly_delay:.1f} seconds")
        # For experimental scenario, schedule experimental fault injection (1-5 minutes)
        elif self.scenario_type == 'experimental':
            anomaly_delay = random.uniform(60, 300)  # 1-5 minutes in seconds
            anomaly_time = time.time() + anomaly_delay
            self.log_event("EXPERIMENTAL_FAULT_SCHEDULED", f"Experimental fault will occur in {anomaly_delay:.1f} seconds")
        else:
            anomaly_time = None
        
        # Monitor scenario
        while time.time() < end_time and self.is_running:
            current_time = time.time()
            
            # NOTE: No auto-execution of procedures
            # Commands are sent manually by test subject and verified against loaded procedure
            
            # Check if it's time to inject anomaly or experimental fault
            if anomaly_time and current_time >= anomaly_time and not self.anomaly_injection_time:
                if self.scenario_type == 'experimental':
                    self._inject_experimental_fault()
                elif self.scenario_type == 'off-nominal':
                    self._inject_compound_anomaly()
                else:
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
            self.injected_anomaly_type = anomaly_type  # Store for CSV output
            
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
    
    def _inject_experimental_fault(self):
        """Inject one of three guaranteed experimental faults (CDH, TTC, or AVI)"""
        with self.lock:
            self.anomaly_injection_time = time.time()
            
            # Choose random experimental fault type
            fault_types = ['CDH', 'TTC', 'AVI']
            fault_type = random.choice(fault_types)
            self.injected_anomaly_type = f"EXPERIMENTAL_{fault_type}"  # Store for CSV output
            
            descriptions = {
                'CDH': 'Command & Data Handling fault - all commanding blocked except CDH_POWER_CYCLE',
                'TTC': 'Telemetry Tracking & Command fault - 75%+ telemetry stale, commanding still works',
                'AVI': 'Avionics fault - all commanding blocked except AVI_POWER_CYCLE, all telemetry stale except AVI'
            }
            
            self.log_event("EXPERIMENTAL_FAULT_INJECTED", f"{fault_type}: {descriptions[fault_type]}")
            
            # Inject the fault directly into spacecraft
            self.spacecraft.inject_experimental_fault(fault_type)
    
    def _inject_compound_anomaly(self):
        """
        Inject 2 simultaneous compound anomalies for off-nominal scenario.
        These occur at inopportune times (3-5 minutes) and test recovery under pressure.
        """
        with self.lock:
            self.anomaly_injection_time = time.time()
            
            # Define 6 compound anomaly types
            compound_anomalies = [
                {
                    'name': 'ADCS_FAULT_DURING_MANEUVER',
                    'description': 'ADCS faults during slew/desat - spacecraft tumbles, telemetry stale except fault flag',
                    'commands': ['INJECT_COMPOUND_ADCS_FAULT']
                },
                {
                    'name': 'BATTERY_DRAIN',
                    'description': 'Battery A or B drains despite working solar arrays - requires cross charge',
                    'commands': ['INJECT_COMPOUND_BATTERY_DRAIN']
                },
                {
                    'name': 'SOLAR_ARRAY_FAULT',
                    'description': 'Solar Array A or B faults - requires cross charge to charge other battery',
                    'commands': ['INJECT_COMPOUND_SOLAR_FAULT']
                },
                {
                    'name': 'COMM_FAULT',
                    'description': 'COMM faults - all telemetry stale, system returns to RX mode',
                    'commands': ['INJECT_COMPOUND_COMM_FAULT']
                },
                {
                    'name': 'PROP_VALVE_STUCK_OPEN',
                    'description': 'PROP faults during maneuver - valve stuck open, propellant leaking',
                    'commands': ['INJECT_COMPOUND_PROP_FAULT']
                },
                {
                    'name': 'TCS_HEATERS_STUCK_ON',
                    'description': 'TCS faults - all heaters stuck ON, temperatures rising to caution/warning',
                    'commands': ['INJECT_COMPOUND_TCS_FAULT']
                }
            ]
            
            # Randomly select 2 different anomalies
            selected_anomalies = random.sample(compound_anomalies, 2)
            
            # Store compound anomaly types for CSV
            anomaly_names = [a['name'] for a in selected_anomalies]
            self.injected_anomaly_type = f"COMPOUND: {anomaly_names[0]} + {anomaly_names[1]}"
            
            self.log_event("COMPOUND_ANOMALY_INJECTED", 
                         f"2 simultaneous faults injected:")
            
            # Inject both anomalies
            for anomaly in selected_anomalies:
                self.log_event("COMPOUND_ANOMALY_DETAIL", 
                             f"  - {anomaly['name']}: {anomaly['description']}")
                
                # Send commands to spacecraft to trigger the anomaly
                for cmd in anomaly['commands']:
                    self.spacecraft.send_command(cmd)
                
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
                'error': error_msg,
                'on_procedure': False,
                'addresses_warning': False
            }
            
            self.command_log.append(cmd_entry)
            self.log_data['commands'].append(cmd_entry)
            
            if not success:
                self.command_errors.append(cmd_entry)
                self.log_event("COMMAND_ERROR", f"Command '{command}' failed: {error_msg}")
            else:
                self.log_event("COMMAND_EXECUTED", f"Command #{self.command_count}: {command}")
                
            # Check against loaded procedure if active
            if self.procedure_active and self.loaded_procedure:
                is_on_procedure, addresses_warning = self._verify_command_against_procedure(command)
                cmd_entry['on_procedure'] = is_on_procedure
                cmd_entry['addresses_warning'] = addresses_warning
                
                # Track off-procedure commands that don't address warnings
                if not is_on_procedure and not addresses_warning:
                    off_proc_entry = cmd_entry.copy()
                    off_proc_entry['expected_command'] = self.loaded_procedure[self.expected_step] if self.expected_step < len(self.loaded_procedure) else 'N/A'
                    self.off_procedure_commands.append(off_proc_entry)
                    self.log_event("OFF_PROCEDURE_COMMAND", 
                                 f"⚠ Command '{command}' is OFF-PROCEDURE (not expected and doesn't address caution/warning)")
                
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
                    
    def load_procedure(self, procedure_steps):
        """
        Load a procedure for background verification
        
        Args:
            procedure_steps: List of command strings that define the correct procedure
        """
        with self.lock:
            self.loaded_procedure = procedure_steps
            self.expected_step = 0
            self.procedure_errors = []
            self.log_event("PROCEDURE_LOADED", f"Loaded procedure with {len(procedure_steps)} steps: {procedure_steps}")
            
    def start_procedure_timing(self):
        """Start timing a nominal procedure (test subject begins executing)"""
        with self.lock:
            self.procedure_start_time = time.time()
            self.procedure_active = True
            self.expected_step = 0
            
            # Load default 3-step procedure if none loaded
            if not self.loaded_procedure:
                self.loaded_procedure = [
                    "ADCS_ON",
                    "AVI_POWER_CYCLE", 
                    "PROP_ON"
                ]
                self.log_event("PROCEDURE_LOADED", f"Default procedure loaded: {self.loaded_procedure}")
            
            self.log_event("PROCEDURE_START", f"Test subject beginning procedure execution - {len(self.loaded_procedure)} steps expected")
            
    def end_procedure_timing(self):
        """End timing a nominal procedure"""
        if self.procedure_start_time:
            with self.lock:
                self.procedure_end_time = time.time()
                procedure_duration = self.procedure_end_time - self.procedure_start_time
                self.procedure_times.append(procedure_duration)
                self.procedure_completion_time = procedure_duration  # Store for CSV
                
                # Summary of procedure execution
                total_errors = len(self.procedure_errors)
                steps_completed = self.expected_step
                steps_expected = len(self.loaded_procedure)
                
                self.log_event("PROCEDURE_COMPLETE", 
                             f"Procedure ended after {procedure_duration:.2f}s | "
                             f"Steps: {steps_completed}/{steps_expected} | "
                             f"Errors: {total_errors}")
                
                self.procedure_start_time = None
                self.procedure_active = False
                
    def _verify_command_against_procedure(self, command):
        """
        Verify if a command matches the expected step in the loaded procedure,
        or if it's a valid response to an active caution/warning.
        
        Args:
            command: The command string that was just executed
            
        Returns:
            tuple: (is_on_procedure, addresses_warning)
        """
        if not self.loaded_procedure or self.expected_step >= len(self.loaded_procedure):
            # No procedure loaded or already completed all steps
            return (False, False)
            
        expected_command = self.loaded_procedure[self.expected_step]
        is_on_procedure = False
        addresses_warning = False
        
        if command == expected_command:
            # Correct command - on procedure
            is_on_procedure = True
            self.expected_step += 1
            self.log_event("PROCEDURE_STEP_CORRECT", 
                         f"✓ Step {self.expected_step}/{len(self.loaded_procedure)} correct: {command}")
        else:
            # Check if command addresses an active caution or warning
            addresses_warning = self._is_command_addressing_warning(command)
            
            if addresses_warning:
                self.log_event("CAUTION_WARNING_RESPONSE", 
                             f"Command '{command}' addresses active caution/warning (allowed deviation)")
            else:
                # Wrong command - off-procedure error
                error_entry = {
                    'step_number': self.expected_step + 1,
                    'expected_command': expected_command,
                    'actual_command': command,
                    'timestamp': datetime.now().isoformat()
                }
                self.procedure_errors.append(error_entry)
                
                self.log_event("PROCEDURE_STEP_ERROR", 
                             f"✗ Step {self.expected_step + 1}/{len(self.loaded_procedure)} ERROR - "
                             f"Expected: '{expected_command}', Got: '{command}'")
        
        return (is_on_procedure, addresses_warning)
    
    def _is_command_addressing_warning(self, command):
        """
        Check if a command is a valid response to an active caution or warning.
        This allows operators to deviate from procedure to address spacecraft issues.
        
        Args:
            command: The command string to check
            
        Returns:
            bool: True if command addresses an active caution/warning
        """
        # Get current spacecraft state
        cautions = self.spacecraft.cautions.copy()
        warnings = self.spacecraft.warnings.copy()
        
        # Define valid response commands for specific cautions/warnings
        caution_response_map = {
            # Battery-related
            'Battery A voltage low': ['EPS_CROSS_CHARGE_ENABLE', 'ADCS_SUN_POINT', 'COMM_RX', 'PROP_OFF', 'TCS_OFF'],
            'Battery B voltage low': ['EPS_CROSS_CHARGE_ENABLE', 'ADCS_SUN_POINT', 'COMM_RX', 'PROP_OFF', 'TCS_OFF'],
            'Battery A voltage critical': ['EPS_CROSS_CHARGE_ENABLE', 'ADCS_SUN_POINT', 'COMM_RX', 'PROP_OFF', 'TCS_OFF', 'SAFE_MODE', 'EPS_POWER_CYCLE'],
            'Battery B voltage critical': ['EPS_CROSS_CHARGE_ENABLE', 'ADCS_SUN_POINT', 'COMM_RX', 'PROP_OFF', 'TCS_OFF', 'SAFE_MODE', 'EPS_POWER_CYCLE'],
            'Battery A SOC low': ['EPS_CROSS_CHARGE_ENABLE', 'ADCS_SUN_POINT', 'EPS_DEPLOY_ARRAY_A', 'EPS_DEPLOY_ARRAY_B', 'EPS_POWER_CYCLE'],
            'Battery B SOC low': ['EPS_CROSS_CHARGE_ENABLE', 'ADCS_SUN_POINT', 'EPS_DEPLOY_ARRAY_A', 'EPS_DEPLOY_ARRAY_B', 'EPS_POWER_CYCLE'],
            'Battery A SOC critical': ['EPS_CROSS_CHARGE_ENABLE', 'SAFE_MODE', 'EPS_POWER_CYCLE'],
            'Battery B SOC critical': ['EPS_CROSS_CHARGE_ENABLE', 'SAFE_MODE', 'EPS_POWER_CYCLE'],
            
            # Temperature-related
            'Flight computer temperature high': ['TCS_ON', 'TCS_DEPLOY_RADIATOR', 'ADCS_SUN_POINT', 'TCS_HEATER_AVI_OFF'],
            'Flight computer temperature critical': ['TCS_ON', 'TCS_DEPLOY_RADIATOR', 'SAFE_MODE', 'TCS_POWER_CYCLE', 'TCS_HEATER_AVI_OFF'],
            'Battery A temperature elevated': ['TCS_DEPLOY_RADIATOR', 'TCS_HEATER_EPS_OFF'],
            'Battery B temperature elevated': ['TCS_DEPLOY_RADIATOR', 'TCS_HEATER_EPS_OFF'],
            'Battery A temperature low': ['TCS_HEATER_EPS_ON'],
            'Battery B temperature low': ['TCS_HEATER_EPS_ON'],
            
            # ADCS-related (including compound tumble anomaly)
            'Roll rate high': ['ADCS_DETUMBLE', 'ADCS_DESAT', 'ADCS_ON', 'ADCS_POWER_CYCLE'],
            'Pitch rate high': ['ADCS_DETUMBLE', 'ADCS_DESAT', 'ADCS_ON', 'ADCS_POWER_CYCLE'],
            'Yaw rate high': ['ADCS_DETUMBLE', 'ADCS_DESAT', 'ADCS_ON', 'ADCS_POWER_CYCLE'],
            'Attitude rate high': ['ADCS_DETUMBLE', 'ADCS_DESAT', 'ADCS_ON', 'ADCS_POWER_CYCLE'],
            'ADCS off-pointing': ['ADCS_NADIR', 'ADCS_SUN_POINT', 'ADCS_DETUMBLE', 'ADCS_ON', 'ADCS_POWER_CYCLE'],
            'Sun pointing error moderate': ['ADCS_SUN_POINT', 'ADCS_POWER_CYCLE'],
            'Nadir pointing error moderate': ['ADCS_NADIR', 'ADCS_POWER_CYCLE'],
            
            # Communication-related (including compound COMM fault)
            'Communication signal weak': ['COMM_TX_RX', 'ADCS_NADIR', 'COMM_POWER_30', 'COMM_POWER_CYCLE'],
            
            # Propulsion-related (including compound valve stuck anomaly)
            'Propellant tank pressure high': ['PROP_VALVE_CLOSE', 'PROP_HEATER_OFF', 'PROP_OFF', 'PROP_POWER_CYCLE'],
            'Propellant tank temperature high': ['PROP_HEATER_OFF', 'TCS_HEATER_PROP_OFF', 'PROP_POWER_CYCLE'],
            'Propellant low': ['PROP_VALVE_CLOSE', 'PROP_OFF'],
            'Propellant critical': ['PROP_VALVE_CLOSE', 'PROP_OFF', 'PROP_POWER_CYCLE'],
            
            # Solar Array faults (compound solar fault anomaly)
            'Solar Array A faulted': ['EPS_CROSS_CHARGE_ENABLE', 'EPS_DEPLOY_ARRAY_A', 'EPS_POWER_CYCLE', 'EPS_MODE_LOW_POWER'],
            'Solar Array B faulted': ['EPS_CROSS_CHARGE_ENABLE', 'EPS_DEPLOY_ARRAY_B', 'EPS_POWER_CYCLE', 'EPS_MODE_LOW_POWER'],
            
            # RWS saturation
            'wheel(s) saturated': ['ADCS_DESAT', 'ADCS_RWS_DESAT', 'PROP_ON', 'PROP_STANDBY'],
            
            # Power cycle commands always valid for faulted subsystems
            'faulted': ['ADCS_POWER_CYCLE', 'EPS_POWER_CYCLE', 'COMM_POWER_CYCLE', 'PROP_POWER_CYCLE', 
                       'TCS_POWER_CYCLE', 'CDH_POWER_CYCLE', 'TTC_POWER_CYCLE', 'STAR_TRACKER_POWER_CYCLE', 
                       'GPS_POWER_CYCLE', 'AVI_POWER_CYCLE', 'AVI_REBOOT']
        }
        
        # Check if command responds to any active caution
        for caution in cautions:
            if caution in caution_response_map:
                if command in caution_response_map[caution]:
                    return True
        
        # Check if command responds to any active warning
        for warning in warnings:
            if warning in caution_response_map:
                if command in caution_response_map[warning]:
                    return True
        
        # Special cases: Power cycle commands always valid if subsystem is faulted
        if 'POWER_CYCLE' in command:
            return True
        
        # CLEAR_CAUTIONS command is always valid if cautions exist
        if command == 'CLEAR_CAUTIONS' and (cautions or warnings):
            return True
        
        # SAFE_MODE is always valid as emergency response
        # Specifically NOT a command error if 2+ warnings/cautions are active
        if command == 'SAFE_MODE':
            total_issues = len(cautions) + len(warnings)
            if total_issues >= 2:
                self.log_event("SAFE_MODE_VALID", f"SAFE_MODE command appropriate - {total_issues} active issues")
            return True
            
        return False
    
    def _monitor_cautions_warnings(self):
        """Monitor spacecraft for cautions and warnings, track when they appear and are resolved"""
        while self.is_running:
            try:
                cautions = self.spacecraft.cautions.copy()
                warnings = self.spacecraft.warnings.copy()
                current_time = time.time()
                
                # Combine cautions and warnings
                current_issues = set(cautions + warnings)
                previous_issues = set(self.active_cautions_warnings.keys())
                
                # New issues that just appeared
                new_issues = current_issues - previous_issues
                for issue in new_issues:
                    self.active_cautions_warnings[issue] = current_time
                    issue_entry = {
                        'timestamp': datetime.now().isoformat(),
                        'elapsed_seconds': current_time - self.start_time if self.start_time else 0,
                        'issue': issue,
                        'type': 'WARNING' if issue in warnings else 'CAUTION',
                        'appeared_at': current_time,
                        'resolved_at': None,
                        'response_time': None
                    }
                    self.cautions_warnings_log.append(issue_entry)
                    self.log_data['cautions_warnings'].append(issue_entry)
                    self.log_event("CAUTION_WARNING_APPEARED", f"{issue_entry['type']}: {issue}")
                
                # Issues that were resolved
                resolved_issues = previous_issues - current_issues
                for issue in resolved_issues:
                    appeared_time = self.active_cautions_warnings.pop(issue)
                    response_time = current_time - appeared_time
                    self.caution_warning_response_times.append(response_time)
                    
                    # Find the entry in log and update it
                    for entry in self.cautions_warnings_log:
                        if entry['issue'] == issue and entry['resolved_at'] is None:
                            entry['resolved_at'] = current_time
                            entry['response_time'] = response_time
                            self.log_event("CAUTION_WARNING_RESOLVED", 
                                         f"{entry['type']} resolved: {issue} (response time: {response_time:.2f}s)")
                            break
                
                time.sleep(0.5)  # Check every 0.5 seconds
            except Exception as e:
                print(f"Error in caution/warning monitoring: {e}")
                time.sleep(1)
                
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
                'off_procedure_commands': len(self.off_procedure_commands),
                'off_procedure_rate': len(self.off_procedure_commands) / max(1, self.command_count),
                'off_procedure_details': self.off_procedure_commands,
                'procedures_completed': len(self.procedure_times),
                'average_procedure_time': sum(self.procedure_times) / len(self.procedure_times) if self.procedure_times else 0,
                'procedure_step_errors': len(self.procedure_errors),
                'procedure_error_details': self.procedure_errors,
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
            
            # Add experimental fault metrics
            if self.scenario_type == 'experimental':
                if self.anomaly_injection_time and self.start_time:
                    metrics['experimental_fault_injection_time'] = self.anomaly_injection_time - self.start_time
                    
                if self.anomaly_detected_time and self.anomaly_injection_time:
                    metrics['fault_detection_delay'] = self.anomaly_detected_time - self.anomaly_injection_time
                    
                if self.anomaly_resolved_time and self.anomaly_injection_time:
                    if self.anomaly_detected_time:
                        metrics['fault_response_time'] = self.anomaly_resolved_time - self.anomaly_detected_time
                    metrics['fault_total_time'] = self.anomaly_resolved_time - self.anomaly_injection_time
                
                # Add commanding without telemetry count for TTC faults
                metrics['commanding_without_telemetry_count'] = self.spacecraft.commanding_without_telemetry_count
                metrics['experimental_fault_type'] = self.spacecraft.experimental_fault_type
                    
            self.log_data['metrics'] = metrics
            self.log_data['end_time'] = datetime.now().isoformat()
            
            # Save to consolidated JSON file (append mode)
            if self.json_consolidated_file:
                # Load existing JSON data if file exists
                existing_data = []
                if os.path.exists(self.json_consolidated_file):
                    try:
                        with open(self.json_consolidated_file, 'r') as f:
                            existing_data = json.load(f)
                            if not isinstance(existing_data, list):
                                existing_data = [existing_data]  # Convert single object to list
                    except (json.JSONDecodeError, FileNotFoundError):
                        existing_data = []
                
                # Append this run's data with unique ID
                self.log_data['run_id'] = self.run_timestamp
                existing_data.append(self.log_data)
                
                # Save consolidated JSON
                with open(self.json_consolidated_file, 'w') as f:
                    json.dump(existing_data, f, indent=2)
            
            # Export to CSV
            self._export_to_csv()
                
            self.log_event("SCENARIO_END", f"Scenario complete. Data appended to {self.json_consolidated_file}")
            
    def stop_scenario(self):
        """Stop the current scenario"""
        if self.is_running:
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
            
            if self.procedure_active and self.loaded_procedure:
                status += f"\nProcedure Active:\n"
                status += f"  Step {self.expected_step}/{len(self.loaded_procedure)}\n"
                status += f"  Errors: {len(self.procedure_errors)}\n"
                if self.expected_step < len(self.loaded_procedure):
                    status += f"  Next: {self.loaded_procedure[self.expected_step]}\n"
            
            if self.scenario_type == 'anomaly' and self.anomaly_injection_time:
                status += "\nAnomaly: ACTIVE\n"
                if self.anomaly_detected_time:
                    status += "Status: DETECTED\n"
                if self.anomaly_resolved_time:
                    status += "Status: RESOLVED\n"
                    
            return status
    
    def _export_to_csv(self):
        """Export experiment data to consolidated CSV file"""
        import csv
        
        if not self.csv_consolidated_file:
            return
        
        with self.lock:
            # Prepare CSV data
            metrics = self.log_data.get('metrics', {})
            
            # Calculate summary statistics
            total_cautions_warnings = len(self.cautions_warnings_log)
            cautions_warnings_text = "; ".join([f"{cw['type']}: {cw['issue']}" for cw in self.cautions_warnings_log])
            
            # Calculate average response time for cautions/warnings
            avg_response_time = None
            if self.caution_warning_response_times:
                avg_response_time = sum(self.caution_warning_response_times) / len(self.caution_warning_response_times)
            
            # Get anomaly information
            anomaly_type = self.injected_anomaly_type if self.injected_anomaly_type else "None"
            anomaly_injection_time = None
            anomaly_resolution_time = None
            
            if self.scenario_type == 'anomaly':
                if self.anomaly_injection_time and self.start_time:
                    anomaly_injection_time = self.anomaly_injection_time - self.start_time
                if self.anomaly_resolved_time and self.anomaly_injection_time:
                    anomaly_resolution_time = self.anomaly_resolved_time - self.anomaly_injection_time
            elif self.scenario_type == 'experimental':
                if self.anomaly_injection_time and self.start_time:
                    anomaly_injection_time = self.anomaly_injection_time - self.start_time
                if self.anomaly_resolved_time and self.anomaly_injection_time:
                    anomaly_resolution_time = self.anomaly_resolved_time - self.anomaly_injection_time
            
            # Prepare row data
            row_data = {
                'Run ID': self.run_timestamp,
                'Subject Number': self.subject_number,
                'Run Type': self.run_type,
                'Scenario Type': self.scenario_type,
                'Start Time': self.log_data.get('start_time', ''),
                'End Time': self.log_data.get('end_time', ''),
                'Total Duration (s)': f"{metrics.get('total_duration_seconds', 0):.2f}",
                'Procedure Completion Time (s)': f"{self.procedure_completion_time:.2f}" if self.procedure_completion_time else 'N/A',
                'Total Commands': metrics.get('total_commands', 0),
                'Command Errors': metrics.get('command_errors', 0),
                'Off-Procedure Commands': metrics.get('off_procedure_commands', 0),
                'Total Cautions/Warnings': total_cautions_warnings,
                'Cautions/Warnings List': cautions_warnings_text,
                'Avg Caution/Warning Response Time (s)': f"{avg_response_time:.2f}" if avg_response_time else 'N/A',
                'Anomaly Injected': anomaly_type,
                'Anomaly Injection Time (s)': f"{anomaly_injection_time:.2f}" if anomaly_injection_time else 'N/A',
                'Anomaly Resolution Time (s)': f"{anomaly_resolution_time:.2f}" if anomaly_resolution_time else 'N/A',
                'Procedure Step Errors': metrics.get('procedure_step_errors', 0),
            }
            
            # Check if CSV file exists to determine if we need to write headers
            file_exists = os.path.exists(self.csv_consolidated_file)
            
            # Append to CSV
            with open(self.csv_consolidated_file, 'a', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=row_data.keys())
                
                if not file_exists:
                    writer.writeheader()
                
                writer.writerow(row_data)
            
            print(f"CSV data appended to: {self.csv_consolidated_file}")


# Global experiment manager instance
experiment_manager = None


def initialize_experiment_manager(spacecraft):
    """Initialize the global experiment manager"""
    global experiment_manager
    experiment_manager = ExperimentManager(spacecraft)
    return experiment_manager
