"""
Spacecraft Telemetry Generator
Realistic 6U CubeSat Simulation at 520km LEO

Simulates a 6U CubeSat (10cm × 20cm × 30cm, 12kg) in a 520km circular Low Earth Orbit
with 8 subsystems:
- AVI: Avionics (main computer, flight software)
- CDH: Command & Data Handling (memory, storage, processing)
- TT&C: Tracking, Telemetry & Command (ground communication link)
- PROP: Propulsion (cold gas thrusters for orbital maneuvers)
- ADCS: Attitude Determination & Control System (orientation control)
- EPS: Electrical Power System (solar panels, battery, power distribution)
- TCS: Thermal Control System (heaters, radiators)
- COMM: Communications (S-band transceiver)
"""
import math
import time
import json
from datetime import datetime
from threading import Thread, RLock
import random

class SpacecraftSimulator:
    def __init__(self):
        self.lock = RLock()
        self.running = False
        
        # 6U CubeSat Orbital parameters (520 km altitude, 51.6° inclination)
        self.orbital_altitude = 520.0  # km
        self.orbital_period = 94.6  # minutes (520km circular orbit)
        self.orbital_inclination = 51.6  # degrees (ISS-like)
        self.epoch_time = time.time()
        
        # 6U CubeSat physical specifications
        # Dimensions: 10cm × 20cm × 30cm (2U × 3U)
        # Mass: ~12 kg with payloads
        self.spacecraft_mass = 12.0  # kg
        self.solar_panel_area = 0.12  # m² (6 deployable panels ~0.02 m² each)
        self.battery_capacity = 100.0  # Wh (Li-ion 11.1V, ~9 Ah)
        
        # === SUBSYSTEM STATES ===
        
        # AVI: Avionics subsystem (main flight computer)
        self.avi_state = {
            'cpu_temp': 25.0,      # °C
            'cpu_load': 35.0,      # %
            'memory_usage': 45.0,  # %
            'uptime': 0.0,         # hours
            'boot_count': 1,
            'watchdog_ok': True,
        }
        
        # CDH: Command & Data Handling
        self.cdh_state = {
            'cmd_count': 0,        # commands received
            'tlm_count': 0,        # telemetry packets sent
            'storage_used': 2.5,   # GB
            'storage_total': 32.0, # GB
            'error_count': 0,
            'last_cmd_time': 0.0,  # seconds ago
        }
        
        # TT&C: Tracking, Telemetry & Command
        self.ttc_state = {
            'link_status': 'NO_LINK',  # NO_LINK, ACQUIRING, LOCKED
            'signal_strength': -95.0,   # dBm
            'uplink_rate': 0.0,         # kbps
            'downlink_rate': 0.0,       # kbps
            'pass_elevation': 0.0,      # degrees
            'doppler_shift': 0.0,       # Hz
        }
        
        # PROP: Propulsion (cold gas thrusters)
        self.prop_state = {
            'system_enabled': False,
            'valve_open': False,
            'tank_pressure': 200.0,  # bar (cold gas N2)
            'tank_temp': 20.0,       # °C
            'propellant_mass': 0.5,  # kg remaining
            'total_delta_v': 0.0,    # m/s used
        }
        
        # ADCS: Attitude Determination & Control
        self.adcs_state = {
            'mode': 'DETUMBLE',      # DETUMBLE, NADIR, SUN_POINT, INERTIAL, OFF
            'enabled': False,
            'roll': 0.0,             # degrees
            'pitch': 0.0,            # degrees
            'yaw': 0.0,              # degrees
            'roll_rate': 0.0,        # deg/s
            'pitch_rate': 0.0,       # deg/s
            'yaw_rate': 0.0,         # deg/s
            'mag_field_x': 0.0,      # μT (magnetometer)
            'mag_field_y': 0.0,      # μT
            'mag_field_z': 0.0,      # μT
            'sun_sensor_x': 0.0,     # degrees
            'sun_sensor_y': 0.0,     # degrees
        }
        
        # EPS: Electrical Power System
        self.eps_state = {
            'mode': 'NOMINAL',       # SAFE, LOW_POWER, NOMINAL, SCIENCE
            'battery_voltage': 11.8, # V (11.1V nominal Li-ion, 3S)
            'battery_current': -0.5, # A (negative = charging)
            'battery_soc': 85.0,     # % state of charge
            'battery_temp': 15.0,    # °C
            'solar_voltage': 12.4,   # V
            'solar_current': 1.2,    # A
            'solar_power': 14.9,     # W
            'bus_voltage': 11.8,     # V (main power bus)
            'bus_current': 0.7,      # A (total load)
            'panel_temp_x_plus': 10.0,  # °C
            'panel_temp_x_minus': 10.0,
            'panel_temp_y_plus': 10.0,
            'panel_temp_y_minus': 10.0,
            'panel_temp_z_plus': 10.0,
            'panel_temp_z_minus': 10.0,
        }
        
        # TCS: Thermal Control System
        self.tcs_state = {
            'heater_avi_on': False,
            'heater_eps_on': False,
            'temp_avi': 25.0,        # °C
            'temp_eps': 15.0,        # °C
            'temp_external': -20.0,  # °C (radiator)
            'radiator_coating': 'GOOD',  # GOOD, DEGRADED
        }
        
        # COMM: Communications (S-band transceiver)
        self.comm_state = {
            'mode': 'STANDBY',       # TX, RX, STANDBY, OFF
            'tx_power': 30.0,        # dBm (1W)
            'frequency': 2200.0,     # MHz (S-band)
            'data_rate': 9.6,        # kbps
            'temp': 30.0,            # °C
            'packets_tx': 0,
            'packets_rx': 0,
        }
        
        # Orbit state
        self.orbit_state = {
            'altitude': 520.0,       # km
            'velocity': 7.613,       # km/s (520km circular orbit)
            'latitude': 0.0,         # degrees
            'longitude': 0.0,        # degrees
            'eclipse': False,        # True when in Earth's shadow
            'sun_angle': 45.0,       # degrees (solar illumination angle)
        }
        
        # Simulation parameters
        self.sim_rate = 1.0  # 1.0 = real-time, >1 = faster
        self.last_update = time.time()
        
        # Thresholds for telemetry health monitoring
        self.thresholds = {
            # EPS thresholds
            'battery_voltage_min': 10.2,      # V (3.4V per cell minimum)
            'battery_voltage_caution': 10.8,  # V (3.6V per cell)
            'battery_voltage_nominal': 11.1,  # V (3.7V per cell)
            'battery_voltage_max': 12.6,      # V (4.2V per cell max)
            'battery_soc_low': 20.0,          # %
            'battery_soc_caution': 40.0,      # %
            'battery_temp_min': -10.0,        # °C
            'battery_temp_max': 45.0,         # °C
            'solar_power_nominal': 15.0,      # W (in sunlight)
            
            # Thermal thresholds
            'temp_avi_min': 0.0,              # °C
            'temp_avi_max': 50.0,             # °C
            'temp_eps_min': -10.0,            # °C
            'temp_eps_max': 45.0,             # °C
            'temp_comm_max': 60.0,            # °C
            
            # ADCS thresholds
            'attitude_pointing_good': 1.0,    # degrees
            'attitude_pointing_caution': 5.0, # degrees
            'attitude_pointing_warning': 10.0, # degrees
            'rate_detumbled': 0.1,            # deg/s
            
            # Propulsion thresholds
            'prop_pressure_min': 50.0,        # bar
            'prop_pressure_nominal': 200.0,   # bar
            
            # CDH thresholds
            'storage_caution': 80.0,          # % used
            'storage_critical': 95.0,         # % used
        }
        
        # Operational modes
        self.operational_mode = "NOMINAL"  # NOMINAL, SAFE, LOW_POWER, SCIENCE
        
        # Event records
        self.evr_log = []
        
        # Caution and warning flags
        self.cautions = []
        self.warnings = []
        
        # Anomaly states (persistent until cleared)
        self.anomaly_states = {
            'battery_low': False,
            'attitude_deviation': False,
            'eps_overtemp': False,
            'avi_overtemp': False,
            'comm_overtemp': False,
            'storage_full': False,
        }
        
        # Command queue
        self.command_queue = []
        
        # Command tracking for experiments
        self.command_counter = 0
        
    def start(self):
        """Start the simulation"""
        self.running = True
        self.sim_thread = Thread(target=self._simulate, daemon=True)
        self.sim_thread.start()
        self.add_evr("Spacecraft telemetry generator started")
        
    def stop(self):
        """Stop the simulation"""
        self.running = False
        self.add_evr("Spacecraft telemetry generator stopped")
        
    def _simulate(self):
        """Main simulation loop"""
        while self.running:
            elapsed_time = time.time() - self.epoch_time
            self._update_orbital_position(elapsed_time)
            self._update_subsystems(elapsed_time)
            self._check_cautions_warnings()
            self._process_commands()
            time.sleep(1.0)  # Update at 1 Hz
            
    def _update_orbital_position(self, elapsed_time):
        """Update spacecraft orbital position using ISS parameters"""
        with self.lock:
            # Calculate position in orbit (simplified circular orbit)
            orbit_fraction = (elapsed_time / 60.0) / self.orbital_period
            angle = 2 * math.pi * orbit_fraction
            
            # Update latitude/longitude with ISS inclination
            self.telemetry['latitude'] = self.orbital_inclination * math.sin(angle)
            self.telemetry['longitude'] = (angle * 180 / math.pi) % 360 - 180
            
            # Add small variations to altitude and velocity (atmospheric drag effects)
            self.telemetry['altitude'] = self.orbital_altitude + random.uniform(-0.3, 0.3)
            self.telemetry['velocity'] = 7.66 + random.uniform(-0.005, 0.005)
            
    def _update_subsystems(self, elapsed_time):
        """Update 6U CubeSat subsystems"""
        with self.lock:
            # Attitude control with target tracking
            # Calculate error from target attitude
            roll_error = self.target_attitude['roll'] - self.telemetry['attitude_roll']
            pitch_error = self.target_attitude['pitch'] - self.telemetry['attitude_pitch']
            yaw_error = self.target_attitude['yaw'] - self.telemetry['attitude_yaw']
            
            # Attitude control with damping
            if self.anomaly_states['attitude_deviation']:
                # Anomaly: poor control, more drift
                self.telemetry['attitude_roll'] += random.uniform(-0.5, 0.5)
                self.telemetry['attitude_pitch'] += random.uniform(-0.5, 0.5)
                self.telemetry['attitude_yaw'] += random.uniform(-0.5, 0.5)
            else:
                # Normal: move toward target with damping
                self.telemetry['attitude_roll'] += roll_error * 0.1 + random.uniform(-0.1, 0.1)
                self.telemetry['attitude_pitch'] += pitch_error * 0.1 + random.uniform(-0.1, 0.1)
                self.telemetry['attitude_yaw'] += yaw_error * 0.1 + random.uniform(-0.1, 0.1)
                    
            # Power system (6U CubeSat with ~18W solar panels)
            orbit_fraction = (elapsed_time / 60.0) / self.orbital_period
            sun_angle = 2 * math.pi * orbit_fraction
            sun_exposure = max(0, math.cos(sun_angle))
            
            # Solar array current (max ~1.5A at 12V for 6U)
            max_solar_current = 1.5 if self.attitude_mode == "SUN_POINTING" else 1.2
            self.telemetry['solar_array_current'] = max_solar_current * sun_exposure + random.uniform(-0.05, 0.05)
            self.telemetry['solar_array_voltage'] = 12.6 if sun_exposure > 0 else 0.0
            
            # Power calculations
            self.telemetry['power_generation'] = self.telemetry['solar_array_current'] * self.telemetry['solar_array_voltage']
            self.telemetry['power_consumption'] = self.telemetry['battery_current'] * self.telemetry['battery_voltage']
            
            # Battery management (12V system, 10.8V cutoff)
            # If battery anomaly is active, drain faster
            drain_rate = 0.02 if self.anomaly_states['battery_low'] else 0.01
            charge_rate = 0.015
            
            if self.telemetry['power_generation'] > self.telemetry['power_consumption']:
                self.telemetry['battery_voltage'] = min(12.6, self.telemetry['battery_voltage'] + charge_rate)
            else:
                self.telemetry['battery_voltage'] = max(10.5, self.telemetry['battery_voltage'] - drain_rate)
                
            # Battery current varies with mode
            if self.operational_mode == "LOW_POWER":
                self.telemetry['battery_current'] = 0.4 + random.uniform(-0.05, 0.05)
            elif self.operational_mode == "SCIENCE":
                self.telemetry['battery_current'] = 1.2 + random.uniform(-0.1, 0.1)
            else:  # NOMINAL or SAFE
                self.telemetry['battery_current'] = 0.8 + random.uniform(-0.05, 0.05)
                
            # Thermal management (6U CubeSat temps)
            # External temperature varies with sun exposure
            if sun_exposure > 0.5:
                target_ext_temp = -20.0  # Sun side
            else:
                target_ext_temp = -80.0  # Eclipse
                
            temp_delta = target_ext_temp - self.telemetry['temperature_external']
            self.telemetry['temperature_external'] += temp_delta * 0.1
            
            # Internal temperature affected by power dissipation and external temp
            base_internal_temp = 20.0
            power_heating = (self.telemetry['power_consumption'] - 5.0) * 0.5  # Heating from electronics
            
            if self.anomaly_states['temperature_high']:
                self.telemetry['temperature_internal'] = min(45, self.telemetry['temperature_internal'] + 0.3)
            elif self.anomaly_states['temperature_low']:
                self.telemetry['temperature_internal'] = max(-5, self.telemetry['temperature_internal'] - 0.3)
            else:
                target_temp = base_internal_temp + power_heating
                self.telemetry['temperature_internal'] += (target_temp - self.telemetry['temperature_internal']) * 0.1
            
            # Battery and CPU temperatures
            self.telemetry['temperature_battery'] = self.telemetry['temperature_internal'] - 2.0 + random.uniform(-1, 1)
            self.telemetry['temperature_cpu'] = self.telemetry['temperature_internal'] + 5.0 + random.uniform(-2, 2)
            
            # Communication signal strength (affected by attitude and position)
            if self.anomaly_states['comm_degraded']:
                self.telemetry['communication_signal'] = max(50, self.telemetry['communication_signal'] - 1.0)
            else:
                # Signal varies with elevation above ground station (simplified)
                base_signal = 90.0 + abs(self.telemetry['latitude']) * 0.1  # Higher at poles
                self.telemetry['communication_signal'] = min(99, base_signal + random.uniform(-3.0, 3.0))
            
            # Data rate varies with signal strength
            if self.telemetry['communication_signal'] > 85:
                self.telemetry['data_rate'] = 9600
            elif self.telemetry['communication_signal'] > 70:
                self.telemetry['data_rate'] = 4800
            else:
                self.telemetry['data_rate'] = 1200
            
    def _check_cautions_warnings(self):
        """Check for caution and warning conditions using configurable thresholds"""
        with self.lock:
            self.cautions.clear()
            self.warnings.clear()
            
            # Battery voltage warnings (12V system)
            if self.telemetry['battery_voltage'] < self.thresholds['battery_voltage_min']:
                self.warnings.append("LOW BATTERY VOLTAGE - CRITICAL")
                self.anomaly_states['battery_low'] = True
            elif self.telemetry['battery_voltage'] < self.thresholds['battery_voltage_caution']:
                self.cautions.append("Battery voltage low")
                self.anomaly_states['battery_low'] = True
                
            # Attitude warnings (configurable thresholds)
            attitude_anomaly = False
            for axis in ['attitude_roll', 'attitude_pitch', 'attitude_yaw']:
                if abs(self.telemetry[axis]) > self.thresholds['attitude_warning']:
                    self.warnings.append(f"ATTITUDE LIMIT: {axis.split('_')[1].upper()}")
                    attitude_anomaly = True
                elif abs(self.telemetry[axis]) > self.thresholds['attitude_caution']:
                    self.cautions.append(f"Attitude deviation: {axis.split('_')[1]}")
                    attitude_anomaly = True
            
            if attitude_anomaly:
                self.anomaly_states['attitude_deviation'] = True
                    
            # Temperature warnings (configurable)
            if self.telemetry['temperature_internal'] > self.thresholds['temperature_max']:
                self.warnings.append("HIGH INTERNAL TEMPERATURE")
                self.anomaly_states['temperature_high'] = True
            elif self.telemetry['temperature_internal'] > self.thresholds['temperature_caution_high']:
                self.cautions.append("Internal temperature elevated")
                self.anomaly_states['temperature_high'] = True
            
            if self.telemetry['temperature_internal'] < self.thresholds['temperature_min']:
                self.warnings.append("LOW INTERNAL TEMPERATURE")
                self.anomaly_states['temperature_low'] = True
            elif self.telemetry['temperature_internal'] < self.thresholds['temperature_caution_low']:
                self.cautions.append("Internal temperature low")
                self.anomaly_states['temperature_low'] = True
                
            # Communication signal
            if self.telemetry['communication_signal'] < 60.0:
                self.warnings.append("COMMUNICATION SIGNAL CRITICAL")
                self.anomaly_states['comm_degraded'] = True
            elif self.telemetry['communication_signal'] < 75.0:
                self.cautions.append("Communication signal weak")
                self.anomaly_states['comm_degraded'] = True
                
    def _process_commands(self):
        """Process queued commands"""
        with self.lock:
            while self.command_queue:
                cmd = self.command_queue.pop(0)
                self.command_counter += 1
                self.add_evr(f"CMD RECV [{self.command_counter}]: {cmd}")
                # Radiate to spacecraft regardless of success
                self.add_evr(f"CMD RADIATE [{self.command_counter}]: Command radiated to spacecraft")
                success = self._execute_command(cmd)
                if success:
                    self.add_evr(f"CMD EXEC [{self.command_counter}]: Executed successfully")
                else:
                    self.add_evr(f"CMD ERROR [{self.command_counter}]: Command failed")
                
    def _execute_command(self, cmd):
        """Execute a specific command - supports standard satellite command syntax
        Returns True if successful, False if error"""
        cmd_upper = cmd.upper().strip()
        
        # === NOMINAL OPERATIONS COMMANDS ===
        
        # Attitude control commands
        if cmd_upper.startswith("ATT_"):
            if "ATT_NADIR" in cmd_upper:
                self.attitude_mode = "NADIR"
                self.target_attitude = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
                self.add_evr("CMD: Attitude mode set to NADIR pointing")
                
            elif "ATT_SUN_POINT" in cmd_upper:
                self.attitude_mode = "SUN_POINTING"
                self.target_attitude = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
                self.add_evr("CMD: Attitude mode set to SUN POINTING")
                
            elif "ATT_INERTIAL" in cmd_upper:
                self.attitude_mode = "INERTIAL"
                self.add_evr("CMD: Attitude mode set to INERTIAL hold")
                
            elif "ATT_SLEW" in cmd_upper:
                # Parse: ATT_SLEW <roll> <pitch> <yaw>
                try:
                    parts = cmd_upper.split()
                    if len(parts) >= 4:
                        roll = float(parts[1])
                        pitch = float(parts[2])
                        yaw = float(parts[3])
                        self.target_attitude = {'roll': roll, 'pitch': pitch, 'yaw': yaw}
                        self.attitude_mode = "TARGET"
                        self.add_evr(f"CMD: Slewing to attitude R={roll}° P={pitch}° Y={yaw}°")
                    else:
                        self.add_evr("ERROR: Invalid ATT_SLEW syntax. Use: ATT_SLEW <roll> <pitch> <yaw>")
                        return False
                except:
                    self.add_evr("ERROR: Invalid ATT_SLEW syntax. Use: ATT_SLEW <roll> <pitch> <yaw>")
                    return False
        
        # Mode change commands
        elif "MODE_" in cmd_upper:
            if "MODE_NOMINAL" in cmd_upper:
                self.operational_mode = "NOMINAL"
                self.add_evr("CMD: Operational mode changed to NOMINAL")
                
            elif "MODE_SCIENCE" in cmd_upper:
                self.operational_mode = "SCIENCE"
                self.add_evr("CMD: Operational mode changed to SCIENCE (high power)")
                
            elif "MODE_LOW_POWER" in cmd_upper:
                self.operational_mode = "LOW_POWER"
                self.add_evr("CMD: Operational mode changed to LOW POWER")
                
        # Threshold adjustment commands
        elif "THR_BATT_MIN" in cmd_upper:
            # Parse: THR_BATT_MIN <voltage>
            try:
                parts = cmd_upper.split()
                if len(parts) >= 2:
                    new_min = float(parts[1])
                    self.thresholds['battery_voltage_min'] = new_min
                    self.add_evr(f"CMD: Battery minimum threshold set to {new_min}V")
                else:
                    self.add_evr("ERROR: Invalid THR_BATT_MIN syntax. Use: THR_BATT_MIN <voltage>")
                    return False
            except:
                self.add_evr("ERROR: Invalid THR_BATT_MIN syntax. Use: THR_BATT_MIN <voltage>")
                return False
                
        elif "THR_BATT_CAUT" in cmd_upper:
            try:
                parts = cmd_upper.split()
                if len(parts) >= 2:
                    new_caut = float(parts[1])
                    self.thresholds['battery_voltage_caution'] = new_caut
                    self.add_evr(f"CMD: Battery caution threshold set to {new_caut}V")
                else:
                    self.add_evr("ERROR: Invalid THR_BATT_CAUT syntax. Use: THR_BATT_CAUT <voltage>")
                    return False
            except:
                self.add_evr("ERROR: Invalid THR_BATT_CAUT syntax. Use: THR_BATT_CAUT <voltage>")
                return False
                
        elif "THR_TEMP_MIN" in cmd_upper:
            try:
                parts = cmd_upper.split()
                if len(parts) >= 2:
                    new_min = float(parts[1])
                    self.thresholds['temperature_min'] = new_min
                    self.add_evr(f"CMD: Temperature minimum threshold set to {new_min}°C")
                else:
                    self.add_evr("ERROR: Invalid THR_TEMP_MIN syntax")
                    return False
            except:
                self.add_evr("ERROR: Invalid THR_TEMP_MIN syntax")
                return False
                
        elif "THR_TEMP_MAX" in cmd_upper:
            try:
                parts = cmd_upper.split()
                if len(parts) >= 2:
                    new_max = float(parts[1])
                    self.thresholds['temperature_max'] = new_max
                    self.add_evr(f"CMD: Temperature maximum threshold set to {new_max}°C")
                else:
                    self.add_evr("ERROR: Invalid THR_TEMP_MAX syntax")
                    return False
            except:
                self.add_evr("ERROR: Invalid THR_TEMP_MAX syntax")
                return False
                
        elif "THR_ATT_CAUT" in cmd_upper:
            try:
                parts = cmd_upper.split()
                if len(parts) >= 2:
                    new_caut = float(parts[1])
                    self.thresholds['attitude_caution'] = new_caut
                    self.add_evr(f"CMD: Attitude caution threshold set to {new_caut}°")
                else:
                    self.add_evr("ERROR: Invalid THR_ATT_CAUT syntax")
                    return False
            except:
                self.add_evr("ERROR: Invalid THR_ATT_CAUT syntax")
                return False
                
        elif "THR_ATT_WARN" in cmd_upper:
            try:
                parts = cmd_upper.split()
                if len(parts) >= 2:
                    new_warn = float(parts[1])
                    self.thresholds['attitude_warning'] = new_warn
                    self.add_evr(f"CMD: Attitude warning threshold set to {new_warn}°")
                else:
                    self.add_evr("ERROR: Invalid THR_ATT_WARN syntax")
                    return False
            except:
                self.add_evr("ERROR: Invalid THR_ATT_WARN syntax")
                return False
        
        # === ANOMALY RESPONSE COMMANDS ===
        
        elif "SAFE_MODE" in cmd_upper:
            self._enter_safe_mode()
            # valid command
            
            
        elif "PWR_RESET" in cmd_upper:
            self.telemetry['battery_voltage'] = 12.6
            self.telemetry['battery_current'] = 0.8
            self.anomaly_states['battery_low'] = False
            self.add_evr("RESPONSE: Power system reset - battery restored to nominal")
            
        elif "ATT_RESET" in cmd_upper:
            self.telemetry['attitude_roll'] = 0.0
            self.telemetry['attitude_pitch'] = 0.0
            self.telemetry['attitude_yaw'] = 0.0
            self.target_attitude = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
            self.anomaly_states['attitude_deviation'] = False
            self.add_evr("RESPONSE: Attitude reset complete - nominal orientation restored")
            
        elif "THERM_RESET" in cmd_upper:
            self.telemetry['temperature_internal'] = 20.0
            self.anomaly_states['temperature_high'] = False
            self.anomaly_states['temperature_low'] = False
            self.add_evr("RESPONSE: Thermal system reset - temperature nominal")
            
        elif "COMM_RESET" in cmd_upper:
            self.telemetry['communication_signal'] = 95.0
            self.anomaly_states['comm_degraded'] = False
            self.add_evr("RESPONSE: Communication system reset - signal restored")
            
        elif "CLEAR_CAUTIONS" in cmd_upper or "CLEAR_CAUT" in cmd_upper:
            self._clear_all_cautions()
            self.add_evr("RESPONSE: All cautions cleared by operator")
            
        elif "BATT_CHARGE" in cmd_upper:
            self.telemetry['battery_voltage'] = min(12.6, self.telemetry['battery_voltage'] + 1.0)
            self.anomaly_states['battery_low'] = False
            self.add_evr("RESPONSE: Battery charge mode activated")
            
        elif "EMERG_SHUTDOWN" in cmd_upper:
            self.operational_mode = "LOW_POWER"
            self.telemetry['battery_current'] = 0.4
            self.add_evr("RESPONSE: EMERGENCY SHUTDOWN - Non-essential systems powered down")
            
        elif "DIAG_MODE" in cmd_upper:
            self.add_evr("RESPONSE: Diagnostic mode activated - running system tests")
            
        # Quick command shortcuts
        elif cmd_upper in ["CMD1", "CMD 1"]:
            self.attitude_mode = "NADIR"
            self.target_attitude = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
            self.add_evr("Quick CMD1: Nadir pointing")
        elif cmd_upper in ["CMD2", "CMD 2"]:
            self.attitude_mode = "SUN_POINTING"
            self.add_evr("Quick CMD2: Sun pointing")
        elif cmd_upper in ["CMD3", "CMD 3"]:
            self.operational_mode = "SCIENCE"
            self.add_evr("Quick CMD3: Science mode")
        elif cmd_upper in ["CMD4", "CMD 4"]:
            self.operational_mode = "LOW_POWER"
            self.add_evr("Quick CMD4: Low power mode")
        elif cmd_upper in ["CMD5", "CMD 5"]:
            self._clear_all_cautions()
            self.add_evr("Quick CMD5: Clear cautions")
        elif cmd_upper in ["CMD6", "CMD 6"]:
            self.add_evr("Quick CMD6: System health check nominal")
        elif cmd_upper in ["CMD7", "CMD 7"]:
            self.telemetry['data_rate'] = 9600
            self.add_evr("Quick CMD7: Comm rate set to 9600 bps")
        elif cmd_upper in ["CMD8", "CMD 8"]:
            self.add_evr("Quick CMD8: Watchdog timer reset")
            
        # === EXPERIMENT ANOMALY INJECTION COMMANDS ===
        
        elif "INJECT_ANOMALY_BATTERY" in cmd_upper:
            self.anomaly_states['battery_low'] = True
            self.telemetry['battery_voltage'] = 10.5  # Below minimum
            self.add_evr("ANOMALY INJECTED: Battery voltage critical")
            
        elif "INJECT_ANOMALY_ATTITUDE" in cmd_upper:
            self.anomaly_states['attitude_deviation'] = True
            self.telemetry['attitude_roll'] = 12.0  # Exceeds warning threshold
            self.add_evr("ANOMALY INJECTED: Attitude control error - off-pointing")
            
        elif "INJECT_ANOMALY_THERMAL" in cmd_upper:
            self.anomaly_states['temperature_high'] = True
            self.telemetry['temperature_internal'] = 45.0  # Above maximum
            self.add_evr("ANOMALY INJECTED: Thermal warning - temperature high")
            
        elif "INJECT_ANOMALY_COMM" in cmd_upper:
            self.anomaly_states['comm_degraded'] = True
            self.telemetry['communication_signal'] = 65.0  # Below threshold
            self.add_evr("ANOMALY INJECTED: Communication signal degraded")
            
        elif "INJECT_ANOMALY_POWER" in cmd_upper:
            self.anomaly_states['battery_low'] = True
            self.telemetry['battery_current'] = -2.0  # High discharge
            self.add_evr("ANOMALY INJECTED: Power system fault - high discharge")
            
        else:
            # Unknown command: radiated but not executed successfully
            self.add_evr(f"CMD UNKNOWN: {cmd}")
            return False
        
        return True  # Command executed successfully for recognized commands
            
    def _enter_safe_mode(self):
        """Enter safe mode - reset all systems to nominal (6U CubeSat)"""
        self.operational_mode = "SAFE"
        self.attitude_mode = "SUN_POINTING"  # Safe mode = sun pointing for power
        self.telemetry['attitude_roll'] = 0.0
        self.telemetry['attitude_pitch'] = 0.0
        self.telemetry['attitude_yaw'] = 0.0
        self.target_attitude = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.telemetry['battery_voltage'] = max(self.telemetry['battery_voltage'], 11.5)
        self.telemetry['battery_current'] = 0.5  # Minimal power consumption
        self.telemetry['temperature_internal'] = 20.0
        self.telemetry['communication_signal'] = 95.0
        
        # Clear all anomalies
        for key in self.anomaly_states:
            self.anomaly_states[key] = False
            
        self.add_evr("SAFE MODE ENGAGED - All systems reset to safe configuration")
        
    def _clear_all_cautions(self):
        """Clear all caution states (user-initiated)"""
        # Only clear cautions if parameters are now nominal
        if self.telemetry['battery_voltage'] >= self.thresholds['battery_voltage_caution']:
            self.anomaly_states['battery_low'] = False
            
        if (abs(self.telemetry['attitude_roll']) < self.thresholds['attitude_caution'] and 
            abs(self.telemetry['attitude_pitch']) < self.thresholds['attitude_caution'] and 
            abs(self.telemetry['attitude_yaw']) < self.thresholds['attitude_caution']):
            self.anomaly_states['attitude_deviation'] = False
            
        if (self.thresholds['temperature_caution_low'] <= self.telemetry['temperature_internal'] <= 
            self.thresholds['temperature_caution_high']):
            self.anomaly_states['temperature_high'] = False
            self.anomaly_states['temperature_low'] = False
            
        if self.telemetry['communication_signal'] >= 75.0:
            self.anomaly_states['comm_degraded'] = False
                
    def add_evr(self, message):
        """Add an event record"""
        with self.lock:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            evr_entry = f"{timestamp} - {message}"
            self.evr_log.append(evr_entry)
            if len(self.evr_log) > 100:  # Keep last 100 entries
                self.evr_log.pop(0)
                
    def send_command(self, command):
        """Queue a command"""
        with self.lock:
            self.command_queue.append(command)
            
    def get_telemetry(self):
        """Get current telemetry snapshot"""
        with self.lock:
            return self.telemetry.copy()
            
    def get_evr_log(self):
        """Get EVR log"""
        with self.lock:
            return self.evr_log.copy()
            
    def get_cautions_warnings(self):
        """Get current cautions and warnings"""
        with self.lock:
            return self.cautions.copy(), self.warnings.copy()
            
    def get_telemetry_channels(self):
        """Get telemetry formatted as channels"""
        with self.lock:
            channels = []
            for key, value in self.telemetry.items():
                channel_name = key.replace('_', ' ').title()
                if isinstance(value, float):
                    data = f"{value:.2f}"
                else:
                    data = str(value)
                channels.append((channel_name, data))
            return channels


# Global simulator instance
spacecraft = SpacecraftSimulator()

if __name__ == "__main__":
    # Test the simulator
    spacecraft.start()
    print("Spacecraft simulator started. Press Ctrl+C to stop.")
    try:
        while True:
            time.sleep(1)
            telemetry = spacecraft.get_telemetry()
            print(f"\nAltitude: {telemetry['altitude']:.2f} km")
            print(f"Velocity: {telemetry['velocity']:.2f} km/s")
            print(f"Position: {telemetry['latitude']:.2f}°, {telemetry['longitude']:.2f}°")
            print(f"Battery: {telemetry['battery_voltage']:.2f} V")
            
            cautions, warnings = spacecraft.get_cautions_warnings()
            if warnings:
                print(f"WARNINGS: {', '.join(warnings)}")
            if cautions:
                print(f"Cautions: {', '.join(cautions)}")
    except KeyboardInterrupt:
        spacecraft.stop()
        print("\nSimulator stopped.")
