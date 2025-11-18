"""
Spacecraft Telemetry Generator
Realistic 6U CubeSat Simulation at 520km LEO

Simulates a 6U CubeSat (10cm x 20cm x 30cm, 12kg) in a 520km circular Low Earth Orbit
with 8 subsystems: AVI, CDH, TT&C, PROP, ADCS, EPS, TCS, COMM
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
        
        # 6U CubeSat Orbital parameters (520 km altitude)
        self.orbital_altitude = 520.0  # km
        self.orbital_period = 94.6  # minutes
        self.orbital_inclination = 51.6  # degrees
        self.epoch_time = time.time()
        
        # Physical specifications
        self.spacecraft_mass = 12.0  # kg
        self.solar_panel_area = 0.12  # m²
        self.battery_capacity = 100.0  # Wh
        
        # === SUBSYSTEM STATES ===
        
        # AVI: Avionics
        self.avi_state = {
            'cpu_temp': 25.0,
            'cpu_load': 35.0,
            'memory_usage': 45.0,
            'memory_total': 512.0,  # MB
            'memory_used': 230.0,   # MB
            'boot_count': 1,
            'uptime': 0.0,  # seconds
            'watchdog_ok': True,
        }
        
        # Thresholds
        self.thresholds = {
            # EPS (Electrical Power System)
            'battery_voltage_min_warn': 10.2,
            'battery_voltage_min_caution': 10.5,  # 10% margin above critical
            'battery_voltage_max_warn': 12.6,
            'battery_voltage_max_caution': 12.4,  # 10% margin below max
            'battery_soc_min_warn': 15.0,
            'battery_soc_min_caution': 20.0,
            'battery_temp_min_warn': -15.0,
            'battery_temp_min_caution': -10.0,
            'battery_temp_max_warn': 50.0,
            'battery_temp_max_caution': 45.0,  # 10% margin
            'battery_current_max_warn': 3.0,
            'battery_current_max_caution': 2.7,
            'solar_voltage_min_caution': 11.5,
            'solar_current_max_warn': 3.0,
            'solar_current_max_caution': 2.7,
            'solar_power_min_caution': 10.0,
            'bus_voltage_min_warn': 10.5,
            'bus_voltage_min_caution': 10.8,
            'bus_voltage_max_warn': 12.8,
            'bus_voltage_max_caution': 12.6,
            'bus_current_max_warn': 3.5,
            'bus_current_max_caution': 3.2,
            'panel_temp_min_warn': -100.0,
            'panel_temp_min_caution': -90.0,
            'panel_temp_max_warn': 80.0,
            'panel_temp_max_caution': 70.0,
            
            # ADCS (Attitude Determination & Control)
            'attitude_sun_point_warn': 10.0,  # Max error for SUN_POINT mode
            'attitude_sun_point_caution': 5.0,  # 50% margin
            'attitude_nadir_warn': 5.0,
            'attitude_nadir_caution': 3.0,
            'attitude_fine_hold_warn': 5.0,  # Max error for FINE_HOLD mode
            'attitude_fine_hold_caution': 2.0,  # 50% margin
            'rate_max_warn': 1.0,  # deg/s
            'rate_max_caution': 0.5,
            'rws_rpm_max_warn': 5400.0,  # 90% of 6000 RPM (within 10% of max)
            'rws_rpm_max_caution': 4500.0,  # 75% of 6000 RPM (within 25% of max)
            'rws_momentum_max_warn': 0.0848,  # Nms (at 5400 RPM)
            'rws_momentum_max_caution': 0.0707,  # Nms (at 4500 RPM)
            
            # TCS (Thermal Control System)
            'temp_avi_min_warn': -5.0,
            'temp_avi_min_caution': 0.0,
            'temp_avi_max_warn': 55.0,
            'temp_avi_max_caution': 50.0,
            'temp_eps_min_warn': -15.0,
            'temp_eps_min_caution': -10.0,
            'temp_eps_max_warn': 50.0,
            'temp_eps_max_caution': 45.0,
            'temp_external_min_warn': -100.0,
            'temp_external_max_warn': 80.0,
            
            # COMM (Communications)
            'comm_temp_max_warn': 65.0,
            'comm_temp_max_caution': 60.0,
            'tx_power_max_warn': 30.0,
            'tx_power_min_warn': 15.0,
            
            # PROP (Propulsion)
            'prop_pressure_min_warn': 50.0,
            'prop_pressure_min_caution': 100.0,
            'prop_pressure_max_warn': 250.0,
            'prop_pressure_max_caution': 230.0,
            'prop_temp_min_warn': -20.0,
            'prop_temp_min_caution': -10.0,
            'prop_temp_max_warn': 60.0,
            'prop_temp_max_caution': 50.0,
            'propellant_mass_min_warn': 0.01,  # kg
            'propellant_mass_min_caution': 0.05,
            
            # AVI (Avionics)
            'cpu_load_max_warn': 95.0,
            'cpu_load_max_caution': 85.0,
            'cpu_temp_max_warn': 75.0,
            'cpu_temp_max_caution': 65.0,
            'memory_usage_max_warn': 95.0,
            'memory_usage_max_caution': 85.0,
            
            # CDH (Command & Data Handling)
            'storage_max_warn': 95.0,
            'storage_max_caution': 85.0,
            
            # TT&C
            'signal_strength_min_warn': -100.0,
            'signal_strength_min_caution': -95.0,
        }
        
        # PROP: Propulsion
        self.prop_state = {
            'system_enabled': False,
            'valve_open': False,
            'tank_pressure': 200.0,
            'tank_temp': 20.0,
            'propellant_mass': 0.5,
            'total_delta_v': 0.0,
            'heater_on': False,
            'desat_valve_auto': False,  # True when valve auto-opened for desaturation
        }
        
        # ADCS: Attitude Determination & Control
        self.adcs_state = {
            'mode': 'DETUMBLE',
            'enabled': False,
            'roll': random.uniform(-30, 30),  # Start in random orientation
            'pitch': random.uniform(-30, 30),
            'yaw': random.uniform(-30, 30),
            'roll_rate': random.uniform(-2.0, 2.0),  # Start tumbling (post-deployment)
            'pitch_rate': random.uniform(-2.0, 2.0),
            'yaw_rate': random.uniform(-2.0, 2.0),
            'rws_rpm_x': random.uniform(-2200, 2200),  # Wheels can spin both directions
            'rws_rpm_y': random.uniform(-2200, 2200),
            'rws_rpm_z': random.uniform(-2200, 2200),
            'rws_momentum_x': 0.0,
            'rws_momentum_y': 0.0,
            'rws_momentum_z': 0.0,
            'rws_saturated_x': False,  # Track individual wheel saturation
            'rws_saturated_y': False,
            'rws_saturated_z': False,
            'rws_desaturation_active': False,  # Track if auto-desaturation is running
            'control_lost': False,  # True when all 3 wheels saturated
            'mag_field_x': 0.0,
            'mag_field_y': 0.0,
            'mag_field_z': 0.0,
            'sun_sensor_x': 0.0,
            'sun_sensor_y': 0.0,
            # FINE_HOLD mode targets (user-specified attitude)
            'target_roll': 0.0,
            'target_pitch': 0.0,
            'target_yaw': 0.0,
        }
        
        # EPS: Electrical Power System
        self.eps_state = {
            'mode': 'NOMINAL',
            'battery_voltage': 11.8,
            'battery_current': -0.5,
            'battery_soc': 85.0,
            'battery_temp': 15.0,
            'solar_voltage': 12.4,
            'solar_current': 1.2,
            'solar_power': 14.9,
            'bus_voltage': 11.8,
            'bus_current': 0.7,
            'panel_temp_x_plus': 10.0,
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
            'heater_prop_on': False,
            'temp_avi': 25.0,
            'temp_eps': 15.0,
            'temp_external': -20.0,
            'radiator_coating': 'GOOD',
        }
        
        # COMM: Communications
        self.comm_state = {
            'mode': 'RX',  # RX, TX, TX_RX (full duplex), STANDBY
            'tx_power': 30.0,
            'frequency': 2200.0,
            'data_rate': 9.6,
            'temp': 30.0,
            'packets_tx': 0,
            'packets_rx': 0,
            'tx_timeout': 0.0,  # Seconds until auto-return to RX (0 = indefinite)
            'tx_time_remaining': 0.0,  # Countdown timer for TX mode
        }
        
        # Safe mode comm cycling state
        self.safe_mode_comm_timer = 0.0
        self.safe_mode_comm_phase = 0  # 0=RX, 1=TX, 2=TX_RX
        
        # CDH: Command & Data Handling
        self.cdh_state = {
            'cmd_count': 0,
            'tlm_count': 0,
            'error_count': 0,
            'storage_used': 0.5,  # GB
            'storage_total': 8.0,  # GB
            'last_cmd_time': 0.0,
        }
        
        # TT&C: Telemetry, Tracking & Command
        self.ttc_state = {
            'link_status': 'NO_LINK',
            'signal_strength': -105.0,
            'uplink_rate': 0.0,
            'downlink_rate': 0.0,
            'pass_elevation': 0.0,
            'doppler_shift': 0.0,
        }
        
        # Orbit state
        self.orbit_state = {
            'altitude': 520.0,
            'velocity': 7.613,
            'latitude': 0.0,
            'longitude': 0.0,
            'eclipse': False,
            'sun_angle': 45.0,
        }
        
        # Simulation parameters
        self.sim_rate = 1.0
        self.last_update = time.time()
        
        # Operational mode
        self.operational_mode = "NOMINAL"
        
        # Event records
        self.evr_log = []
        
        # Caution and warning flags
        self.cautions = []
        self.warnings = []
        
        # Anomaly states
        self.anomaly_states = {
            'battery_low': False,
            'attitude_deviation': False,
            'eps_overtemp': False,
            'avi_overtemp': False,
            'comm_overtemp': False,
            'storage_full': False,
            'adcs_control_lost': False,
        }
        
        # Command queue
        self.command_queue = []
        self.command_counter = 0
        
    def start(self):
        """Start the simulation"""
        self.running = True
        self.sim_thread = Thread(target=self._simulate, daemon=True)
        self.sim_thread.start()
        self.add_evr("6U CubeSat telemetry generator started - 520km LEO")
        
    def stop(self):
        """Stop the simulation"""
        self.running = False
        self.add_evr("Spacecraft telemetry generator stopped")
        
    def _simulate(self):
        """Main simulation loop"""
        while self.running:
            elapsed_time = time.time() - self.epoch_time
            self._update_orbit(elapsed_time)
            self._update_eps(elapsed_time)
            self._update_adcs(elapsed_time)
            self._update_tcs(elapsed_time)
            self._update_avi(elapsed_time)
            self._update_cdh(elapsed_time)
            self._update_ttc(elapsed_time)
            self._update_prop(elapsed_time)
            self._update_comm(elapsed_time)
            self._check_cautions_warnings()
            self._process_commands()
            time.sleep(1.0)  # Update at 1 Hz
            
    def _update_orbit(self, elapsed_time):
        """Update orbital position (520km circular LEO)"""
        with self.lock:
            orbit_fraction = (elapsed_time / 60.0) / self.orbital_period
            angle = 2 * math.pi * orbit_fraction
            
            self.orbit_state['latitude'] = self.orbital_inclination * math.sin(angle)
            self.orbit_state['longitude'] = (angle * 180 / math.pi) % 360 - 180
            self.orbit_state['altitude'] = self.orbital_altitude + random.uniform(-0.2, 0.2)
            self.orbit_state['velocity'] = 7.613 + random.uniform(-0.003, 0.003)
            
            sun_angle = 2 * math.pi * orbit_fraction
            self.orbit_state['eclipse'] = math.cos(sun_angle) < -0.1
            self.orbit_state['sun_angle'] = abs(math.degrees(math.acos(max(-1, min(1, math.cos(sun_angle))))))
            
    def _update_eps(self, elapsed_time):
        """Update Electrical Power System"""
        with self.lock:
            if self.orbit_state['eclipse']:
                sun_exposure = 0.0
            else:
                base_exposure = 1.0 - (self.orbit_state['sun_angle'] / 180.0) * 0.5
                if self.adcs_state['mode'] == 'SUN_POINT':
                    sun_exposure = min(1.0, base_exposure * 1.2)
                else:
                    sun_exposure = base_exposure * 0.7
            
            max_power = 30.0
            self.eps_state['solar_power'] = max_power * sun_exposure + random.uniform(-0.5, 0.5)
            self.eps_state['solar_voltage'] = 12.4 if sun_exposure > 0.1 else 0.0
            self.eps_state['solar_current'] = self.eps_state['solar_power'] / max(0.1, self.eps_state['solar_voltage'])
            
            if self.eps_state['mode'] == 'SAFE':
                base_load = 5.0  # Minimal systems only
            elif self.eps_state['mode'] == 'LOW_POWER':
                base_load = 7.0  # Reduced operations
            elif self.eps_state['mode'] == 'SCIENCE':
                base_load = 15.0  # Full science operations
            else:
                base_load = 10.0  # Normal operations
            
            # Add power draw for each enabled subsystem
            if self.adcs_state['enabled']:
                base_load += 3.0  # ADCS magnetorquers, reaction wheels, sensors
            if self.prop_state['system_enabled']:
                base_load += 1.5  # Propulsion valves and heaters
            if self.comm_state['mode'] == 'TX':
                base_load += 5.0  # Transmitter power (30W RF)
            elif self.comm_state['mode'] == 'RX':
                base_load += 0.5  # Receiver only
            if self.tcs_state['heater_avi_on']:
                base_load += 2.0  # Avionics heater
            if self.tcs_state['heater_eps_on']:
                base_load += 2.0  # EPS heater
            if self.prop_state['heater_on']:
                base_load += 2.0  # PROP tank heater
            
            # CDH, AVI, and TT&C are always on (included in base_load)
                
            self.eps_state['bus_current'] = base_load / self.eps_state['bus_voltage']
            
            power_balance = self.eps_state['solar_power'] - base_load
            
            if power_balance > 0:
                charge_rate = min(power_balance / self.eps_state['battery_voltage'], 2.0)
                self.eps_state['battery_current'] = -charge_rate
                self.eps_state['battery_soc'] = min(100.0, self.eps_state['battery_soc'] + 0.05)
                self.eps_state['battery_voltage'] = min(12.6, self.eps_state['battery_voltage'] + 0.01)
            else:
                discharge_rate = abs(power_balance) / self.eps_state['battery_voltage']
                self.eps_state['battery_current'] = discharge_rate
                drain_mult = 2.0 if self.anomaly_states['battery_low'] else 1.0
                self.eps_state['battery_soc'] = max(0.0, self.eps_state['battery_soc'] - 0.05 * drain_mult)
                self.eps_state['battery_voltage'] = max(10.2, self.eps_state['battery_voltage'] - 0.01 * drain_mult)
            
            self.eps_state['battery_voltage'] = 10.2 + (self.eps_state['battery_soc'] / 100.0) * 2.4
            self.eps_state['bus_voltage'] = self.eps_state['battery_voltage']
            
            ambient = -20.0 if self.orbit_state['eclipse'] else 10.0
            heat_from_charging = abs(self.eps_state['battery_current']) * 0.5
            temp_delta = (ambient + heat_from_charging - self.eps_state['battery_temp']) * 0.05
            self.eps_state['battery_temp'] += temp_delta
            
            if self.orbit_state['eclipse']:
                target_temp = -50.0
            else:
                target_temp = 30.0 - (self.orbit_state['sun_angle'] / 180.0) * 40.0
            
            for panel in ['panel_temp_x_plus', 'panel_temp_x_minus', 'panel_temp_y_plus', 
                         'panel_temp_y_minus', 'panel_temp_z_plus', 'panel_temp_z_minus']:
                current = self.eps_state[panel]
                self.eps_state[panel] = current + (target_temp - current) * 0.1 + random.uniform(-2, 2)
            
    def _update_adcs(self, elapsed_time):
        """Update Attitude Determination & Control System with proper orbital dynamics"""
        with self.lock:
            # Calculate orbital position for reference frame
            orbit_angle = 2 * math.pi * ((elapsed_time / 60.0) / self.orbital_period)
            
            # Update magnetometer (Earth's magnetic field in LEO ~30-50 µT)
            self.adcs_state['mag_field_x'] = 30.0 * math.cos(orbit_angle) + random.uniform(-2, 2)
            self.adcs_state['mag_field_y'] = 20.0 * math.sin(orbit_angle) + random.uniform(-2, 2)
            self.adcs_state['mag_field_z'] = 40.0 + random.uniform(-3, 3)
            
            # Update sun sensors
            if not self.orbit_state['eclipse']:
                self.adcs_state['sun_sensor_x'] = self.orbit_state['sun_angle'] + random.uniform(-1, 1)
                self.adcs_state['sun_sensor_y'] = random.uniform(-10, 10)
            else:
                self.adcs_state['sun_sensor_x'] = 0.0
                self.adcs_state['sun_sensor_y'] = 0.0
            
            if not self.adcs_state['enabled']:
                # ADCS disabled - natural tumbling with environmental torques
                self.adcs_state['roll_rate'] += random.uniform(-0.01, 0.01)
                self.adcs_state['pitch_rate'] += random.uniform(-0.01, 0.01)
                self.adcs_state['yaw_rate'] += random.uniform(-0.01, 0.01)
                self.adcs_state['roll'] += self.adcs_state['roll_rate']
                self.adcs_state['pitch'] += self.adcs_state['pitch_rate']
                self.adcs_state['yaw'] += self.adcs_state['yaw_rate']
            else:
                # ADCS enabled - calculate target attitudes based on mode
                mode = self.adcs_state['mode']
                
                if mode == 'DETUMBLE':
                    # DETUMBLE: Target is zero rates, angles updated by current rates
                    # Control happens in RWS section below
                    self.adcs_state['roll'] += self.adcs_state['roll_rate']
                    self.adcs_state['pitch'] += self.adcs_state['pitch_rate']
                    self.adcs_state['yaw'] += self.adcs_state['yaw_rate']
                    
                elif mode == 'SUN_POINT':
                    # SUN_POINT: Point +X axis at sun (roll = sun_angle, pitch/yaw = 0)
                    # This is our 0,0,0 reference for orbital frame
                    target_roll = 0.0  # Sun angle is already tracked by spacecraft body
                    target_pitch = 0.0
                    target_yaw = 0.0
                    
                    # Calculate attitude errors
                    roll_error = target_roll - self.adcs_state['roll']
                    pitch_error = target_pitch - self.adcs_state['pitch']
                    yaw_error = target_yaw - self.adcs_state['yaw']
                    
                    # PD controller: rate = -Kp*error - Kd*current_rate
                    Kp = 0.05  # Proportional gain
                    Kd = 0.8   # Derivative gain (damping)
                    
                    target_roll_rate = Kp * roll_error - Kd * self.adcs_state['roll_rate']
                    target_pitch_rate = Kp * pitch_error - Kd * self.adcs_state['pitch_rate']
                    target_yaw_rate = Kp * yaw_error - Kd * self.adcs_state['yaw_rate']
                    
                    # Update rates towards target (RWS provides torque)
                    self.adcs_state['roll_rate'] += (target_roll_rate - self.adcs_state['roll_rate']) * 0.1
                    self.adcs_state['pitch_rate'] += (target_pitch_rate - self.adcs_state['pitch_rate']) * 0.1
                    self.adcs_state['yaw_rate'] += (target_yaw_rate - self.adcs_state['yaw_rate']) * 0.1
                    
                    # Update angles
                    self.adcs_state['roll'] += self.adcs_state['roll_rate']
                    self.adcs_state['pitch'] += self.adcs_state['pitch_rate']
                    self.adcs_state['yaw'] += self.adcs_state['yaw_rate']
                    
                elif mode == 'NADIR':
                    # NADIR: Point +Z axis at Earth center (local vertical/local horizontal)
                    # In orbital frame: roll compensates for orbit curvature
                    # Pitch and yaw track nadir direction
                    orbital_rate = 360.0 / self.orbital_period  # deg/min
                    
                    target_roll = 0.0  # No roll in LVLH frame
                    target_pitch = 0.0  # Nadir pointing
                    target_yaw = 0.0  # Aligned with velocity vector
                    
                    roll_error = target_roll - self.adcs_state['roll']
                    pitch_error = target_pitch - self.adcs_state['pitch']
                    yaw_error = target_yaw - self.adcs_state['yaw']
                    
                    Kp = 0.05
                    Kd = 0.8
                    
                    # Nadir pointing requires compensating for orbital motion
                    target_roll_rate = Kp * roll_error - Kd * (self.adcs_state['roll_rate'])
                    target_pitch_rate = Kp * pitch_error - Kd * (self.adcs_state['pitch_rate'] - orbital_rate/60.0)
                    target_yaw_rate = Kp * yaw_error - Kd * self.adcs_state['yaw_rate']
                    
                    self.adcs_state['roll_rate'] += (target_roll_rate - self.adcs_state['roll_rate']) * 0.1
                    self.adcs_state['pitch_rate'] += (target_pitch_rate - self.adcs_state['pitch_rate']) * 0.1
                    self.adcs_state['yaw_rate'] += (target_yaw_rate - self.adcs_state['yaw_rate']) * 0.1
                    
                    self.adcs_state['roll'] += self.adcs_state['roll_rate']
                    self.adcs_state['pitch'] += self.adcs_state['pitch_rate']
                    self.adcs_state['yaw'] += self.adcs_state['yaw_rate']
                    
                elif mode == 'INERTIAL':
                    # INERTIAL: Maintain fixed orientation in inertial space
                    # Slowly damp rates to zero, angles remain constant in inertial frame
                    # but appear to change in orbital frame
                    self.adcs_state['roll_rate'] *= 0.95
                    self.adcs_state['pitch_rate'] *= 0.95
                    self.adcs_state['yaw_rate'] *= 0.95
                    
                    self.adcs_state['roll'] += self.adcs_state['roll_rate']
                    self.adcs_state['pitch'] += self.adcs_state['pitch_rate']
                    self.adcs_state['yaw'] += self.adcs_state['yaw_rate']
                    
                elif mode == 'FINE_HOLD':
                    # FINE_HOLD: User-specified attitude (target_roll, target_pitch, target_yaw)
                    # Spacecraft slews to target and holds with reaction wheels
                    target_roll = self.adcs_state['target_roll']
                    target_pitch = self.adcs_state['target_pitch']
                    target_yaw = self.adcs_state['target_yaw']
                    
                    # Calculate attitude errors
                    roll_error = target_roll - self.adcs_state['roll']
                    pitch_error = target_pitch - self.adcs_state['pitch']
                    yaw_error = target_yaw - self.adcs_state['yaw']
                    
                    # Wrap errors to ±180° (take shortest path)
                    roll_error = (roll_error + 180) % 360 - 180
                    pitch_error = (pitch_error + 180) % 360 - 180
                    yaw_error = (yaw_error + 180) % 360 - 180
                    
                    # PD controller for fine control
                    Kp = 0.05  # Proportional gain
                    Kd = 0.8   # Derivative gain (damping)
                    
                    target_roll_rate = Kp * roll_error - Kd * self.adcs_state['roll_rate']
                    target_pitch_rate = Kp * pitch_error - Kd * self.adcs_state['pitch_rate']
                    target_yaw_rate = Kp * yaw_error - Kd * self.adcs_state['yaw_rate']
                    
                    # Update rates towards target
                    self.adcs_state['roll_rate'] += (target_roll_rate - self.adcs_state['roll_rate']) * 0.1
                    self.adcs_state['pitch_rate'] += (target_pitch_rate - self.adcs_state['pitch_rate']) * 0.1
                    self.adcs_state['yaw_rate'] += (target_yaw_rate - self.adcs_state['yaw_rate']) * 0.1
                    
                    # Update angles
                    self.adcs_state['roll'] += self.adcs_state['roll_rate']
                    self.adcs_state['pitch'] += self.adcs_state['pitch_rate']
                    self.adcs_state['yaw'] += self.adcs_state['yaw_rate']
            
            # Wrap angles to ±180°
            self.adcs_state['roll'] = (self.adcs_state['roll'] + 180) % 360 - 180
            self.adcs_state['pitch'] = (self.adcs_state['pitch'] + 180) % 360 - 180
            self.adcs_state['yaw'] = (self.adcs_state['yaw'] + 180) % 360 - 180
                
            # Update Reaction Wheel System (RWS) - RPM and momentum
            # Momentum (Nms) = Inertia (kg·m²) × Angular_velocity (rad/s)
            # For 6U CubeSat reaction wheels: I ≈ 0.00015 kg·m², max RPM ≈ 6000
            # Nominal operating RPM: 2000-4000, below 2000 indicates insufficient momentum storage
            max_rpm = 6000.0
            min_rpm = 2000.0  # Minimum nominal operating speed
            wheel_inertia = 0.00015  # kg·m²
            
            if self.adcs_state['enabled']:
                mode = self.adcs_state['mode']
            
            # === REACTION WHEEL SYSTEM (RWS) UPDATE ===
            # Wheels dynamically respond to spacecraft rates in ALL modes
            # Conservation of angular momentum: as spacecraft rotates, wheels spin opposite
            max_rpm = 6000.0
            min_rpm = 2000.0
            wheel_inertia = 0.00015  # kg·m²
            
            if self.adcs_state['enabled'] and not self.adcs_state['control_lost']:
                mode = self.adcs_state['mode']
                
                # ALL modes: Wheels absorb/provide angular momentum
                # Wheel acceleration proportional to body rates (torque = I * alpha)
                # Negative sign: wheel spins opposite to spacecraft rotation
                
                if mode == 'DETUMBLE':
                    # DETUMBLE: Absorb spacecraft angular momentum to reduce rates
                    # Wheel RPM change = -rate * gain (wheels spin opposite to spacecraft)
                    # Lower gain to prevent runaway accumulation
                    torque_gain = 200.0  # Moderate gain for realistic momentum absorption
                    
                    # Only accumulate wheel momentum if rates are significant
                    # This prevents indefinite wheel spinup when rates are near zero
                    rate_threshold = 0.01  # deg/s - ignore rates below this
                    
                    if abs(self.adcs_state['roll_rate']) > rate_threshold:
                        self.adcs_state['rws_rpm_x'] += -self.adcs_state['roll_rate'] * torque_gain
                    if abs(self.adcs_state['pitch_rate']) > rate_threshold:
                        self.adcs_state['rws_rpm_y'] += -self.adcs_state['pitch_rate'] * torque_gain
                    if abs(self.adcs_state['yaw_rate']) > rate_threshold:
                        self.adcs_state['rws_rpm_z'] += -self.adcs_state['yaw_rate'] * torque_gain
                    
                    # Wheels provide counter-torque to damp rates
                    damping_factor = 0.90
                    self.adcs_state['roll_rate'] *= damping_factor
                    self.adcs_state['pitch_rate'] *= damping_factor
                    self.adcs_state['yaw_rate'] *= damping_factor
                    
                elif mode in ['SUN_POINT', 'NADIR', 'INERTIAL', 'FINE_HOLD']:
                    # POINTING MODES: Wheels maintain attitude, compensate for disturbances
                    # Wheels respond to body rates to maintain pointing
                    control_gain = 100.0  # Lower gain for stable fine control
                    
                    # Only apply control if rates are significant (prevents drift accumulation)
                    rate_threshold = 0.001  # deg/s
                    
                    if abs(self.adcs_state['roll_rate']) > rate_threshold:
                        self.adcs_state['rws_rpm_x'] += -self.adcs_state['roll_rate'] * control_gain
                    if abs(self.adcs_state['pitch_rate']) > rate_threshold:
                        self.adcs_state['rws_rpm_y'] += -self.adcs_state['pitch_rate'] * control_gain
                    if abs(self.adcs_state['yaw_rate']) > rate_threshold:
                        self.adcs_state['rws_rpm_z'] += -self.adcs_state['yaw_rate'] * control_gain
                    
                    # Environmental perturbations (gravity gradient, solar pressure, drag)
                    # Much smaller to prevent unrealistic accumulation
                    perturbation_x = random.uniform(-2, 2)
                    perturbation_y = random.uniform(-2, 2)
                    perturbation_z = random.uniform(-2, 2)
                    self.adcs_state['rws_rpm_x'] += perturbation_x
                    self.adcs_state['rws_rpm_y'] += perturbation_y
                    self.adcs_state['rws_rpm_z'] += perturbation_z
                
                # Clamp wheels to maximum RPM
                self.adcs_state['rws_rpm_x'] = max(-max_rpm, min(max_rpm, self.adcs_state['rws_rpm_x']))
                self.adcs_state['rws_rpm_y'] = max(-max_rpm, min(max_rpm, self.adcs_state['rws_rpm_y']))
                self.adcs_state['rws_rpm_z'] = max(-max_rpm, min(max_rpm, self.adcs_state['rws_rpm_z']))
                
                # === AUTOMATIC REACTION WHEEL DESATURATION ===
                # Spacecraft automatically manages wheel momentum using thrusters
                # Thrust vector in +X direction, spacecraft slews to align thrust for desaturation
                desat_threshold = 4500.0  # Begin desaturation at 75% capacity
                target_rpm = 2000.0  # Target RPM after desaturation (nominal operating point)
                
                needs_desat_x = abs(self.adcs_state['rws_rpm_x']) > desat_threshold
                needs_desat_y = abs(self.adcs_state['rws_rpm_y']) > desat_threshold
                needs_desat_z = abs(self.adcs_state['rws_rpm_z']) > desat_threshold
                
                any_needs_desat = needs_desat_x or needs_desat_y or needs_desat_z
                
                if any_needs_desat and self.prop_state['system_enabled'] and self.prop_state['propellant_mass'] > 0.01:
                    # Start or continue desaturation
                    if not self.adcs_state['rws_desaturation_active']:
                        # Just started desaturation
                        self.adcs_state['rws_desaturation_active'] = True
                        axes_desat = []
                        if needs_desat_x: axes_desat.append('X')
                        if needs_desat_y: axes_desat.append('Y')
                        if needs_desat_z: axes_desat.append('Z')
                        self.add_evr(f"RWS desaturation initiated - axes: {', '.join(axes_desat)}")
                        self.add_evr("Spacecraft slewing to desaturation attitude...")
                    
                    # Desaturation maneuver in progress
                    # Spacecraft autonomously opens/closes valve as needed
                    desat_rate = 50.0  # RPM reduction per cycle
                    fuel_per_axis = 0.00001  # kg fuel per axis per cycle
                    
                    valve_needed = False
                    
                    if needs_desat_x and abs(self.adcs_state['rws_rpm_x']) > target_rpm:
                        valve_needed = True
                        if self.adcs_state['rws_rpm_x'] > 0:
                            self.adcs_state['rws_rpm_x'] -= desat_rate
                        else:
                            self.adcs_state['rws_rpm_x'] += desat_rate
                        self.prop_state['propellant_mass'] -= fuel_per_axis
                            
                    if needs_desat_y and abs(self.adcs_state['rws_rpm_y']) > target_rpm:
                        valve_needed = True
                        if self.adcs_state['rws_rpm_y'] > 0:
                            self.adcs_state['rws_rpm_y'] -= desat_rate
                        else:
                            self.adcs_state['rws_rpm_y'] += desat_rate
                        self.prop_state['propellant_mass'] -= fuel_per_axis
                            
                    if needs_desat_z and abs(self.adcs_state['rws_rpm_z']) > target_rpm:
                        valve_needed = True
                        if self.adcs_state['rws_rpm_z'] > 0:
                            self.adcs_state['rws_rpm_z'] -= desat_rate
                        else:
                            self.adcs_state['rws_rpm_z'] += desat_rate
                        self.prop_state['propellant_mass'] -= fuel_per_axis
                    
                    # Automatic valve control
                    if valve_needed and not self.prop_state['desat_valve_auto']:
                        self.prop_state['valve_open'] = True
                        self.prop_state['desat_valve_auto'] = True
                        self.add_evr("PROP valve opened (auto desaturation)")
                    elif not valve_needed and self.prop_state['desat_valve_auto']:
                        self.prop_state['valve_open'] = False
                        self.prop_state['desat_valve_auto'] = False
                        self.add_evr("PROP valve closed (auto desaturation)")
                        
                elif self.adcs_state['rws_desaturation_active']:
                    # Desaturation complete or aborted
                    self.adcs_state['rws_desaturation_active'] = False
                    if self.prop_state['desat_valve_auto']:
                        self.prop_state['valve_open'] = False
                        self.prop_state['desat_valve_auto'] = False
                        self.add_evr("PROP valve closed (auto desaturation)")
                    
                    if not any_needs_desat:
                        self.add_evr("RWS desaturation complete - wheels nominal")
                    elif not self.prop_state['system_enabled']:
                        self.add_evr("RWS desaturation aborted - PROP not enabled")
                    else:
                        self.add_evr("RWS desaturation aborted - insufficient propellant")
                
                # === CHECK FOR CONTROL LOSS DUE TO SATURATION ===
                all_saturated = (abs(self.adcs_state['rws_rpm_x']) >= max_rpm and
                                abs(self.adcs_state['rws_rpm_y']) >= max_rpm and
                                abs(self.adcs_state['rws_rpm_z']) >= max_rpm)
                
                if all_saturated and not self.adcs_state['control_lost']:
                    # CRITICAL: Just lost control
                    self.adcs_state['control_lost'] = True
                    self.add_evr("CRITICAL: All RWS saturated - ADCS control LOST!")
                    self.add_evr("Initiating automatic safe mode sequence...")
                    self._enter_safe_mode()
                    
                elif all_saturated and self.adcs_state['control_lost']:
                    # Still saturated - spacecraft tumbles
                    tumble_accel = 0.1  # deg/s² 
                    self.adcs_state['roll_rate'] += random.uniform(-tumble_accel, tumble_accel)
                    self.adcs_state['pitch_rate'] += random.uniform(-tumble_accel, tumble_accel)
                    self.adcs_state['yaw_rate'] += random.uniform(-tumble_accel, tumble_accel)
                    
                    max_tumble_rate = 5.0  # deg/s
                    self.adcs_state['roll_rate'] = max(-max_tumble_rate, min(max_tumble_rate, self.adcs_state['roll_rate']))
                    self.adcs_state['pitch_rate'] = max(-max_tumble_rate, min(max_tumble_rate, self.adcs_state['pitch_rate']))
                    self.adcs_state['yaw_rate'] = max(-max_tumble_rate, min(max_tumble_rate, self.adcs_state['yaw_rate']))
                    
                elif not all_saturated and self.adcs_state['control_lost']:
                    # Recovered from saturation
                    self.adcs_state['control_lost'] = False
                    self.add_evr("RWS desaturated - attitude control restored")
                
                # Maintain minimum operating RPM
                if abs(self.adcs_state['rws_rpm_x']) < min_rpm:
                    self.adcs_state['rws_rpm_x'] = min_rpm if self.adcs_state['rws_rpm_x'] >= 0 else -min_rpm
                if abs(self.adcs_state['rws_rpm_y']) < min_rpm:
                    self.adcs_state['rws_rpm_y'] = min_rpm if self.adcs_state['rws_rpm_y'] >= 0 else -min_rpm
                if abs(self.adcs_state['rws_rpm_z']) < min_rpm:
                    self.adcs_state['rws_rpm_z'] = min_rpm if self.adcs_state['rws_rpm_z'] >= 0 else -min_rpm
                
                # Angular momentum = I * omega (convert RPM to rad/s)
                self.adcs_state['rws_momentum_x'] = wheel_inertia * (self.adcs_state['rws_rpm_x'] * 2 * math.pi / 60.0)
                self.adcs_state['rws_momentum_y'] = wheel_inertia * (self.adcs_state['rws_rpm_y'] * 2 * math.pi / 60.0)
                self.adcs_state['rws_momentum_z'] = wheel_inertia * (self.adcs_state['rws_rpm_z'] * 2 * math.pi / 60.0)
            else:
                # ADCS disabled - wheels spin down slowly due to friction
                # But maintain minimum bias momentum for stability
                self.adcs_state['rws_rpm_x'] *= 0.98
                self.adcs_state['rws_rpm_y'] *= 0.98
                self.adcs_state['rws_rpm_z'] *= 0.98
                
                # Floor at minimum RPM
                if abs(self.adcs_state['rws_rpm_x']) < min_rpm:
                    self.adcs_state['rws_rpm_x'] = min_rpm if self.adcs_state['rws_rpm_x'] >= 0 else -min_rpm
                if abs(self.adcs_state['rws_rpm_y']) < min_rpm:
                    self.adcs_state['rws_rpm_y'] = min_rpm if self.adcs_state['rws_rpm_y'] >= 0 else -min_rpm
                if abs(self.adcs_state['rws_rpm_z']) < min_rpm:
                    self.adcs_state['rws_rpm_z'] = min_rpm if self.adcs_state['rws_rpm_z'] >= 0 else -min_rpm
                
                self.adcs_state['rws_momentum_x'] = wheel_inertia * (self.adcs_state['rws_rpm_x'] * 2 * math.pi / 60.0)
                self.adcs_state['rws_momentum_y'] = wheel_inertia * (self.adcs_state['rws_rpm_y'] * 2 * math.pi / 60.0)
                self.adcs_state['rws_momentum_z'] = wheel_inertia * (self.adcs_state['rws_rpm_z'] * 2 * math.pi / 60.0)
                
    def _update_tcs(self, elapsed_time):
        """Update Thermal Control System"""
        with self.lock:
            if self.orbit_state['eclipse']:
                target_external = -80.0
            else:
                target_external = -20.0 + (self.orbit_state['sun_angle'] / 180.0) * 40.0
            
            temp_delta = (target_external - self.tcs_state['temp_external']) * 0.1
            self.tcs_state['temp_external'] += temp_delta
            
            base_avi_temp = 20.0
            cpu_heat = (self.avi_state['cpu_load'] / 100.0) * 10.0
            heater_heat = 15.0 if self.tcs_state['heater_avi_on'] else 0.0
            cooling = (self.tcs_state['temp_avi'] - self.tcs_state['temp_external']) * 0.05
            
            target_avi = base_avi_temp + cpu_heat + heater_heat - cooling
            temp_mult = 2.0 if self.anomaly_states['avi_overtemp'] else 1.0
            self.tcs_state['temp_avi'] += (target_avi - self.tcs_state['temp_avi']) * 0.1 * temp_mult
            
            base_eps_temp = 15.0
            battery_heat = abs(self.eps_state['battery_current']) * 2.0
            heater_heat_eps = 15.0 if self.tcs_state['heater_eps_on'] else 0.0
            cooling_eps = (self.tcs_state['temp_eps'] - self.tcs_state['temp_external']) * 0.05
            
            target_eps = base_eps_temp + battery_heat + heater_heat_eps - cooling_eps
            temp_mult_eps = 2.0 if self.anomaly_states['eps_overtemp'] else 1.0
            self.tcs_state['temp_eps'] += (target_eps - self.tcs_state['temp_eps']) * 0.1 * temp_mult_eps
            
            # Auto-enable heaters if critical low temps (but don't auto-disable when commanded on)
            if self.tcs_state['temp_avi'] < 5.0 and not self.tcs_state['heater_avi_on']:
                self.tcs_state['heater_avi_on'] = True
                self.add_evr("TCS: AVI heater auto-enabled (critical temp)")
                
            if self.tcs_state['temp_eps'] < 0.0 and not self.tcs_state['heater_eps_on']:
                self.tcs_state['heater_eps_on'] = True
                self.add_evr("TCS: EPS heater auto-enabled (critical temp)")
            
            # PROP tank thermal management (manual control only)
            # Propellant tank temperature follows external temp + heater contribution
            if self.prop_state['heater_on']:
                heater_heat_prop = 20.0
            else:
                heater_heat_prop = 0.0
            
            ambient = self.tcs_state['temp_external']
            target_prop = ambient + heater_heat_prop
            self.prop_state['tank_temp'] += (target_prop - self.prop_state['tank_temp']) * 0.05
                
    def _update_avi(self, elapsed_time):
        """Update Avionics"""
        with self.lock:
            base_load = 30.0
            if self.adcs_state['enabled']:
                base_load += 10.0
            if self.comm_state['mode'] == 'TX':
                base_load += 5.0
            if self.prop_state['system_enabled']:
                base_load += 5.0
                
            self.avi_state['cpu_load'] = base_load + random.uniform(-5, 5)
            self.avi_state['cpu_temp'] = self.tcs_state['temp_avi'] + self.avi_state['cpu_load'] * 0.2
            self.avi_state['memory_usage'] = min(95.0, self.avi_state['memory_usage'] + random.uniform(-0.1, 0.2))
            self.avi_state['uptime'] = elapsed_time / 3600.0
            self.avi_state['watchdog_ok'] = True
            
    def _update_cdh(self, elapsed_time):
        """Update Command & Data Handling"""
        with self.lock:
            self.cdh_state['tlm_count'] += 1
            self.cdh_state['storage_used'] = min(self.cdh_state['storage_total'], 
                                                  self.cdh_state['storage_used'] + random.uniform(0, 0.001))
            
            if len(self.command_queue) > 0:
                self.cdh_state['last_cmd_time'] = 0.0
            else:
                self.cdh_state['last_cmd_time'] += 1.0
                
    def _update_ttc(self, elapsed_time):
        """Update Tracking, Telemetry & Command"""
        with self.lock:
            orbit_fraction = ((elapsed_time / 60.0) / self.orbital_period) % 1.0
            pass_window = (orbit_fraction % 0.333) < 0.07
            
            # During normal operations, maintain LOCKED status for ground communication
            # Pass window logic used for elevation data but not link status
            if pass_window:
                max_elevation = 40.0 * math.sin(math.pi * (orbit_fraction % 0.333) / 0.07)
                self.ttc_state['pass_elevation'] = max_elevation
                
                if max_elevation > 10.0:
                    self.ttc_state['signal_strength'] = -85.0 + (max_elevation / 40.0) * 15.0
                elif max_elevation > 5.0:
                    self.ttc_state['signal_strength'] = -95.0
                else:
                    self.ttc_state['signal_strength'] = -105.0
            else:
                self.ttc_state['pass_elevation'] = 0.0
                self.ttc_state['signal_strength'] = -85.0
            
            # Link status: LOCKED during experiments/operations (realistic for ground station communication)
            self.ttc_state['link_status'] = 'LOCKED'
            self.ttc_state['uplink_rate'] = 9.6
            self.ttc_state['downlink_rate'] = 9.6
            
            self.ttc_state['doppler_shift'] = random.uniform(-5000, 5000)
            
    def _update_prop(self, elapsed_time):
        """Update Propulsion System"""
        with self.lock:
            if not self.prop_state['system_enabled']:
                return
            
            # Update tank pressure based on propellant mass
            if self.prop_state['propellant_mass'] > 0:
                self.prop_state['tank_pressure'] = 200.0 * (self.prop_state['propellant_mass'] / 0.5)
            else:
                self.prop_state['tank_pressure'] = 0.0
            
            # Manual valve operation (user-commanded maneuvers)
            # Automatic desaturation valve control is handled in _update_adcs()
            if self.prop_state['valve_open'] and not self.prop_state['desat_valve_auto']:
                # Manual maneuver in progress
                consumption = 0.001  # kg per cycle
                self.prop_state['propellant_mass'] = max(0.0, self.prop_state['propellant_mass'] - consumption)
                
                if self.prop_state['propellant_mass'] <= 0:
                    self.prop_state['valve_open'] = False
                    self.add_evr("PROP: Propellant depleted, valve closed")
                else:
                    self.prop_state['total_delta_v'] += 0.1  # m/s
            
    def _update_comm(self, elapsed_time):
        """Update Communications"""
        with self.lock:
            # Handle safe mode comm cycling
            if self.operational_mode == 'SAFE':
                self.safe_mode_comm_timer += 1.0  # Increment by 1 second per cycle
                
                if self.safe_mode_comm_phase == 0:  # RX phase - 60 seconds
                    if self.comm_state['mode'] != 'RX':
                        self.comm_state['mode'] = 'RX'
                    if self.safe_mode_comm_timer >= 60.0:
                        self.safe_mode_comm_phase = 1
                        self.safe_mode_comm_timer = 0.0
                        self.comm_state['mode'] = 'TX'
                        
                elif self.safe_mode_comm_phase == 1:  # TX phase - 60 seconds
                    if self.comm_state['mode'] != 'TX':
                        self.comm_state['mode'] = 'TX'
                    if self.safe_mode_comm_timer >= 60.0:
                        self.safe_mode_comm_phase = 2
                        self.safe_mode_comm_timer = 0.0
                        self.comm_state['mode'] = 'TX_RX'
                        
                elif self.safe_mode_comm_phase == 2:  # TX_RX phase - 30 seconds
                    if self.comm_state['mode'] != 'TX_RX':
                        self.comm_state['mode'] = 'TX_RX'
                    if self.safe_mode_comm_timer >= 30.0:
                        self.safe_mode_comm_phase = 0
                        self.safe_mode_comm_timer = 0.0
                        self.comm_state['mode'] = 'RX'
            
            # Handle TX mode timeout (not in safe mode)
            elif self.comm_state['mode'] == 'TX' and self.comm_state['tx_timeout'] > 0:
                self.comm_state['tx_time_remaining'] -= 1.0
                if self.comm_state['tx_time_remaining'] <= 0:
                    self.comm_state['mode'] = 'RX'
                    self.comm_state['tx_timeout'] = 0.0
                    self.comm_state['tx_time_remaining'] = 0.0
                    self.add_evr("COMM: TX timeout - returned to RX mode")
            
            # Default to RX mode unless commanded otherwise or anomaly present
            if self.comm_state['mode'] == 'STANDBY' and not self.anomaly_states['comm_overtemp']:
                self.comm_state['mode'] = 'RX'
            
            # Temperature based on mode
            if self.comm_state['mode'] == 'TX':
                base_temp = 50.0 + (self.comm_state['tx_power'] / 30.0) * 10.0
                temp_mult = 2.0 if self.anomaly_states['comm_overtemp'] else 1.0
            elif self.comm_state['mode'] == 'TX_RX':
                # Duplex mode - higher temperature than RX, lower than TX-only
                base_temp = 45.0 + (self.comm_state['tx_power'] / 30.0) * 8.0
                temp_mult = 2.0 if self.anomaly_states['comm_overtemp'] else 1.0
            elif self.comm_state['mode'] == 'RX':
                base_temp = 35.0
                temp_mult = 1.0
            else:  # STANDBY
                base_temp = 25.0
                temp_mult = 1.0
            
            cooling = (self.comm_state['temp'] - self.tcs_state['temp_external']) * 0.1
            target_temp = base_temp - cooling
            self.comm_state['temp'] += (target_temp - self.comm_state['temp']) * 0.1 * temp_mult
            
            # Packet transmission
            if self.comm_state['mode'] in ['TX', 'TX_RX'] and self.ttc_state['link_status'] == 'LOCKED':
                self.comm_state['packets_tx'] += 1
            
            # Packet reception
            if self.comm_state['mode'] in ['RX', 'TX_RX'] and self.ttc_state['link_status'] == 'LOCKED':
                if random.random() < 0.1:
                    self.comm_state['packets_rx'] += 1
    
    def _enter_safe_mode(self):
        """
        Execute automatic safe mode sequence when critical anomaly detected
        Safe mode is designed to protect spacecraft and wait for ground intervention
        """
        with self.lock:
            self.add_evr("=== ENTERING SAFE MODE ===")
            
            # Set system to SAFE mode
            self.eps_state['mode'] = 'SAFE'
            self.operational_mode = 'SAFE'
            
            # Disable high-power consumers to conserve battery
            self.adcs_state['enabled'] = False
            self.adcs_state['mode'] = 'DETUMBLE'  # Will re-enable in DETUMBLE when recovered
            self.add_evr("SAFE MODE: ADCS disabled")
            
            # NOTE: Reaction wheels maintain their current RPM in safe mode
            # They are NOT spun down - this preserves angular momentum state
            
            # Initialize safe mode comm cycling (RX → TX → TX_RX)
            self.comm_state['mode'] = 'RX'  # Start with RX
            self.safe_mode_comm_timer = 0.0
            self.safe_mode_comm_phase = 0
            self.add_evr("SAFE MODE: COMM cycling initiated (60s RX → 60s TX → 30s TX_RX)")
            
            # Disable PROP to prevent uncommanded maneuvers
            self.prop_state['system_enabled'] = False
            self.prop_state['valve_open'] = False
            self.add_evr("SAFE MODE: PROP disabled")
            
            # Point solar panels to sun (if possible without ADCS)
            # In real system, this might use magnetic torquers or wait for passive stability
            self.add_evr("SAFE MODE: Attempting sun acquisition for power")
            
            # Reset error counters
            self.cdh_state['error_count'] = 0
            
            self.add_evr("=== SAFE MODE ACTIVE - Awaiting ground intervention ===")
            
    def _check_cautions_warnings(self):
        """Check for caution and warning conditions"""
        with self.lock:
            self.cautions = []
            self.warnings = []
            
            # === EPS (Electrical Power System) ===
            # Battery Voltage
            if self.eps_state['battery_voltage'] < self.thresholds['battery_voltage_min_warn']:
                self.warnings.append("Battery voltage critical low")
                self.anomaly_states['battery_low'] = True
            elif self.eps_state['battery_voltage'] < self.thresholds['battery_voltage_min_caution']:
                self.cautions.append("Battery voltage low")
                
            # Battery SOC (100% is allowed)
            if self.eps_state['battery_soc'] < self.thresholds['battery_soc_min_warn']:
                self.warnings.append("Battery SOC critical")
                self.anomaly_states['battery_low'] = True
            elif self.eps_state['battery_soc'] < self.thresholds['battery_soc_min_caution']:
                self.cautions.append("Battery SOC low")
                
            # Battery Temperature
            if self.eps_state['battery_temp'] > self.thresholds['battery_temp_max_warn']:
                self.warnings.append("Battery temperature high")
                self.anomaly_states['eps_overtemp'] = True
            elif self.eps_state['battery_temp'] > self.thresholds['battery_temp_max_caution']:
                self.cautions.append("Battery temperature elevated")
            if self.eps_state['battery_temp'] < self.thresholds['battery_temp_min_warn']:
                self.warnings.append("Battery temperature critical low")
            elif self.eps_state['battery_temp'] < self.thresholds['battery_temp_min_caution']:
                self.cautions.append("Battery temperature low")
                
            # Battery Current
            if abs(self.eps_state['battery_current']) > self.thresholds['battery_current_max_warn']:
                self.warnings.append("Battery current exceeds maximum")
            elif abs(self.eps_state['battery_current']) > self.thresholds['battery_current_max_caution']:
                self.cautions.append("Battery current high")
                
            # Bus Current
            if self.eps_state['bus_current'] > self.thresholds['bus_current_max_warn']:
                self.warnings.append("Bus current exceeds maximum")
            elif self.eps_state['bus_current'] > self.thresholds['bus_current_max_caution']:
                self.cautions.append("Bus current high")
            
            # === ADCS (Attitude Determination & Control) ===
            if self.adcs_state['enabled']:
                # Only check attitude errors in pointing modes (not DETUMBLE)
                # During detumble, large angles and rates are expected!
                if self.adcs_state['mode'] == 'SUN_POINT':
                    # SUN_POINT reference frame is (0,0,0) - check deviation from zero
                    sun_pointing_error = abs(self.adcs_state['roll'])
                    pitch_error = abs(self.adcs_state['pitch'])
                    yaw_error = abs(self.adcs_state['yaw'])
                    max_error = max(sun_pointing_error, pitch_error, yaw_error)
                    
                    if max_error > self.thresholds['attitude_sun_point_warn']:
                        self.warnings.append(f"Sun pointing error large ({max_error:.1f}°)")
                        self.anomaly_states['attitude_deviation'] = True
                    elif max_error > self.thresholds['attitude_sun_point_caution']:
                        self.cautions.append(f"Sun pointing error moderate ({max_error:.1f}°)")
                        
                elif self.adcs_state['mode'] == 'NADIR':
                    max_error = max(abs(self.adcs_state['roll']), abs(self.adcs_state['pitch']), abs(self.adcs_state['yaw']))
                    if max_error > self.thresholds['attitude_nadir_warn']:
                        self.warnings.append(f"Nadir pointing error large ({max_error:.1f}°)")
                        self.anomaly_states['attitude_deviation'] = True
                    elif max_error > self.thresholds['attitude_nadir_caution']:
                        self.cautions.append(f"Nadir pointing error moderate ({max_error:.1f}°)")
                
                elif self.adcs_state['mode'] == 'FINE_HOLD':
                    # Check error from target attitude
                    roll_error = abs(self.adcs_state['target_roll'] - self.adcs_state['roll'])
                    pitch_error = abs(self.adcs_state['target_pitch'] - self.adcs_state['pitch'])
                    yaw_error = abs(self.adcs_state['target_yaw'] - self.adcs_state['yaw'])
                    
                    # Wrap errors to ±180° for accurate error measurement
                    roll_error = min(roll_error, 360 - roll_error)
                    pitch_error = min(pitch_error, 360 - pitch_error)
                    yaw_error = min(yaw_error, 360 - yaw_error)
                    
                    max_error = max(roll_error, pitch_error, yaw_error)
                    
                    if max_error > self.thresholds['attitude_fine_hold_warn']:
                        self.warnings.append(f"Fine hold pointing error large ({max_error:.1f}°)")
                        self.anomaly_states['attitude_deviation'] = True
                    elif max_error > self.thresholds['attitude_fine_hold_caution']:
                        self.cautions.append(f"Fine hold pointing error moderate ({max_error:.1f}°)")
                
                # Attitude rate warnings only for pointing modes (not DETUMBLE or INERTIAL)
                if self.adcs_state['mode'] in ['SUN_POINT', 'NADIR']:
                    max_rate = max(abs(self.adcs_state['roll_rate']), 
                                  abs(self.adcs_state['pitch_rate']), 
                                  abs(self.adcs_state['yaw_rate']))
                    if max_rate > self.thresholds['rate_max_warn']:
                        self.warnings.append(f"Attitude rate excessive ({max_rate:.2f}°/s)")
                    elif max_rate > self.thresholds['rate_max_caution']:
                        self.cautions.append(f"Attitude rate high ({max_rate:.2f}°/s)")
                
                # RWS saturation monitoring (applies to all modes)
                # Check individual wheels for saturation
                rpm_x = abs(self.adcs_state['rws_rpm_x'])
                rpm_y = abs(self.adcs_state['rws_rpm_y'])
                rpm_z = abs(self.adcs_state['rws_rpm_z'])
                
                # Check caution threshold (within 25% of max = 4500 RPM)
                if rpm_x > self.thresholds['rws_rpm_max_caution']:
                    if rpm_x > self.thresholds['rws_rpm_max_warn']:
                        self.warnings.append(f"RWS X-axis near saturation ({rpm_x:.0f} RPM)")
                    else:
                        self.cautions.append(f"RWS X-axis RPM high ({rpm_x:.0f} RPM)")
                        
                if rpm_y > self.thresholds['rws_rpm_max_caution']:
                    if rpm_y > self.thresholds['rws_rpm_max_warn']:
                        self.warnings.append(f"RWS Y-axis near saturation ({rpm_y:.0f} RPM)")
                    else:
                        self.cautions.append(f"RWS Y-axis RPM high ({rpm_y:.0f} RPM)")
                        
                if rpm_z > self.thresholds['rws_rpm_max_caution']:
                    if rpm_z > self.thresholds['rws_rpm_max_warn']:
                        self.warnings.append(f"RWS Z-axis near saturation ({rpm_z:.0f} RPM)")
                    else:
                        self.cautions.append(f"RWS Z-axis RPM high ({rpm_z:.0f} RPM)")
                
                # Check for full saturation (>= 6000 RPM)
                max_rpm = 6000.0
                self.adcs_state['rws_saturated_x'] = (rpm_x >= max_rpm)
                self.adcs_state['rws_saturated_y'] = (rpm_y >= max_rpm)
                self.adcs_state['rws_saturated_z'] = (rpm_z >= max_rpm)
                
                # Check if all 3 wheels saturated - CRITICAL: Loss of attitude control
                all_saturated = (self.adcs_state['rws_saturated_x'] and 
                                self.adcs_state['rws_saturated_y'] and 
                                self.adcs_state['rws_saturated_z'])
                
                if all_saturated and not self.adcs_state['control_lost']:
                    self.adcs_state['control_lost'] = True
                    self.warnings.append("CRITICAL: All reaction wheels saturated - ADCS control lost!")
                    self.anomaly_states['adcs_control_lost'] = True
                    self.add_evr("CRITICAL ANOMALY: All RWS saturated, entering safe mode")
                elif all_saturated:
                    self.warnings.append("CRITICAL: ADCS control lost - all wheels saturated")
                    
                # Desaturation recommendation
                saturated_count = sum([self.adcs_state['rws_saturated_x'], 
                                      self.adcs_state['rws_saturated_y'], 
                                      self.adcs_state['rws_saturated_z']])
                if saturated_count > 0 and not self.prop_state['system_enabled']:
                    self.cautions.append(f"{saturated_count} wheel(s) saturated - enable PROP for desaturation")
                    
                # RWS Momentum
                max_momentum = max(abs(self.adcs_state['rws_momentum_x']), 
                                  abs(self.adcs_state['rws_momentum_y']), 
                                  abs(self.adcs_state['rws_momentum_z']))
                if max_momentum > self.thresholds['rws_momentum_max_warn']:
                    self.warnings.append(f"Reaction wheel momentum saturation ({max_momentum:.4f} Nms)")
                elif max_momentum > self.thresholds['rws_momentum_max_caution']:
                    self.cautions.append(f"Reaction wheel momentum high ({max_momentum:.4f} Nms)")
            
            # === TCS (Thermal Control System) ===
            # Avionics Temperature
            if self.tcs_state['temp_avi'] > self.thresholds['temp_avi_max_warn']:
                self.warnings.append("Avionics temperature critical high")
                self.anomaly_states['avi_overtemp'] = True
            elif self.tcs_state['temp_avi'] > self.thresholds['temp_avi_max_caution']:
                self.cautions.append("Avionics temperature elevated")
            if self.tcs_state['temp_avi'] < self.thresholds['temp_avi_min_warn']:
                self.warnings.append("Avionics temperature critical low")
            elif self.tcs_state['temp_avi'] < self.thresholds['temp_avi_min_caution']:
                self.cautions.append("Avionics temperature low")
                
            # EPS Temperature
            if self.tcs_state['temp_eps'] > self.thresholds['temp_eps_max_warn']:
                self.warnings.append("EPS temperature critical high")
                self.anomaly_states['eps_overtemp'] = True
            elif self.tcs_state['temp_eps'] > self.thresholds['temp_eps_max_caution']:
                self.cautions.append("EPS temperature elevated")
            if self.tcs_state['temp_eps'] < self.thresholds['temp_eps_min_warn']:
                self.warnings.append("EPS temperature critical low")
            elif self.tcs_state['temp_eps'] < self.thresholds['temp_eps_min_caution']:
                self.cautions.append("EPS temperature low")
                
            # === COMM (Communications) ===
            if self.comm_state['temp'] > self.thresholds['comm_temp_max_warn']:
                self.warnings.append("COMM temperature critical high")
                self.anomaly_states['comm_overtemp'] = True
            elif self.comm_state['temp'] > self.thresholds['comm_temp_max_caution']:
                self.cautions.append("COMM temperature elevated")
            
            # === PROP (Propulsion) ===
            if self.prop_state['system_enabled']:
                # Tank Pressure
                if self.prop_state['tank_pressure'] < self.thresholds['prop_pressure_min_warn']:
                    self.warnings.append("Propellant tank pressure critical low")
                elif self.prop_state['tank_pressure'] < self.thresholds['prop_pressure_min_caution']:
                    self.cautions.append("Propellant tank pressure low")
                if self.prop_state['tank_pressure'] > self.thresholds['prop_pressure_max_warn']:
                    self.warnings.append("Propellant tank pressure critical high")
                elif self.prop_state['tank_pressure'] > self.thresholds['prop_pressure_max_caution']:
                    self.cautions.append("Propellant tank pressure high")
                
                # Tank Temperature
                if self.prop_state['tank_temp'] < self.thresholds['prop_temp_min_warn']:
                    self.warnings.append("Propellant tank temperature critical low")
                elif self.prop_state['tank_temp'] < self.thresholds['prop_temp_min_caution']:
                    self.cautions.append("Propellant tank temperature low")
                if self.prop_state['tank_temp'] > self.thresholds['prop_temp_max_warn']:
                    self.warnings.append("Propellant tank temperature critical high")
                elif self.prop_state['tank_temp'] > self.thresholds['prop_temp_max_caution']:
                    self.cautions.append("Propellant tank temperature high")
                
                # Propellant Mass
                if self.prop_state['propellant_mass'] < self.thresholds['propellant_mass_min_warn']:
                    self.warnings.append(f"Propellant nearly depleted ({self.prop_state['propellant_mass']:.3f} kg)")
                elif self.prop_state['propellant_mass'] < self.thresholds['propellant_mass_min_caution']:
                    self.cautions.append(f"Propellant low ({self.prop_state['propellant_mass']:.3f} kg)")
            
            # === AVI (Avionics) ===
            # CPU Load
            if self.avi_state['cpu_load'] > self.thresholds['cpu_load_max_warn']:
                self.warnings.append(f"CPU load critical ({self.avi_state['cpu_load']:.1f}%)")
            elif self.avi_state['cpu_load'] > self.thresholds['cpu_load_max_caution']:
                self.cautions.append(f"CPU load high ({self.avi_state['cpu_load']:.1f}%)")
            
            # CPU Temperature
            if self.avi_state['cpu_temp'] > self.thresholds['cpu_temp_max_warn']:
                self.warnings.append(f"CPU temperature critical ({self.avi_state['cpu_temp']:.1f}°C)")
                self.anomaly_states['avi_overtemp'] = True
            elif self.avi_state['cpu_temp'] > self.thresholds['cpu_temp_max_caution']:
                self.cautions.append(f"CPU temperature elevated ({self.avi_state['cpu_temp']:.1f}°C)")
            
            # Memory Usage
            if self.avi_state['memory_usage'] > self.thresholds['memory_usage_max_warn']:
                self.warnings.append(f"Memory usage critical ({self.avi_state['memory_usage']:.1f}%)")
            elif self.avi_state['memory_usage'] > self.thresholds['memory_usage_max_caution']:
                self.cautions.append(f"Memory usage high ({self.avi_state['memory_usage']:.1f}%)")
            
            # === CDH (Command & Data Handling) ===
            storage_pct = (self.cdh_state['storage_used'] / self.cdh_state['storage_total']) * 100.0
            if storage_pct > self.thresholds['storage_max_warn']:
                self.warnings.append(f"Storage critical ({storage_pct:.1f}%)")
                self.anomaly_states['storage_full'] = True
            elif storage_pct > self.thresholds['storage_max_caution']:
                self.cautions.append(f"Storage high ({storage_pct:.1f}%)")
            
            # === TT&C (Tracking, Telemetry & Command) ===
            if self.ttc_state['link_status'] == 'LOCKED':
                if self.ttc_state['signal_strength'] < self.thresholds['signal_strength_min_warn']:
                    self.warnings.append(f"Signal strength critical ({self.ttc_state['signal_strength']:.1f} dBm)")
                elif self.ttc_state['signal_strength'] < self.thresholds['signal_strength_min_caution']:
                    self.cautions.append(f"Signal strength low ({self.ttc_state['signal_strength']:.1f} dBm)")
            
            # Increment error count for each warning detected
            if len(self.warnings) > 0:
                self.cdh_state['error_count'] += len(self.warnings)
                
    def _process_commands(self):
        """Process queued commands"""
        with self.lock:
            if not self.command_queue:
                return
                
            command = self.command_queue.pop(0)
            self._execute_command(command)
            
    def _execute_command(self, command):
        """Execute a spacecraft command"""
        self.command_counter += 1
        self.cdh_state['cmd_count'] += 1
        self.add_evr(f"Executing command: {command}")
        
        # ADCS commands
        if command == "ADCS_ON":
            self.adcs_state['enabled'] = True
            self.adcs_state['mode'] = 'DETUMBLE'
        elif command == "ADCS_OFF":
            self.adcs_state['enabled'] = False
        elif command == "ADCS_NADIR":
            self.adcs_state['enabled'] = True
            self.adcs_state['mode'] = 'NADIR'
        elif command == "ADCS_SUN_POINT":
            self.adcs_state['enabled'] = True
            self.adcs_state['mode'] = 'SUN_POINT'
            
            # Check if this exits safe mode
            if self.operational_mode == 'SAFE':
                self.operational_mode = 'NOMINAL'
                self.eps_state['mode'] = 'NOMINAL'
                self.safe_mode_comm_timer = 0.0
                self.safe_mode_comm_phase = 0
                self.add_evr("Safe mode EXITED - Sun pointing commanded")
        elif command == "ADCS_INERTIAL":
            self.adcs_state['enabled'] = True
            self.adcs_state['mode'] = 'INERTIAL'
        elif command == "ADCS_DETUMBLE":
            self.adcs_state['enabled'] = True
            self.adcs_state['mode'] = 'DETUMBLE'
        elif command.startswith("ADCS_FINE_HOLD"):
            # ADCS_FINE_HOLD <roll> <pitch> <yaw>
            # Example: ADCS_FINE_HOLD 45 30 -15
            parts = command.split()
            if len(parts) == 4:
                try:
                    target_roll = float(parts[1])
                    target_pitch = float(parts[2])
                    target_yaw = float(parts[3])
                    
                    # Clamp to ±180°
                    target_roll = (target_roll + 180) % 360 - 180
                    target_pitch = (target_pitch + 180) % 360 - 180
                    target_yaw = (target_yaw + 180) % 360 - 180
                    
                    self.adcs_state['target_roll'] = target_roll
                    self.adcs_state['target_pitch'] = target_pitch
                    self.adcs_state['target_yaw'] = target_yaw
                    self.adcs_state['enabled'] = True
                    self.adcs_state['mode'] = 'FINE_HOLD'
                    self.add_evr(f"ADCS: Fine hold mode - target R={target_roll:.1f}° P={target_pitch:.1f}° Y={target_yaw:.1f}°")
                except ValueError:
                    self.add_evr("ERROR: ADCS_FINE_HOLD requires 3 numeric arguments (roll pitch yaw)")
            else:
                self.add_evr("ERROR: ADCS_FINE_HOLD format: ADCS_FINE_HOLD <roll> <pitch> <yaw>")
        elif command == "ADCS_RESET":
            self.adcs_state['roll'] = 0.0
            self.adcs_state['pitch'] = 0.0
            self.adcs_state['yaw'] = 0.0
            self.adcs_state['roll_rate'] = 0.0
            self.adcs_state['pitch_rate'] = 0.0
            self.adcs_state['yaw_rate'] = 0.0
        elif command == "ADCS_RWS_DESAT":
            # Manual reaction wheel desaturation command
            # User must have PROP enabled for this to work
            if self.prop_state['system_enabled'] and self.prop_state['propellant_mass'] > 0.05:
                # Reduce all wheel speeds to nominal range (2500 RPM)
                target_rpm = 2500.0
                self.adcs_state['rws_rpm_x'] = target_rpm if self.adcs_state['rws_rpm_x'] > 0 else -target_rpm
                self.adcs_state['rws_rpm_y'] = target_rpm if self.adcs_state['rws_rpm_y'] > 0 else -target_rpm
                self.adcs_state['rws_rpm_z'] = target_rpm if self.adcs_state['rws_rpm_z'] > 0 else -target_rpm
                # Consume propellant for maneuver
                self.prop_state['propellant_mass'] -= 0.02  # kg
                self.adcs_state['control_lost'] = False
                self.add_evr("RWS desaturation complete - wheels at nominal RPM")
            else:
                self.add_evr("RWS desaturation failed - PROP not enabled or insufficient fuel")
            
        # EPS commands
        elif command == "EPS_MODE_SAFE":
            self.eps_state['mode'] = 'SAFE'
            self.operational_mode = 'SAFE'
        elif command == "EPS_MODE_LOW_POWER":
            self.eps_state['mode'] = 'LOW_POWER'
            self.operational_mode = 'LOW_POWER'
        elif command == "EPS_MODE_NOMINAL":
            self.eps_state['mode'] = 'NOMINAL'
            self.operational_mode = 'NOMINAL'
        elif command == "EPS_MODE_SCIENCE":
            self.eps_state['mode'] = 'SCIENCE'
            self.operational_mode = 'SCIENCE'
        elif command == "EPS_RESET":
            self.eps_state['battery_voltage'] = 11.8
            self.eps_state['battery_soc'] = 85.0
            self.anomaly_states['battery_low'] = False
            
        # COMM commands
        elif command.startswith("COMM_TX"):
            # COMM_TX [timeout_seconds]
            # Example: COMM_TX 300 (TX for 5 minutes then return to RX)
            # If no timeout, TX mode is indefinite until manual change or safe mode
            parts = command.split()
            if len(parts) == 2:
                try:
                    timeout = float(parts[1])
                    self.comm_state['tx_timeout'] = timeout
                    self.comm_state['tx_time_remaining'] = timeout
                    self.add_evr(f"COMM: TX mode enabled for {timeout:.0f} seconds")
                except ValueError:
                    self.add_evr("ERROR: COMM_TX timeout must be numeric")
                    return
            else:
                self.comm_state['tx_timeout'] = 0.0  # Indefinite
                self.comm_state['tx_time_remaining'] = 0.0
                self.add_evr("COMM: TX mode enabled (indefinite - spacecraft NOT commandable)")
            
            self.comm_state['mode'] = 'TX'
            
        elif command == "COMM_RX":
            self.comm_state['mode'] = 'RX'
            self.comm_state['tx_timeout'] = 0.0
            self.comm_state['tx_time_remaining'] = 0.0
            
        elif command == "COMM_TX_RX":
            self.comm_state['mode'] = 'TX_RX'
            self.comm_state['tx_timeout'] = 0.0
            self.comm_state['tx_time_remaining'] = 0.0
            
            # Check if this exits safe mode
            if self.operational_mode == 'SAFE':
                self.operational_mode = 'NOMINAL'
                self.eps_state['mode'] = 'NOMINAL'
                self.safe_mode_comm_timer = 0.0
                self.safe_mode_comm_phase = 0
                self.add_evr("Safe mode EXITED - COMM set to TX_RX (full duplex)")
                
        elif command == "COMM_STANDBY":
            self.comm_state['mode'] = 'STANDBY'
            self.comm_state['tx_timeout'] = 0.0
            self.comm_state['tx_time_remaining'] = 0.0
        elif command == "COMM_POWER_30":
            self.comm_state['tx_power'] = 30.0
        elif command == "COMM_POWER_27":
            self.comm_state['tx_power'] = 27.0
        elif command == "COMM_POWER_20":
            self.comm_state['tx_power'] = 20.0
        elif command == "COMM_RESET":
            self.comm_state['mode'] = 'RX'  # Reset to RX mode (default)
            self.comm_state['tx_power'] = 30.0
            self.comm_state['temp'] = 30.0
            self.anomaly_states['comm_overtemp'] = False
            
        # PROP commands
        elif command == "PROP_ON":
            self.prop_state['system_enabled'] = True
        elif command == "PROP_OFF":
            self.prop_state['system_enabled'] = False
            self.prop_state['valve_open'] = False
        elif command == "PROP_VALVE_OPEN":
            if self.prop_state['system_enabled']:
                self.prop_state['valve_open'] = True
        elif command == "PROP_VALVE_CLOSE":
            self.prop_state['valve_open'] = False
            
        # TCS commands
        elif command == "TCS_HEATER_AVI_ON":
            self.tcs_state['heater_avi_on'] = True
        elif command == "TCS_HEATER_AVI_OFF":
            self.tcs_state['heater_avi_on'] = False
        elif command == "TCS_HEATER_EPS_ON":
            self.tcs_state['heater_eps_on'] = True
        elif command == "TCS_HEATER_EPS_OFF":
            self.tcs_state['heater_eps_on'] = False
        elif command == "TCS_HEATER_PROP_ON":
            self.prop_state['heater_on'] = True
        elif command == "TCS_HEATER_PROP_OFF":
            self.prop_state['heater_on'] = False
            
        # AVI commands
        elif command == "AVI_REBOOT":
            self.avi_state['boot_count'] += 1
            self.avi_state['cpu_load'] = 30.0
            self.avi_state['memory_usage'] = 40.0
        elif command == "AVI_POWER_CYCLE":
            self.avi_state['boot_count'] += 1
            self.avi_state['uptime'] = 0.0
            self.epoch_time = time.time()
            
        # System commands
        elif command == "SAFE_MODE":
            self._enter_safe_mode()
        elif command == "EXIT_SAFE_MODE":
            # Exit safe mode - restore to nominal operations
            if self.eps_state['mode'] == 'SAFE':
                self.eps_state['mode'] = 'NOMINAL'
                self.operational_mode = 'NOMINAL'
                self.comm_state['mode'] = 'RX'
                # Don't auto-enable ADCS or PROP - let operator do it manually
                self.add_evr("Exited safe mode - spacecraft in NOMINAL mode")
                self.adcs_state['control_lost'] = False
            else:
                self.add_evr("EXIT_SAFE_MODE ignored - not in safe mode")
        elif command == "SYS_RESET":
            self.__init__()
            self.add_evr("System reset complete")
                
    def add_evr(self, message):
        """Add event record - only transmitted to ground when in TX_RX mode"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        evr = f"[{timestamp}] {message}"
        
        # Always log internally
        self.evr_log.append(evr)
        if len(self.evr_log) > 1000:
            self.evr_log.pop(0)
        
        # Only transmit to ground (print) if in TX_RX mode
        if self.comm_state['mode'] == 'TX_RX':
            print(evr)
        # Silent otherwise - EVR stored but not downlinked
                
    def send_command(self, command):
        """Queue a command for execution"""
        with self.lock:
            # Block commands if in TX-only mode (spacecraft cannot receive)
            if self.comm_state['mode'] == 'TX' and self.operational_mode != 'SAFE':
                self.add_evr(f"Command REJECTED: {command} - spacecraft in TX-only mode (not commandable)")
                return
            
            self.command_queue.append(command)
            self.add_evr(f"Command queued: {command}")
            
    def get_evr_log(self):
        """Get EVR log"""
        with self.lock:
            return self.evr_log.copy()
            
    def get_cautions_warnings(self):
        """Get current cautions and warnings"""
        with self.lock:
            return self.cautions.copy(), self.warnings.copy()
            
    def get_telemetry(self):
        """Get current telemetry organized by subsystem"""
        with self.lock:
            return {
                'orbit': self.orbit_state.copy(),
                'eps': self.eps_state.copy(),
                'adcs': self.adcs_state.copy(),
                'tcs': self.tcs_state.copy(),
                'avi': self.avi_state.copy(),
                'cdh': self.cdh_state.copy(),
                'ttc': self.ttc_state.copy(),
                'prop': self.prop_state.copy(),
                'comm': self.comm_state.copy()
            }
            
    def get_telemetry_channels(self):
        """Get telemetry formatted as flat channels (backward compatibility)"""
        with self.lock:
            channels = []
            for subsys_name, subsys_data in [
                ('orbit', self.orbit_state),
                ('eps', self.eps_state),
                ('adcs', self.adcs_state),
                ('tcs', self.tcs_state),
                ('avi', self.avi_state),
                ('cdh', self.cdh_state),
                ('ttc', self.ttc_state),
                ('prop', self.prop_state),
                ('comm', self.comm_state)
            ]:
                for key, value in subsys_data.items():
                    channel_name = f"{subsys_name.upper()}_{key.upper()}"
                    if isinstance(value, float):
                        data = f"{value:.2f}"
                    elif isinstance(value, bool):
                        data = "YES" if value else "NO"
                    else:
                        data = str(value)
                    channels.append((channel_name, data))
            return channels


# Global simulator instance
spacecraft = SpacecraftSimulator()

if __name__ == "__main__":
    # Test the 6U CubeSat simulator
    spacecraft.start()
    print("=== 6U CubeSat Simulator Started ===")
    print("Orbital Parameters: 520km circular LEO, 51.6° inclination")
    print("Press Ctrl+C to stop.\n")
    
    try:
        while True:
            time.sleep(2)
            tlm = spacecraft.get_telemetry()
            
            print(f"\n=== Orbit ===")
            print(f"Alt: {tlm['orbit']['altitude']:.2f} km  Vel: {tlm['orbit']['velocity']:.3f} km/s")
            print(f"Lat: {tlm['orbit']['latitude']:.2f}°  Lon: {tlm['orbit']['longitude']:.2f}°")
            print(f"Eclipse: {tlm['orbit']['eclipse']}  Sun Angle: {tlm['orbit']['sun_angle']:.1f}°")
            
            print(f"\n=== EPS ===")
            print(f"Mode: {tlm['eps']['mode']}  Battery: {tlm['eps']['battery_voltage']:.2f}V @ {tlm['eps']['battery_soc']:.1f}%")
            print(f"Solar: {tlm['eps']['solar_power']:.1f}W  Bus: {tlm['eps']['bus_voltage']:.2f}V @ {tlm['eps']['bus_current']:.2f}A")
            
            print(f"\n=== ADCS ===")
            print(f"Mode: {tlm['adcs']['mode']}  Enabled: {tlm['adcs']['enabled']}")
            print(f"Attitude: R={tlm['adcs']['roll']:.1f}° P={tlm['adcs']['pitch']:.1f}° Y={tlm['adcs']['yaw']:.1f}°")
            
            print(f"\n=== Thermal ===")
            print(f"AVI: {tlm['tcs']['temp_avi']:.1f}°C  EPS: {tlm['tcs']['temp_eps']:.1f}°C  Ext: {tlm['tcs']['temp_external']:.1f}°C")
            
            print(f"\n=== TT&C ===")
            print(f"Link: {tlm['ttc']['link_status']}  Signal: {tlm['ttc']['signal_strength']:.1f} dBm  Elev: {tlm['ttc']['pass_elevation']:.1f}°")
            
            print(f"\n=== COMM ===")
            print(f"Mode: {tlm['comm']['mode']}  TX Power: {tlm['comm']['tx_power']:.0f} dBm  Temp: {tlm['comm']['temp']:.1f}°C")
            print(f"Packets: TX={tlm['comm']['packets_tx']}  RX={tlm['comm']['packets_rx']}")
            
            cautions, warnings = spacecraft.get_cautions_warnings()
            if warnings:
                print(f"\n⚠️  WARNINGS: {', '.join(warnings)}")
            if cautions:
                print(f"⚡ CAUTIONS: {', '.join(cautions)}")
                
    except KeyboardInterrupt:
        spacecraft.stop()
        print("\n\nSimulator stopped.")
        print("\nSimulator stopped.")
