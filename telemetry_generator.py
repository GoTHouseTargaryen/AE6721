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
        
        # Last telemetry cache - stores last downlinked values
        # Used when spacecraft is not transmitting (RX/STANDBY mode)
        self.last_telemetry_cache = {}
        
        # 6U CubeSat Orbital parameters (520 km altitude)
        self.orbital_altitude = 520.0  # km
        self.orbital_period = 94.6  # minutes
        self.orbital_inclination = 97.4  # degrees (sun-synchronous at 520km)
        self.epoch_time = time.time()
        
        # Sun-synchronous orbit parameters
        self.sun_synchronous = True
        self.local_time_ascending_node = 10.5  # 10:30 AM local time (typical sun-sync)
        self.beta_angle_variation_period = 365.25  # days (yearly cycle)
        self.beta_angle_max = 75.0  # Maximum beta angle for sun-sync orbit
        
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
            'rws_rpm_max_warn': 5500.0,  # Warning threshold at abs(5500) RPM
            'rws_rpm_max_caution': 4500.0,  # Caution threshold at abs(4500) RPM
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
        
        # PROP: Propulsion - Heated Catalyst Monopropellant System
        self.prop_state = {
            'standby_mode': False,  # True = 5V power on (electronics only)
            'system_enabled': False,  # True = VBat power on (full operation)
            'catalyst_heater_on': False,  # Catalyst bed heater
            'catalyst_temp': 20.0,  # °C (needs 350°C for operation)
            'catalyst_ready': False,  # True when temp > 300°C
            'valve_open': False,
            'tank_pressure': 200.0,  # psi
            'tank_temp': 20.0,  # °C
            'propellant_mass': 0.5,  # kg (hydrazine or similar monoprop)
            'propellant_type': 'N2H4',  # Hydrazine monopropellant
            'total_delta_v': 0.0,  # m/s
            'desat_valve_auto': False,  # True when valve auto-opened for desaturation
            'voltage_5v': 0.0,  # V (5V rail for electronics)
            'current_5v': 0.0,  # A (standby electronics)
            'voltage_vbat': 0.0,  # V (battery voltage for heater and valves)
            'current_vbat': 0.0,  # A (heater + valve actuators)
            'faulted': False,  # Requires power cycle and reset
        }
        
        # ADCS: Attitude Determination & Control
        self.adcs_state = {
            'mode': 'DETUMBLE',
            'enabled': False,
            'roll': random.uniform(-30, 30),  # Start in random orientation
            'pitch': random.uniform(-30, 30),
            'yaw': random.uniform(-30, 30),
            'roll_rate': random.uniform(-6.0, 6.0),  # Start tumbling fast (post-deployment)
            'pitch_rate': random.uniform(-6.0, 6.0),  # Total rotation 10-18 deg/s
            'yaw_rate': random.uniform(-6.0, 6.0),
            'rws_rpm_x': 0.0,  # Start with wheels at 0 RPM (disabled)
            'rws_rpm_y': 0.0,
            'rws_rpm_z': 0.0,
            'rws_momentum_x': 0.0,
            'rws_momentum_y': 0.0,
            'rws_momentum_z': 0.0,
            'rws_saturated_x': False,  # Track individual wheel saturation
            'rws_saturated_y': False,
            'rws_saturated_z': False,
            'rws_desaturation_active': False,  # Track if auto-desaturation is running
            'desat_slewing': False,  # True when slewing to desaturation attitude
            'desat_attitude_reached': False,  # True when aligned for desaturation
            'control_lost': False,  # True when all 3 wheels saturated
            'mag_field_x': 0.0,
            'mag_field_y': 0.0,
            'mag_field_z': 0.0,
            'sun_sensor_x': 0.0,
            'sun_sensor_y': 0.0,
            'voltage': 0.0,  # V (bus voltage when on)
            'current': 0.0,  # A (~0.83A at 12V when on = 10W nominal)
            # FINE_HOLD mode targets (user-specified attitude)
            'target_roll': 0.0,
            'target_pitch': 0.0,
            'target_yaw': 0.0,
            'faulted': False,  # Requires power cycle and reset
        }
        
        # EPS: Electrical Power System
        self.eps_state = {
            'mode': 'NOMINAL',
            # Solar Arrays (independent)
            'solar_array_a_deployed': False,
            'solar_array_b_deployed': False,
            'solar_array_a_deploying': False,
            'solar_array_b_deploying': False,
            'solar_array_a_deploy_time': 0.0,
            'solar_array_b_deploy_time': 0.0,
            'solar_voltage_a': 0.0,
            'solar_current_a': 0.0,
            'solar_power_a': 0.0,
            'solar_voltage_b': 0.0,
            'solar_current_b': 0.0,
            'solar_power_b': 0.0,
            'solar_power_total': 0.0,
            # Batteries (independent with cross-charging option)
            'battery_a_voltage': 11.8,
            'battery_a_current': 0.0,
            'battery_a_soc': 85.0,
            'battery_a_temp': 15.0,
            'battery_b_voltage': 11.8,
            'battery_b_current': 0.0,
            'battery_b_soc': 85.0,
            'battery_b_temp': 15.0,
            'cross_charging_enabled': False,
            # Bus
            'bus_voltage': 11.8,
            'bus_current': 0.7,
            # Panel temperatures
            'panel_temp_x_plus': 10.0,
            'panel_temp_x_minus': 10.0,
            'panel_temp_y_plus': 10.0,
            'panel_temp_y_minus': 10.0,
            'panel_temp_z_plus': 10.0,
            'panel_temp_z_minus': 10.0,
            'faulted': False,  # Requires power cycle and reset
        }
        
        # TCS: Thermal Control System
        self.tcs_state = {
            'heater_avi_on': False,
            'heater_eps_on': False,
            'heater_prop_on': False,
            'temp_avi': 25.0,
            'temp_eps': 15.0,
            'temp_external': -20.0,
            'radiator_deployed': False,
            'radiator_deploying': False,
            'radiator_deploy_time': 0.0,
            'voltage': 12.0,  # V (bus voltage)
            'current': 0.0,  # A (varies with heater usage: ~0.42A per heater = 5W)
            'faulted': False,  # Requires power cycle and reset
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
            'voltage': 12.0,  # V (bus voltage)
            'current': 0.42,  # A (RX: ~5W = 0.42A, TX: ~30W = 2.5A, STANDBY: ~1W = 0.08A)
            'faulted': False,  # Requires power cycle and reset
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
            'voltage': 12.0,  # V (bus voltage)
            'current': 0.25,  # A (~3W = 0.25A at 12V)
            'faulted': False,  # Requires power cycle and reset
        }
        
        # TT&C: Telemetry, Tracking & Command
        self.ttc_state = {
            'link_status': 'NO_LINK',
            'signal_strength': -105.0,
            'uplink_rate': 0.0,
            'downlink_rate': 0.0,
            'pass_elevation': 0.0,
            'doppler_shift': 0.0,
            'voltage': 12.0,  # V (bus voltage)
            'current': 0.17,  # A (~2W = 0.17A at 12V)
            'faulted': False,  # Requires power cycle and reset
        }
        
        # Star Tracker: Precision attitude determination
        self.star_tracker_state = {
            'enabled': False,
            'num_stars_tracked': 0,
            'attitude_valid': False,
            'attitude_accuracy': 0.0,  # arcseconds
            'exposure_time': 100.0,  # milliseconds
            'sensor_temp': 20.0,  # °C
            'voltage': 0.0,  # V (bus voltage when on)
            'current': 0.0,  # A (0 when off, ~0.21A at 12V when on = 2.5W)
            'faulted': False,  # Requires power cycle and reset
            'init_time_remaining': 180.0,  # seconds until ready
            'roll': 0.0,  # degrees (measured attitude)
            'pitch': 0.0,  # degrees (measured attitude)
            'yaw': 0.0,  # degrees (measured attitude)
        }
        
        # GPS: Position/velocity determination
        self.gps_state = {
            'enabled': False,
            'num_satellites': 0,
            'position_valid': False,
            'position_accuracy': 0.0,  # meters
            'velocity_accuracy': 0.0,  # m/s
            'time_to_first_fix': 0.0,  # seconds
            'receiver_temp': 20.0,  # °C
            'voltage': 0.0,  # V (bus voltage when on)
            'current': 0.0,  # A (0 when off, ~0.25A at 12V when on = 3.0W)
            'faulted': False,  # Requires power cycle and reset
        }
        
        # Orbit state
        self.orbit_state = {
            'altitude': 520.0,
            'velocity': 7.613,
            'latitude': 0.0,
            'longitude': 0.0,
            'eclipse': False,
            'sun_angle': 45.0,
            'beta_angle': 0.0,  # Sun-orbit plane angle (degrees)
            'eclipse_fraction': 0.35,  # Fraction of orbit in eclipse
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
            # Compound anomaly states (off-nominal scenario)
            'battery_a_drain': False,
            'battery_b_drain': False,
            'solar_array_a_fault': False,
            'solar_array_b_fault': False,
            'prop_valve_stuck_open': False,
            'tcs_heaters_stuck_on': False,
        }
        
        # Fault injection control
        self.fault_injection_enabled = False
        self.fault_injection_time_min = 120.0  # 2 minutes
        self.fault_injection_time_max = 300.0  # 5 minutes
        self.fault_scheduled_time = None
        self.fault_injected = False
        
        # Experimental fault system (for researcher-controlled experiments)
        self.experimental_fault_active = False
        self.experimental_fault_type = None  # 'CDH', 'TTC', or 'AVI'
        self.experimental_fault_stale_subsystems = []  # Which subsystems have stale telemetry
        self.commanding_without_telemetry_count = 0
        
        # Command queue
        self.command_queue = []
        self.command_counter = 0
        self.last_command = None
        self.last_command_success = True
        
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
            self._check_fault_injection(elapsed_time)
            self._update_orbit(elapsed_time)
            self._update_eps(elapsed_time)
            self._update_adcs(elapsed_time)
            self._update_tcs(elapsed_time)
            self._update_avi(elapsed_time)
            self._update_cdh(elapsed_time)
            self._update_ttc(elapsed_time)
            self._update_prop(elapsed_time)
            self._update_comm(elapsed_time)
            self._update_star_tracker(elapsed_time)
            self._update_gps(elapsed_time)
            self._check_cautions_warnings()
            self._process_commands()
            time.sleep(1.0)  # Update at 1 Hz
            
    def _update_orbit(self, elapsed_time):
        """Update orbital position (520km sun-synchronous LEO)"""
        with self.lock:
            # Orbit fraction (0 to 1)
            orbit_fraction = (elapsed_time / 60.0) / self.orbital_period
            orbit_angle = 2 * math.pi * orbit_fraction  # Mean anomaly
            
            # Calculate beta angle (sun-orbit plane angle) - varies over year
            # Beta angle determines eclipse duration and solar illumination
            days_since_epoch = elapsed_time / 86400.0  # Convert to days
            year_fraction = (days_since_epoch % self.beta_angle_variation_period) / self.beta_angle_variation_period
            
            # Beta angle oscillates between -beta_max and +beta_max over the year
            # At beta=0, maximum eclipse duration (~35% of orbit)
            # At beta=±90, no eclipse (continuous sun)
            self.orbit_state['beta_angle'] = self.beta_angle_max * math.sin(2 * math.pi * year_fraction)
            
            # Calculate eclipse fraction based on beta angle
            # Eclipse fraction = 0 when |beta| > 75°, max ~0.35 when beta = 0
            beta_rad = math.radians(abs(self.orbit_state['beta_angle']))
            if abs(self.orbit_state['beta_angle']) >= 75.0:
                self.orbit_state['eclipse_fraction'] = 0.0  # Continuous sunlight
            else:
                # Eclipse fraction reduces as beta increases
                # At beta=0: 35% eclipse, At beta=60: ~10% eclipse
                self.orbit_state['eclipse_fraction'] = 0.35 * math.cos(beta_rad)
            
            # Determine if currently in eclipse based on orbital position
            # Eclipse occurs on the night side of Earth
            eclipse_half_angle = self.orbit_state['eclipse_fraction'] * math.pi
            
            # For sun-synchronous orbit, eclipse occurs near orbit angle = π (descending node)
            # Shift based on beta angle
            orbit_phase = (orbit_angle % (2 * math.pi))
            eclipse_center = math.pi  # Eclipse centered at descending node
            
            # Check if in eclipse
            angle_from_eclipse_center = abs(orbit_phase - eclipse_center)
            if angle_from_eclipse_center > math.pi:
                angle_from_eclipse_center = 2 * math.pi - angle_from_eclipse_center
            
            self.orbit_state['eclipse'] = angle_from_eclipse_center < eclipse_half_angle
            
            # Calculate sun angle relative to spacecraft body
            # For sun-synchronous orbit, sun angle varies with orbital position and beta
            if self.orbit_state['eclipse']:
                self.orbit_state['sun_angle'] = 0.0  # No sun in eclipse
            else:
                # Base sun angle from orbital geometry (sun direction in orbit frame)
                # At beta=0: sun angle varies 0-90° over orbit
                # At beta=90: sun angle constant (perpendicular to orbit plane)
                sun_elevation = math.sin(beta_rad)  # Component perpendicular to orbit
                sun_azimuth = math.cos(beta_rad) * math.cos(orbit_phase - eclipse_center)  # In-plane component
                
                # Base sun angle in orbit frame (assumes +X points at sun at roll=0)
                base_sun_angle_rad = math.acos(max(-1.0, min(1.0, math.sqrt(sun_elevation**2 + sun_azimuth**2))))
                base_sun_angle = math.degrees(base_sun_angle_rad)
                
                # Adjust sun angle based on spacecraft attitude
                # Solar panels are on -X axis, so optimal pointing is roll=180°
                # Simplified model: sun angle affected primarily by roll error from optimal
                roll_error = abs((self.adcs_state['roll'] - 180.0 + 180) % 360 - 180)
                pitch_error = abs(self.adcs_state['pitch'])
                yaw_error = abs(self.adcs_state['yaw'])
                
                # Attitude error increases effective sun angle
                # Perfect sun pointing (roll=180, pitch=0, yaw=0) minimizes sun angle
                attitude_penalty = math.sqrt(roll_error**2 + pitch_error**2 + yaw_error**2) / 3.0
                
                # Final sun angle: base angle modified by attitude
                # When perfectly pointed (roll=180°), sun_angle approaches base angle
                # When badly pointed, sun angle increases
                # Special case: in SUN_POINT mode, dramatically reduce sun angle when well-pointed
                if self.adcs_state['mode'] == 'SUN_POINT' and attitude_penalty < 10.0:
                    # Well-pointed in sun mode: minimize sun angle for maximum power
                    self.orbit_state['sun_angle'] = min(15.0, attitude_penalty)
                else:
                    self.orbit_state['sun_angle'] = min(90.0, base_sun_angle + attitude_penalty)
            
            # Update latitude and longitude (standard orbital mechanics)
            self.orbit_state['latitude'] = self.orbital_inclination * math.sin(orbit_angle)
            self.orbit_state['longitude'] = (orbit_angle * 180 / math.pi) % 360 - 180
            self.orbit_state['altitude'] = self.orbital_altitude + random.uniform(-0.2, 0.2)
            self.orbit_state['velocity'] = 7.613 + random.uniform(-0.003, 0.003)
    
    def _apply_delta_v_to_orbit(self, delta_v_magnitude, direction_angles):
        """Apply delta-V to orbital parameters using proper coordinate transformations
        
        Reference Frame:
        - Spacecraft body frame at launch: +X axis aligned with Vernal Equinox (ECI frame)
        - Body axes: +X forward (thrust), +Y starboard, +Z nadir
        - Attitude angles (roll, pitch, yaw) represent rotation from initial alignment
        
        Args:
            delta_v_magnitude: Delta-V magnitude in m/s
            direction_angles: dict with 'roll', 'pitch', 'yaw' in degrees
        """
        # Earth parameters
        mu = 3.986e14  # m³/s² - Earth's gravitational parameter
        r_earth = 6371.0  # km - Earth radius
        
        # Current orbital parameters
        r = (r_earth + self.orbital_altitude) * 1000  # Convert to meters
        v_orbital = self.orbit_state['velocity'] * 1000  # Convert km/s to m/s
        
        # COORDINATE TRANSFORMATION: Body frame → ECI frame
        # At launch, body +X was aligned with vernal equinox (ECI +X)
        # Current attitude represents rotation from that initial alignment
        roll_rad = math.radians(direction_angles['roll'])
        pitch_rad = math.radians(direction_angles['pitch'])
        yaw_rad = math.radians(direction_angles['yaw'])
        
        # Thrust is along body +X axis. Transform to ECI frame using rotation matrices.
        # Rotation sequence: Yaw (Z) → Pitch (Y) → Roll (X)
        # This gives us thrust direction in ECI frame
        
        # Thrust vector in body frame (along +X)
        thrust_body = [delta_v_magnitude, 0.0, 0.0]
        
        # Apply yaw rotation (about Z)
        cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
        thrust_after_yaw = [
            cy * thrust_body[0] - sy * thrust_body[1],
            sy * thrust_body[0] + cy * thrust_body[1],
            thrust_body[2]
        ]
        
        # Apply pitch rotation (about Y)
        cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
        thrust_after_pitch = [
            cp * thrust_after_yaw[0] + sp * thrust_after_yaw[2],
            thrust_after_yaw[1],
            -sp * thrust_after_yaw[0] + cp * thrust_after_yaw[2]
        ]
        
        # Apply roll rotation (about X)
        cr, sr = math.cos(roll_rad), math.sin(roll_rad)
        thrust_eci = [
            thrust_after_pitch[0],
            cr * thrust_after_pitch[1] - sr * thrust_after_pitch[2],
            sr * thrust_after_pitch[1] + cr * thrust_after_pitch[2]
        ]
        
        # For circular orbit approximation, decompose delta-V into orbital frame:
        # Radial (toward/away from Earth), Tangential (along velocity), Normal (perpendicular to orbit plane)
        # Assume spacecraft is at equator (latitude=0) with velocity in ECI +Y direction for simplicity
        # This is a simplification - full orbital mechanics would track true anomaly and RAAN
        
        # Position vector (assume equatorial orbit, spacecraft at longitude from state)
        lon_rad = math.radians(self.orbit_state['longitude'])
        pos_eci = [
            r * math.cos(lon_rad),
            r * math.sin(lon_rad),
            0.0  # Equatorial orbit assumption
        ]
        
        # Velocity vector (perpendicular to position for circular orbit)
        vel_eci = [
            -v_orbital * math.sin(lon_rad),
            v_orbital * math.cos(lon_rad),
            0.0
        ]
        
        # Decompose thrust into radial, tangential, normal components
        # Radial: component along position vector
        pos_mag = math.sqrt(sum(p**2 for p in pos_eci))
        pos_unit = [p / pos_mag for p in pos_eci]
        dv_radial = sum(thrust_eci[i] * pos_unit[i] for i in range(3))
        
        # Tangential: component along velocity vector
        vel_mag = math.sqrt(sum(v**2 for v in vel_eci))
        vel_unit = [v / vel_mag for v in vel_eci]
        dv_tangential = sum(thrust_eci[i] * vel_unit[i] for i in range(3))
        
        # Normal: component perpendicular to orbit plane (cross product of radial and tangential)
        normal_unit = [
            pos_unit[1] * vel_unit[2] - pos_unit[2] * vel_unit[1],
            pos_unit[2] * vel_unit[0] - pos_unit[0] * vel_unit[2],
            pos_unit[0] * vel_unit[1] - pos_unit[1] * vel_unit[0]
        ]
        normal_mag = math.sqrt(sum(n**2 for n in normal_unit))
        if normal_mag > 0:
            normal_unit = [n / normal_mag for n in normal_unit]
            dv_normal = sum(thrust_eci[i] * normal_unit[i] for i in range(3))
        else:
            dv_normal = 0.0
        
        # Apply delta-V components to orbital velocity
        v_new = v_orbital + dv_tangential
        
        # Check escape velocity
        escape_velocity_squared = 2 * mu / r
        if v_new**2 >= escape_velocity_squared:
            v_new = 0.99 * math.sqrt(escape_velocity_squared)
            self.add_evr("WARNING: Delta-V would exceed escape velocity - clamping")
        
        # Calculate new orbital parameters
        # Tangential delta-V affects semi-major axis
        denominator = 2.0/r - v_new**2/mu
        if denominator <= 0:
            self.add_evr("ERROR: Invalid orbital parameters after delta-V - reverting")
            return
        
        a_new = 1.0 / denominator
        
        # Radial delta-V affects eccentricity (changes orbit shape)
        # Simplified: radial thrust at periapsis/apoapsis changes eccentricity
        # For small delta-V, approximate effect on altitude variation
        altitude_variation = abs(dv_radial) * r / v_orbital  # meters
        
        # Normal delta-V affects inclination (orbit plane tilt)
        # Δi ≈ Δv_normal / v_orbital (radians)
        inclination_change_rad = dv_normal / v_orbital if v_orbital > 0 else 0.0
        inclination_change_deg = math.degrees(inclination_change_rad)
        
        # Sanity check
        if a_new < r_earth * 1000 or a_new > r_earth * 1000 * 100:
            self.add_evr(f"WARNING: Delta-V resulted in unrealistic orbit (a={a_new/1000:.0f}km) - skipping")
            return
        
        # Update orbital parameters
        altitude_new = (a_new / 1000.0) - r_earth  # km
        period_new = 2 * math.pi * math.sqrt(a_new**3 / mu) / 60.0  # minutes
        
        self.orbital_altitude = altitude_new
        self.orbital_period = period_new
        self.orbit_state['velocity'] = v_new / 1000.0  # km/s
        
        # Log significant orbital changes
        if abs(inclination_change_deg) > 0.01:
            self.add_evr(f"Orbital inclination changed by {inclination_change_deg:.3f}° due to normal delta-V component")
        if altitude_variation > 100:  # More than 100m altitude change
            self.add_evr(f"Orbit eccentricity affected: altitude variation ~{altitude_variation/1000:.2f} km")
    
    def _check_fault_injection(self, elapsed_time):
        """Check if it's time to inject a fault"""
        if not self.fault_injection_enabled or self.fault_injected:
            return
        
        # Schedule fault time on first check
        if self.fault_scheduled_time is None:
            self.fault_scheduled_time = random.uniform(self.fault_injection_time_min, self.fault_injection_time_max)
            self.add_evr(f"Fault injection scheduled at T+{self.fault_scheduled_time:.1f}s")
        
        # Check if it's time to inject fault
        if elapsed_time >= self.fault_scheduled_time:
            self._inject_fault()
            self.fault_injected = True
    
    def _inject_fault(self):
        """Inject a random fault into a subsystem"""
        with self.lock:
            # Select random subsystem to fault
            subsystems = ['ADCS', 'EPS', 'COMM', 'PROP', 'TCS', 'CDH', 'TTC', 'STAR_TRACKER', 'GPS']
            faulted_subsystem = random.choice(subsystems)
            
            if faulted_subsystem == 'ADCS':
                self.adcs_state['faulted'] = True
                self.adcs_state['enabled'] = False
                self.add_evr("*** FAULT INJECTED: ADCS SYSTEM FAULT - Power cycle required ***")
            elif faulted_subsystem == 'EPS':
                self.eps_state['faulted'] = True
                self.add_evr("*** FAULT INJECTED: EPS SYSTEM FAULT - Power cycle required ***")
            elif faulted_subsystem == 'COMM':
                self.comm_state['faulted'] = True
                self.comm_state['mode'] = 'STANDBY'
                self.add_evr("*** FAULT INJECTED: COMM SYSTEM FAULT - Power cycle required ***")
            elif faulted_subsystem == 'PROP':
                self.prop_state['faulted'] = True
                self.prop_state['system_enabled'] = False
                self.prop_state['valve_open'] = False
                self.add_evr("*** FAULT INJECTED: PROP SYSTEM FAULT - Power cycle required ***")
            elif faulted_subsystem == 'TCS':
                self.tcs_state['faulted'] = True
                self.add_evr("*** FAULT INJECTED: TCS SYSTEM FAULT - Power cycle required ***")
            elif faulted_subsystem == 'CDH':
                self.cdh_state['faulted'] = True
                self.add_evr("*** FAULT INJECTED: CDH SYSTEM FAULT - Power cycle required ***")
            elif faulted_subsystem == 'TTC':
                self.ttc_state['faulted'] = True
                self.add_evr("*** FAULT INJECTED: TTC SYSTEM FAULT - Power cycle required ***")
            elif faulted_subsystem == 'STAR_TRACKER':
                self.star_tracker_state['faulted'] = True
                self.star_tracker_state['enabled'] = False
                self.add_evr("*** FAULT INJECTED: STAR TRACKER FAULT - Power cycle required ***")
            elif faulted_subsystem == 'GPS':
                self.gps_state['faulted'] = True
                self.gps_state['enabled'] = False
                self.add_evr("*** FAULT INJECTED: GPS FAULT - Power cycle required ***")
            
            # Log to console regardless of comm mode
            print(f"[FAULT] {faulted_subsystem} system faulted - requires power cycle and reset")
    
    def inject_experimental_fault(self, fault_type):
        """Inject a specific fault for experimental scenarios
        
        Args:
            fault_type: 'CDH', 'TTC', or 'AVI'
        """
        with self.lock:
            if self.experimental_fault_active:
                return  # Only one experimental fault at a time
            
            self.experimental_fault_active = True
            self.experimental_fault_type = fault_type
            self.commanding_without_telemetry_count = 0
            
            if fault_type == 'CDH':
                # CDH fault: Set fault flag, reject all commands except CDH_POWER_CYCLE
                self.cdh_state['faulted'] = True
                self.add_evr("EXPERIMENTAL FAULT: CDH FAULTED - No commands accepted until power cycle")
                print("[EXPERIMENTAL FAULT] CDH faulted - commanding disabled until power cycle")
                
            elif fault_type == 'TTC':
                # TTC fault: Make 75%+ of telemetry channels stale, TTC fault flag never goes stale
                self.ttc_state['faulted'] = True
                
                # Randomly select which subsystems go stale (at least 75%)
                all_subsystems = ['orbit', 'eps', 'adcs', 'tcs', 'avi', 'cdh', 'star_tracker', 'gps', 'prop', 'comm']
                num_to_stale = random.randint(8, 10)  # 8-10 out of 10 = 80-100%
                self.experimental_fault_stale_subsystems = random.sample(all_subsystems, num_to_stale)
                
                self.add_evr(f"EXPERIMENTAL FAULT: TTC FAULTED - {len(self.experimental_fault_stale_subsystems)}/10 subsystems telemetry stale")
                self.add_evr(f"Stale subsystems: {', '.join(self.experimental_fault_stale_subsystems)}")
                print(f"[EXPERIMENTAL FAULT] TTC faulted - {len(self.experimental_fault_stale_subsystems)} subsystems stale")
                
            elif fault_type == 'AVI':
                # AVI fault: All telemetry goes stale except AVI fault flag
                self.avi_state['faulted'] = True
                self.experimental_fault_stale_subsystems = ['orbit', 'eps', 'adcs', 'tcs', 'cdh', 'ttc', 'star_tracker', 'gps', 'prop', 'comm']
                # Note: 'avi' is not in the stale list, so AVI telemetry (including fault flag) remains visible
                self.add_evr("EXPERIMENTAL FAULT: AVI FAULTED - All telemetry stale except AVI, commanding disabled")
                print("[EXPERIMENTAL FAULT] AVI faulted - all telemetry stale, commanding disabled")
    
    def clear_experimental_fault(self):
        """Clear experimental fault (called by power cycle commands)"""
        with self.lock:
            if not self.experimental_fault_active:
                return
            
            fault_type = self.experimental_fault_type
            self.experimental_fault_active = False
            self.experimental_fault_type = None
            self.experimental_fault_stale_subsystems = []
            
            self.add_evr(f"EXPERIMENTAL FAULT CLEARED: {fault_type} power cycled")
            print(f"[EXPERIMENTAL FAULT] {fault_type} fault cleared")
            
    def _update_eps(self, elapsed_time):
        """Update Electrical Power System"""
        with self.lock:
            # Update solar array deployment timers
            if self.eps_state['solar_array_a_deploying']:
                self.eps_state['solar_array_a_deploy_time'] += 1.0
                if self.eps_state['solar_array_a_deploy_time'] >= 10.0:
                    self.eps_state['solar_array_a_deploying'] = False
                    self.eps_state['solar_array_a_deployed'] = not self.eps_state['solar_array_a_deployed']
                    status = "deployed" if self.eps_state['solar_array_a_deployed'] else "stowed"
                    self.add_evr(f"EPS: Solar Array A {status}")
                    
            if self.eps_state['solar_array_b_deploying']:
                self.eps_state['solar_array_b_deploy_time'] += 1.0
                if self.eps_state['solar_array_b_deploy_time'] >= 10.0:
                    self.eps_state['solar_array_b_deploying'] = False
                    self.eps_state['solar_array_b_deployed'] = not self.eps_state['solar_array_b_deployed']
                    status = "deployed" if self.eps_state['solar_array_b_deployed'] else "stowed"
                    self.add_evr(f"EPS: Solar Array B {status}")
            
            # Calculate sun exposure based on eclipse and sun geometry
            if self.orbit_state['eclipse']:
                sun_exposure = 0.0
            else:
                # Solar illumination based on sun angle (cosine law)
                # sun_angle = 0° means sun perpendicular to panel (max power)
                # sun_angle = 90° means sun edge-on to panel (zero power)
                sun_angle_rad = math.radians(self.orbit_state['sun_angle'])
                cosine_factor = max(0.0, math.cos(sun_angle_rad))
                
                # Beta angle affects overall illumination geometry
                # At high beta angles, more consistent sun exposure
                beta_factor = 1.0 - 0.3 * (abs(self.orbit_state['beta_angle']) / 90.0)
                
                # Spacecraft attitude affects solar panel pointing
                if self.adcs_state['mode'] == 'SUN_POINT':
                    # Sun pointing: panels optimally oriented (best case)
                    # When sun pointing, effective sun angle is minimized
                    # Boost power generation significantly in this mode
                    sun_exposure = min(1.0, cosine_factor * 1.5 + 0.3)  # Boosted from 1.3 to 1.5 + base
                else:
                    # Non-optimal pointing: reduced efficiency
                    # Body-mounted panels average ~0.6 of ideal
                    sun_exposure = cosine_factor * beta_factor * 0.6
            
            # Solar Array A (only generates power when deployed)
            max_power_per_array = 15.0  # Watts at AM0, 1367 W/m², optimal angle
            if self.eps_state['solar_array_a_deployed']:
                self.eps_state['solar_power_a'] = max_power_per_array * sun_exposure + random.uniform(-0.2, 0.2)
                self.eps_state['solar_voltage_a'] = 12.4 if sun_exposure > 0.1 else 0.0
                self.eps_state['solar_current_a'] = self.eps_state['solar_power_a'] / max(0.1, self.eps_state['solar_voltage_a'])
                
                # COMPOUND ANOMALY: Solar Array A Fault - zero output
                if self.anomaly_states['solar_array_a_fault']:
                    self.eps_state['solar_power_a'] = 0.0
                    self.eps_state['solar_voltage_a'] = 0.0
                    self.eps_state['solar_current_a'] = 0.0
                    self.eps_state['solar_array_a_status'] = 'FAULTED'
            else:
                self.eps_state['solar_power_a'] = 0.0
                self.eps_state['solar_voltage_a'] = 0.0
                self.eps_state['solar_current_a'] = 0.0
            
            # Solar Array B (only generates power when deployed)
            if self.eps_state['solar_array_b_deployed']:
                self.eps_state['solar_power_b'] = max_power_per_array * sun_exposure + random.uniform(-0.2, 0.2)
                self.eps_state['solar_voltage_b'] = 12.4 if sun_exposure > 0.1 else 0.0
                self.eps_state['solar_current_b'] = self.eps_state['solar_power_b'] / max(0.1, self.eps_state['solar_voltage_b'])
                
                # COMPOUND ANOMALY: Solar Array B Fault - zero output
                if self.anomaly_states['solar_array_b_fault']:
                    self.eps_state['solar_power_b'] = 0.0
                    self.eps_state['solar_voltage_b'] = 0.0
                    self.eps_state['solar_current_b'] = 0.0
                    self.eps_state['solar_array_b_status'] = 'FAULTED'
            else:
                self.eps_state['solar_power_b'] = 0.0
                self.eps_state['solar_voltage_b'] = 0.0
                self.eps_state['solar_current_b'] = 0.0
            
            self.eps_state['solar_power_total'] = self.eps_state['solar_power_a'] + self.eps_state['solar_power_b']
            
            # Calculate total load
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
            if self.star_tracker_state['enabled']:
                base_load += 2.5  # Star tracker camera and processing
            if self.gps_state['enabled']:
                base_load += 3.0  # GPS receiver
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
            if self.tcs_state['heater_prop_on']:
                base_load += 2.0  # PROP tank heater
            
            # CROSS-CHARGING CONFIGURATION
            # When enabled: Solar Array A can charge Battery B, Solar Array B can charge Battery A
            # This provides redundancy if one array or battery fails
            if self.eps_state['cross_charging_enabled']:
                # Pool all solar power and distribute to both batteries
                total_solar_power = self.eps_state['solar_power_a'] + self.eps_state['solar_power_b']
                
                # Split load across both batteries
                load_per_battery = base_load / 2.0
                
                # Battery A can be charged by either solar array
                power_balance_a = (total_solar_power / 2.0) - load_per_battery
                if power_balance_a > 0 and self.eps_state['battery_a_soc'] < 100.0:
                    charge_rate_a = min(power_balance_a / self.eps_state['battery_a_voltage'], 2.0)
                    self.eps_state['battery_a_current'] = -charge_rate_a
                    self.eps_state['battery_a_soc'] = min(100.0, self.eps_state['battery_a_soc'] + 0.05)
                else:
                    discharge_rate_a = abs(power_balance_a) / self.eps_state['battery_a_voltage']
                    self.eps_state['battery_a_current'] = discharge_rate_a
                    self.eps_state['battery_a_soc'] = max(0.0, self.eps_state['battery_a_soc'] - 0.05)
                
                # Battery B can be charged by either solar array
                power_balance_b = (total_solar_power / 2.0) - load_per_battery
                if power_balance_b > 0 and self.eps_state['battery_b_soc'] < 100.0:
                    charge_rate_b = min(power_balance_b / self.eps_state['battery_b_voltage'], 2.0)
                    self.eps_state['battery_b_current'] = -charge_rate_b
                    self.eps_state['battery_b_soc'] = min(100.0, self.eps_state['battery_b_soc'] + 0.05)
                else:
                    discharge_rate_b = abs(power_balance_b) / self.eps_state['battery_b_voltage']
                    self.eps_state['battery_b_current'] = discharge_rate_b
                    self.eps_state['battery_b_soc'] = max(0.0, self.eps_state['battery_b_soc'] - 0.05)
            else:
                # NORMAL MODE: Each solar array charges its own battery
                # Battery A charged by Solar Array A only
                power_balance_a = self.eps_state['solar_power_a'] - (base_load / 2.0)
                if power_balance_a > 0:
                    charge_rate_a = min(power_balance_a / self.eps_state['battery_a_voltage'], 2.0)
                    self.eps_state['battery_a_current'] = -charge_rate_a
                    self.eps_state['battery_a_soc'] = min(100.0, self.eps_state['battery_a_soc'] + 0.05)
                else:
                    discharge_rate_a = abs(power_balance_a) / self.eps_state['battery_a_voltage']
                    self.eps_state['battery_a_current'] = discharge_rate_a
                    self.eps_state['battery_a_soc'] = max(0.0, self.eps_state['battery_a_soc'] - 0.05)
                
                # Battery B charged by Solar Array B only
                power_balance_b = self.eps_state['solar_power_b'] - (base_load / 2.0)
                if power_balance_b > 0:
                    charge_rate_b = min(power_balance_b / self.eps_state['battery_b_voltage'], 2.0)
                    self.eps_state['battery_b_current'] = -charge_rate_b
                    self.eps_state['battery_b_soc'] = min(100.0, self.eps_state['battery_b_soc'] + 0.05)
                else:
                    discharge_rate_b = abs(power_balance_b) / self.eps_state['battery_b_voltage']
                    self.eps_state['battery_b_current'] = discharge_rate_b
                    self.eps_state['battery_b_soc'] = max(0.0, self.eps_state['battery_b_soc'] - 0.05)
            
            # Update battery voltages based on state of charge
            self.eps_state['battery_a_voltage'] = 10.2 + (self.eps_state['battery_a_soc'] / 100.0) * 2.4
            self.eps_state['battery_b_voltage'] = 10.2 + (self.eps_state['battery_b_soc'] / 100.0) * 2.4
            
            # COMPOUND ANOMALY: Battery Drain - voltage decreases despite positive solar power
            # This simulates internal cell failure or short circuit
            if self.anomaly_states['battery_a_drain']:
                self.eps_state['battery_a_voltage'] -= 0.05  # Rapid drain: -50mV per cycle
                self.eps_state['battery_a_soc'] = max(0.0, self.eps_state['battery_a_soc'] - 0.3)  # Fast discharge
                if self.eps_state['battery_a_voltage'] < 10.0:
                    self.eps_state['battery_a_voltage'] = 10.0  # Floor at 10V
                    
            if self.anomaly_states['battery_b_drain']:
                self.eps_state['battery_b_voltage'] -= 0.05  # Rapid drain: -50mV per cycle
                self.eps_state['battery_b_soc'] = max(0.0, self.eps_state['battery_b_soc'] - 0.3)  # Fast discharge
                if self.eps_state['battery_b_voltage'] < 10.0:
                    self.eps_state['battery_b_voltage'] = 10.0  # Floor at 10V
            
            # Bus voltage is average of both batteries
            self.eps_state['bus_voltage'] = (self.eps_state['battery_a_voltage'] + self.eps_state['battery_b_voltage']) / 2.0
            
            # Calculate bus current from total load
            self.eps_state['bus_current'] = base_load / self.eps_state['bus_voltage']
            
            # Battery temperatures
            # Batteries have internal heat generation and thermal mass
            # They cool slowly in eclipse but don't reach full ambient temperature
            ambient = -20.0 if self.orbit_state['eclipse'] else 10.0
            heat_from_charging_a = abs(self.eps_state['battery_a_current']) * 0.5
            heat_from_charging_b = abs(self.eps_state['battery_b_current']) * 0.5
            
            # Batteries have thermal mass and internal heat, so they stabilize warmer than ambient
            # In eclipse: stabilize around 0°C (not -20°C ambient)
            # In sun: stabilize around 10-20°C depending on charging
            target_temp_a = ambient + heat_from_charging_a + 15.0  # Thermal mass offset
            target_temp_b = ambient + heat_from_charging_b + 15.0
            
            # Slow thermal response (large thermal mass)
            temp_delta_a = (target_temp_a - self.eps_state['battery_a_temp']) * 0.03
            temp_delta_b = (target_temp_b - self.eps_state['battery_b_temp']) * 0.03
            self.eps_state['battery_a_temp'] += temp_delta_a
            self.eps_state['battery_b_temp'] += temp_delta_b
            
            # Enforce physical limits (batteries won't go below -10°C or above 50°C)
            self.eps_state['battery_a_temp'] = max(-10.0, min(50.0, self.eps_state['battery_a_temp']))
            self.eps_state['battery_b_temp'] = max(-10.0, min(50.0, self.eps_state['battery_b_temp']))
            
            # Solar panel temperatures
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
            
            # Update voltage and current based on enabled state
            if not self.adcs_state['enabled']:
                # ADCS disabled - no power draw
                self.adcs_state['voltage'] = 0.0
                self.adcs_state['current'] = 0.0
                # Natural tumbling with environmental torques
                self.adcs_state['roll_rate'] += random.uniform(-0.01, 0.01)
                self.adcs_state['pitch_rate'] += random.uniform(-0.01, 0.01)
                self.adcs_state['yaw_rate'] += random.uniform(-0.01, 0.01)
                self.adcs_state['roll'] += self.adcs_state['roll_rate']
                self.adcs_state['pitch'] += self.adcs_state['pitch_rate']
                self.adcs_state['yaw'] += self.adcs_state['yaw_rate']
            else:
                # ADCS enabled - ~10W nominal = ~0.83A at 12V
                self.adcs_state['voltage'] = self.eps_state['bus_voltage']
                self.adcs_state['current'] = 10.0 / self.eps_state['bus_voltage'] if self.eps_state['bus_voltage'] > 0 else 0.0
                # ADCS enabled - calculate target attitudes based on mode
                mode = self.adcs_state['mode']
                
                if mode == 'DESAT':
                    # DESAT mode: Slew to desaturation attitude using normal ADCS control
                    # Target attitude for thruster firing
                    target_roll = self.adcs_state.get('desat_target_roll', 0.0)
                    target_pitch = self.adcs_state.get('desat_target_pitch', 45.0)
                    target_yaw = self.adcs_state.get('desat_target_yaw', 0.0)
                    
                    # Calculate attitude errors
                    roll_error = target_roll - self.adcs_state['roll']
                    pitch_error = target_pitch - self.adcs_state['pitch']
                    yaw_error = target_yaw - self.adcs_state['yaw']
                    
                    # Wrap errors to ±180°
                    roll_error = (roll_error + 180) % 360 - 180
                    pitch_error = (pitch_error + 180) % 360 - 180
                    yaw_error = (yaw_error + 180) % 360 - 180
                    
                    # PD controller for attitude control (more aggressive for desat)
                    Kp = 0.08  # Higher proportional gain for faster slewing
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
                
                elif mode == 'DETUMBLE':
                    # DETUMBLE: Target is zero rates, drive all rates to 0 using PD control
                    # Same control law as pointing modes, but targeting zero attitude rates
                    target_roll_rate = 0.0
                    target_pitch_rate = 0.0
                    target_yaw_rate = 0.0
                    
                    # PD controller: rate = -Kp*error - Kd*current_rate
                    # For detumble, error is just the current rate (target rate is 0)
                    Kp = 0.05  # Proportional gain (not used since target rate is 0)
                    Kd = 0.8   # Derivative gain (damping)
                    
                    # Direct rate damping: target_rate = -Kd * current_rate
                    target_roll_rate = -Kd * self.adcs_state['roll_rate']
                    target_pitch_rate = -Kd * self.adcs_state['pitch_rate']
                    target_yaw_rate = -Kd * self.adcs_state['yaw_rate']
                    
                    # Update rates towards target (reaction wheels provide torque)
                    self.adcs_state['roll_rate'] += (target_roll_rate - self.adcs_state['roll_rate']) * 0.1
                    self.adcs_state['pitch_rate'] += (target_pitch_rate - self.adcs_state['pitch_rate']) * 0.1
                    self.adcs_state['yaw_rate'] += (target_yaw_rate - self.adcs_state['yaw_rate']) * 0.1
                    
                    # Update angles
                    self.adcs_state['roll'] += self.adcs_state['roll_rate']
                    self.adcs_state['pitch'] += self.adcs_state['pitch_rate']
                    self.adcs_state['yaw'] += self.adcs_state['yaw_rate']
                    
                elif mode == 'SUN_POINT':
                    # SUN_POINT: Point -X axis (solar panels) at sun for maximum power
                    # Rotate 180° to point -X instead of +X toward sun
                    target_roll = 180.0  # Flip spacecraft to point -X at sun
                    target_pitch = 0.0
                    target_yaw = 0.0
                    
                    # Calculate attitude errors
                    roll_error = target_roll - self.adcs_state['roll']
                    pitch_error = target_pitch - self.adcs_state['pitch']
                    yaw_error = target_yaw - self.adcs_state['yaw']
                    
                    # Wrap errors to ±180° (take shortest path)
                    roll_error = (roll_error + 180) % 360 - 180
                    pitch_error = (pitch_error + 180) % 360 - 180
                    yaw_error = (yaw_error + 180) % 360 - 180
                    
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
                    # NADIR: Point +X axis at Earth center (nadir direction)
                    # For +X nadir pointing: pitch down 90° from orbital frame
                    orbital_rate = 360.0 / self.orbital_period  # deg/min
                    
                    target_roll = 0.0  # No roll in LVLH frame
                    target_pitch = -90.0  # Pitch down 90° to point +X at Earth
                    target_yaw = 0.0  # Aligned with velocity vector
                    
                    roll_error = target_roll - self.adcs_state['roll']
                    pitch_error = target_pitch - self.adcs_state['pitch']
                    yaw_error = target_yaw - self.adcs_state['yaw']
                    
                    # Wrap errors to ±180°
                    roll_error = (roll_error + 180) % 360 - 180
                    pitch_error = (pitch_error + 180) % 360 - 180
                    yaw_error = (yaw_error + 180) % 360 - 180
                    
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
            # 3-WHEEL PYRAMID CONFIGURATION (skewed, not body-axis aligned)
            # Wheel 1: +X biased (roll), tilted toward +Y and +Z
            # Wheel 2: +Y biased (pitch), tilted toward +Z and +X  
            # Wheel 3: +Z biased (yaw), tilted toward +X and +Y
            # Each wheel contributes to multiple axes for redundancy and efficiency
            #
            # Wheel orientation unit vectors (normalized):
            # Wheel 1: [0.707, 0.500, 0.500]  - primarily X, some Y/Z
            # Wheel 2: [0.500, 0.707, 0.500]  - primarily Y, some Z/X
            # Wheel 3: [0.500, 0.500, 0.707]  - primarily Z, some X/Y
            # (45° tilt from each body axis)
            
            # For 6U CubeSat reaction wheels: I ≈ 0.00015 kg·m², max RPM ≈ 6000
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
                
                if mode == 'DESAT':
                    # DESAT mode: Wheels provide torque for slewing to desaturation attitude
                    # Similar to pointing modes but for desaturation target
                    control_gain = 10.0  # RPM per deg/s
                    
                    rate_threshold = 0.001  # deg/s
                    
                    # Compute desired torque in body axes
                    torque_x = -self.adcs_state['roll_rate'] * control_gain if abs(self.adcs_state['roll_rate']) > rate_threshold else 0.0
                    torque_y = -self.adcs_state['pitch_rate'] * control_gain if abs(self.adcs_state['pitch_rate']) > rate_threshold else 0.0
                    torque_z = -self.adcs_state['yaw_rate'] * control_gain if abs(self.adcs_state['yaw_rate']) > rate_threshold else 0.0
                    
                    # Map body torques to wheel speeds using skewed configuration
                    w1_contribution = 0.707 * torque_x + 0.500 * torque_y + 0.500 * torque_z
                    w2_contribution = 0.500 * torque_x + 0.707 * torque_y + 0.500 * torque_z
                    w3_contribution = 0.500 * torque_x + 0.500 * torque_y + 0.707 * torque_z
                    
                    # Check saturation and redistribute if needed
                    sat_1 = self.adcs_state['rws_saturated_x']
                    sat_2 = self.adcs_state['rws_saturated_y']
                    sat_3 = self.adcs_state['rws_saturated_z']
                    
                    if sat_1:
                        w2_contribution += 0.5 * w1_contribution
                        w3_contribution += 0.5 * w1_contribution
                        w1_contribution = 0.0
                    if sat_2:
                        w1_contribution += 0.5 * w2_contribution
                        w3_contribution += 0.5 * w2_contribution
                        w2_contribution = 0.0
                    if sat_3:
                        w1_contribution += 0.5 * w3_contribution
                        w2_contribution += 0.5 * w3_contribution
                        w3_contribution = 0.0
                    
                    # Apply to wheels
                    self.adcs_state['rws_rpm_x'] += w1_contribution
                    self.adcs_state['rws_rpm_y'] += w2_contribution
                    self.adcs_state['rws_rpm_z'] += w3_contribution
                
                elif mode == 'DETUMBLE':
                    # DETUMBLE: Wheels counter body rates to drive them to zero
                    control_gain = 10.0  # RPM per deg/s - allows 360° rotation without saturation
                    
                    # Only apply control if rates are significant (prevents drift accumulation)
                    rate_threshold = 0.001  # deg/s
                    
                    # Compute desired torque in body axes
                    torque_x = -self.adcs_state['roll_rate'] * control_gain if abs(self.adcs_state['roll_rate']) > rate_threshold else 0.0
                    torque_y = -self.adcs_state['pitch_rate'] * control_gain if abs(self.adcs_state['pitch_rate']) > rate_threshold else 0.0
                    torque_z = -self.adcs_state['yaw_rate'] * control_gain if abs(self.adcs_state['yaw_rate']) > rate_threshold else 0.0
                    
                    # Map body torques to wheel speeds using skewed configuration
                    # Each wheel contributes to multiple axes based on its orientation
                    # Wheel 1 (primarily X): 70.7% X, 50% Y, 50% Z
                    # Wheel 2 (primarily Y): 50% X, 70.7% Y, 50% Z  
                    # Wheel 3 (primarily Z): 50% X, 50% Y, 70.7% Z
                    
                    w1_contribution = 0.707 * torque_x + 0.500 * torque_y + 0.500 * torque_z
                    w2_contribution = 0.500 * torque_x + 0.707 * torque_y + 0.500 * torque_z
                    w3_contribution = 0.500 * torque_x + 0.500 * torque_y + 0.707 * torque_z
                    
                    # Check saturation and redistribute if needed
                    sat_1 = self.adcs_state['rws_saturated_x']
                    sat_2 = self.adcs_state['rws_saturated_y']
                    sat_3 = self.adcs_state['rws_saturated_z']
                    
                    # If a wheel is saturated, redistribute its command to the other two
                    if sat_1:
                        w2_contribution += 0.5 * w1_contribution
                        w3_contribution += 0.5 * w1_contribution
                        w1_contribution = 0.0
                    if sat_2:
                        w1_contribution += 0.5 * w2_contribution
                        w3_contribution += 0.5 * w2_contribution
                        w2_contribution = 0.0
                    if sat_3:
                        w1_contribution += 0.5 * w3_contribution
                        w2_contribution += 0.5 * w3_contribution
                        w3_contribution = 0.0
                    
                    # Apply to wheels
                    self.adcs_state['rws_rpm_x'] += w1_contribution
                    self.adcs_state['rws_rpm_y'] += w2_contribution
                    self.adcs_state['rws_rpm_z'] += w3_contribution
                    
                elif mode in ['SUN_POINT', 'NADIR', 'INERTIAL', 'FINE_HOLD']:
                    # POINTING MODES: Wheels maintain attitude, compensate for disturbances
                    control_gain = 10.0  # RPM per deg/s - allows 360° rotation without saturation
                    
                    # Only apply control if rates are significant (prevents drift accumulation)
                    rate_threshold = 0.001  # deg/s
                    
                    # Compute desired torque in body axes
                    torque_x = -self.adcs_state['roll_rate'] * control_gain if abs(self.adcs_state['roll_rate']) > rate_threshold else 0.0
                    torque_y = -self.adcs_state['pitch_rate'] * control_gain if abs(self.adcs_state['pitch_rate']) > rate_threshold else 0.0
                    torque_z = -self.adcs_state['yaw_rate'] * control_gain if abs(self.adcs_state['yaw_rate']) > rate_threshold else 0.0
                    
                    # Map body torques to wheel speeds using skewed configuration
                    w1_contribution = 0.707 * torque_x + 0.500 * torque_y + 0.500 * torque_z
                    w2_contribution = 0.500 * torque_x + 0.707 * torque_y + 0.500 * torque_z
                    w3_contribution = 0.500 * torque_x + 0.500 * torque_y + 0.707 * torque_z
                    
                    # Check saturation and redistribute if needed
                    sat_1 = self.adcs_state['rws_saturated_x']
                    sat_2 = self.adcs_state['rws_saturated_y']
                    sat_3 = self.adcs_state['rws_saturated_z']
                    
                    if sat_1:
                        w2_contribution += 0.5 * w1_contribution
                        w3_contribution += 0.5 * w1_contribution
                        w1_contribution = 0.0
                    if sat_2:
                        w1_contribution += 0.5 * w2_contribution
                        w3_contribution += 0.5 * w2_contribution
                        w2_contribution = 0.0
                    if sat_3:
                        w1_contribution += 0.5 * w3_contribution
                        w2_contribution += 0.5 * w3_contribution
                        w3_contribution = 0.0
                    
                    # Apply to wheels
                    self.adcs_state['rws_rpm_x'] += w1_contribution
                    self.adcs_state['rws_rpm_y'] += w2_contribution
                    self.adcs_state['rws_rpm_z'] += w3_contribution
                    
                    # Environmental perturbations (gravity gradient, solar pressure, drag)
                    # Much smaller to prevent unrealistic accumulation
                    perturbation_x = random.uniform(-2, 2)
                    perturbation_y = random.uniform(-2, 2)
                    perturbation_z = random.uniform(-2, 2)
                    self.adcs_state['rws_rpm_x'] += perturbation_x
                    self.adcs_state['rws_rpm_y'] += perturbation_y
                    self.adcs_state['rws_rpm_z'] += perturbation_z
                
                # Clamp wheels to maximum RPM and update saturation flags
                prev_sat_x = self.adcs_state['rws_saturated_x']
                prev_sat_y = self.adcs_state['rws_saturated_y']
                prev_sat_z = self.adcs_state['rws_saturated_z']

                self.adcs_state['rws_rpm_x'] = max(-max_rpm, min(max_rpm, self.adcs_state['rws_rpm_x']))
                self.adcs_state['rws_rpm_y'] = max(-max_rpm, min(max_rpm, self.adcs_state['rws_rpm_y']))
                self.adcs_state['rws_rpm_z'] = max(-max_rpm, min(max_rpm, self.adcs_state['rws_rpm_z']))

                self.adcs_state['rws_saturated_x'] = abs(self.adcs_state['rws_rpm_x']) >= (0.999 * max_rpm)
                self.adcs_state['rws_saturated_y'] = abs(self.adcs_state['rws_rpm_y']) >= (0.999 * max_rpm)
                self.adcs_state['rws_saturated_z'] = abs(self.adcs_state['rws_rpm_z']) >= (0.999 * max_rpm)

                # EVRs on saturation state changes
                if self.adcs_state['rws_saturated_x'] and not prev_sat_x:
                    self.add_evr("ADCS: Reaction wheel X saturated - redistributing control to Y/Z")
                if self.adcs_state['rws_saturated_y'] and not prev_sat_y:
                    self.add_evr("ADCS: Reaction wheel Y saturated - redistributing control to X/Z")
                if self.adcs_state['rws_saturated_z'] and not prev_sat_z:
                    self.add_evr("ADCS: Reaction wheel Z saturated - redistributing control to X/Y")

                # Declare complete control loss if all three are saturated
                if (self.adcs_state['rws_saturated_x'] and 
                    self.adcs_state['rws_saturated_y'] and 
                    self.adcs_state['rws_saturated_z']):
                    if not self.adcs_state['control_lost']:
                        self.adcs_state['control_lost'] = True
                        self.add_evr("ADCS: All wheels saturated - control lost; awaiting desaturation")
                else:
                    # If any wheel recovers, restore overall control availability
                    if self.adcs_state['control_lost']:
                        self.adcs_state['control_lost'] = False
                        self.add_evr("ADCS: Control restored (at least one wheel recovered)")
                
                # === AUTOMATIC REACTION WHEEL DESATURATION ===
                # Spacecraft automatically manages wheel momentum using thrusters
                # Simplified: slew to fixed reasonable attitude then fire thrusters
                desat_threshold = 4500.0  # Begin desaturation at 75% capacity
                target_rpm = 500.0  # Target RPM after desaturation (nominal operating point)
                desat_deadband = 100.0  # Deadband around target (stop when within ±100 RPM)
                
                # Fixed desaturation attitude - reasonable orientation for thruster firing
                # Uses +X thruster, so point away from desired delta-V direction
                # Simple fixed attitude works well enough for momentum dumping
                desat_roll = 0.0
                desat_pitch = 45.0  # 45° pitch up
                desat_yaw = 0.0
                
                attitude_tolerance = 10.0  # degrees - acceptable pointing error for thruster firing
                
                # Check if any axis exceeds threshold to START desaturation
                needs_desat_start_x = abs(self.adcs_state['rws_rpm_x']) > desat_threshold
                needs_desat_start_y = abs(self.adcs_state['rws_rpm_y']) > desat_threshold
                needs_desat_start_z = abs(self.adcs_state['rws_rpm_z']) > desat_threshold
                
                any_needs_desat_start = needs_desat_start_x or needs_desat_start_y or needs_desat_start_z
                
                # Check if any axis is outside target deadband (continue desaturation)
                outside_deadband_x = abs(self.adcs_state['rws_rpm_x']) > (target_rpm + desat_deadband)
                outside_deadband_y = abs(self.adcs_state['rws_rpm_y']) > (target_rpm + desat_deadband)
                outside_deadband_z = abs(self.adcs_state['rws_rpm_z']) > (target_rpm + desat_deadband)
                
                any_outside_deadband = outside_deadband_x or outside_deadband_y or outside_deadband_z
                
                # Check if spacecraft is in desaturation attitude
                roll_error = abs(self.adcs_state['roll'] - desat_roll)
                pitch_error = abs(self.adcs_state['pitch'] - desat_pitch)
                yaw_error = abs(self.adcs_state['yaw'] - desat_yaw)
                in_desat_attitude = (roll_error < attitude_tolerance and 
                                    pitch_error < attitude_tolerance and 
                                    yaw_error < attitude_tolerance)
                
                # Start desaturation if threshold exceeded AND catalyst ready AND in attitude
                # Catalyst must be at operating temperature (300°C+) for thruster firing
                if (self.prop_state['system_enabled'] and 
                    self.prop_state['catalyst_ready'] and 
                    self.prop_state['propellant_mass'] > 0.01):
                    if any_needs_desat_start or (self.adcs_state['rws_desaturation_active'] and any_outside_deadband):
                        
                        # Start or continue desaturation
                        if not self.adcs_state['rws_desaturation_active']:
                            # Just started desaturation sequence
                            # Store previous mode to restore after desaturation
                            if not hasattr(self.adcs_state, 'pre_desat_mode'):
                                self.adcs_state['pre_desat_mode'] = self.adcs_state['mode']
                            self.adcs_state['mode'] = 'DESAT'
                            # Set target attitude for ADCS to slew to
                            self.adcs_state['desat_target_roll'] = desat_roll
                            self.adcs_state['desat_target_pitch'] = desat_pitch
                            self.adcs_state['desat_target_yaw'] = desat_yaw
                            self.adcs_state['rws_desaturation_active'] = True
                            self.adcs_state['desat_attitude_reached'] = False
                            axes_desat = []
                            if needs_desat_start_x: axes_desat.append('X')
                            if needs_desat_start_y: axes_desat.append('Y')
                            if needs_desat_start_z: axes_desat.append('Z')
                            self.add_evr(f"RWS desaturation initiated - axes: {', '.join(axes_desat)}")
                            self.add_evr(f"Target: {target_rpm:.0f} RPM ± {desat_deadband:.0f} RPM")
                            self.add_evr(f"Slewing to desaturation attitude (R={desat_roll}° P={desat_pitch}° Y={desat_yaw}°)")
                        
                        # Check if attitude reached
                        if not self.adcs_state['desat_attitude_reached'] and in_desat_attitude:
                            self.adcs_state['desat_attitude_reached'] = True
                            self.add_evr(f"Desaturation attitude reached - beginning thruster firing")
                        
                        # Fire thrusters to reduce wheel momentum when at desaturation attitude
                        if self.adcs_state['desat_attitude_reached'] and any_outside_deadband:
                            # Thruster specifications (typical 1N monoprop thruster for 6U CubeSat)
                            thrust_force = 1.0  # Newtons
                            moment_arm = 0.15  # meters (thruster offset from CoM for 6U)
                            burn_duration_per_cycle = 0.1  # seconds per simulation cycle
                            
                            # Calculate delta-V from thrust
                            spacecraft_mass = 12.0 + self.prop_state['propellant_mass']  # kg (dry + wet)
                            delta_v_per_cycle = (thrust_force * burn_duration_per_cycle) / spacecraft_mass  # m/s
                            
                            # Calculate angular momentum change from thruster torque
                            torque = moment_arm * thrust_force  # N·m
                            angular_impulse = torque * burn_duration_per_cycle  # N·m·s
                            total_momentum_reduction = angular_impulse * 300.0  # Scaling factor: N·m·s to RPM
                            
                            # Calculate fuel consumption
                            isp = 220.0  # seconds (N2H4 monoprop)
                            g0 = 9.81  # m/s²
                            fuel_per_cycle = (spacecraft_mass * delta_v_per_cycle) / (isp * g0)  # kg
                            
                            # Apply fuel consumption for thruster burn
                            fuel_before = self.prop_state['propellant_mass']
                            self.prop_state['propellant_mass'] -= fuel_per_cycle
                            self.prop_state['propellant_mass'] = max(0.0, self.prop_state['propellant_mass'])
                            fuel_consumed = fuel_before - self.prop_state['propellant_mass']
                            
                            # Accumulate total delta-V for mission tracking
                            self.prop_state['total_delta_v'] += delta_v_per_cycle
                            
                            # Log thruster firing event (throttled)
                            if not hasattr(self, '_last_thruster_evr_time'):
                                self._last_thruster_evr_time = 0.0
                            if elapsed_time - self._last_thruster_evr_time > 5.0:  # EVR every 5 seconds
                                self.add_evr(f"THRUSTERS FIRING: Desaturation burn (ΔV={delta_v_per_cycle*1000:.1f} mm/s, fuel={fuel_consumed*1000:.2f}g)")
                                self.add_evr(f"Momentum reduction: {total_momentum_reduction:.0f} RPM, Remaining fuel: {self.prop_state['propellant_mass']*1000:.1f}g")
                                self._last_thruster_evr_time = elapsed_time
                            
                            # Reduce momentum across ALL wheels proportionally
                            h_x = self.adcs_state['rws_momentum_x']
                            h_y = self.adcs_state['rws_momentum_y']
                            h_z = self.adcs_state['rws_momentum_z']
                            h_mag = math.sqrt(h_x**2 + h_y**2 + h_z**2)
                            
                            if h_mag > 0:
                                # Calculate proportion of momentum in each axis
                                h_x_frac = abs(h_x) / h_mag
                                h_y_frac = abs(h_y) / h_mag
                                h_z_frac = abs(h_z) / h_mag
                                
                                # Reduce each wheel proportionally
                                if self.adcs_state['rws_rpm_x'] > 0:
                                    self.adcs_state['rws_rpm_x'] -= total_momentum_reduction * h_x_frac
                                else:
                                    self.adcs_state['rws_rpm_x'] += total_momentum_reduction * h_x_frac
                                    
                                if self.adcs_state['rws_rpm_y'] > 0:
                                    self.adcs_state['rws_rpm_y'] -= total_momentum_reduction * h_y_frac
                                else:
                                    self.adcs_state['rws_rpm_y'] += total_momentum_reduction * h_y_frac
                                    
                                if self.adcs_state['rws_rpm_z'] > 0:
                                    self.adcs_state['rws_rpm_z'] -= total_momentum_reduction * h_z_frac
                                else:
                                    self.adcs_state['rws_rpm_z'] += total_momentum_reduction * h_z_frac
                            
                            # Automatic valve control
                            if not self.prop_state['desat_valve_auto']:
                                self.prop_state['valve_open'] = True
                                self.prop_state['desat_valve_auto'] = True
                                self.add_evr("PROP valve opened (auto desaturation)")
                        else:
                            # Not firing thrusters - close valve if it was open for desat
                            if self.prop_state['desat_valve_auto']:
                                self.prop_state['valve_open'] = False
                                self.prop_state['desat_valve_auto'] = False
                                self.add_evr("PROP valve closed (desat pause)")
                    
                    elif self.adcs_state['rws_desaturation_active'] and not any_outside_deadband:
                        # Desaturation complete - all axes within deadband
                        self.adcs_state['rws_desaturation_active'] = False
                        self.adcs_state['desat_attitude_reached'] = False
                        # Restore previous ADCS mode
                        if hasattr(self.adcs_state, 'pre_desat_mode'):
                            self.adcs_state['mode'] = self.adcs_state['pre_desat_mode']
                            del self.adcs_state['pre_desat_mode']
                        if self.prop_state['desat_valve_auto']:
                            self.prop_state['valve_open'] = False
                            self.prop_state['desat_valve_auto'] = False
                            self.add_evr("PROP valve closed (auto desaturation)")
                        self.add_evr(f"RWS desaturation complete - ADCS mode restored to {self.adcs_state['mode']}")
                        self.add_evr(f"RWS desaturation complete - all axes within {target_rpm:.0f} ± {desat_deadband:.0f} RPM")
                        
                elif self.adcs_state['rws_desaturation_active']:
                    # Desaturation aborted (PROP disabled, catalyst not ready, or out of fuel)
                    self.adcs_state['rws_desaturation_active'] = False
                    self.adcs_state['desat_attitude_reached'] = False
                    if self.prop_state['desat_valve_auto']:
                        self.prop_state['valve_open'] = False
                        self.prop_state['desat_valve_auto'] = False
                        self.add_evr("PROP valve closed (auto desaturation)")
                    
                    if not self.prop_state['system_enabled']:
                        self.add_evr("RWS desaturation aborted - PROP not enabled")
                    elif not self.prop_state['catalyst_ready']:
                        self.add_evr("RWS desaturation aborted - catalyst not ready (need 300°C+)")
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
                
                # Angular momentum = I * omega (convert RPM to rad/s)
                self.adcs_state['rws_momentum_x'] = wheel_inertia * (self.adcs_state['rws_rpm_x'] * 2 * math.pi / 60.0)
                self.adcs_state['rws_momentum_y'] = wheel_inertia * (self.adcs_state['rws_rpm_y'] * 2 * math.pi / 60.0)
                self.adcs_state['rws_momentum_z'] = wheel_inertia * (self.adcs_state['rws_rpm_z'] * 2 * math.pi / 60.0)
            else:
                # ADCS disabled - wheels maintain their momentum (no natural decay)
                # Momentum stored in wheels persists until removed via thruster desaturation
                self.adcs_state['rws_momentum_x'] = wheel_inertia * (self.adcs_state['rws_rpm_x'] * 2 * math.pi / 60.0)
                self.adcs_state['rws_momentum_y'] = wheel_inertia * (self.adcs_state['rws_rpm_y'] * 2 * math.pi / 60.0)
                self.adcs_state['rws_momentum_z'] = wheel_inertia * (self.adcs_state['rws_rpm_z'] * 2 * math.pi / 60.0)
            
            # COMPOUND ANOMALY: PROP valve stuck open - continuous propellant leak
            if self.anomaly_states['prop_valve_stuck_open'] and self.prop_state['system_enabled']:
                # Uncontrolled propellant venting
                leak_rate = 0.01  # kg per cycle (much faster than normal burn)
                self.prop_state['propellant_mass'] -= leak_rate
                self.prop_state['propellant_mass'] = max(0.0, self.prop_state['propellant_mass'])
                
                # Uncontrolled thrust creates deltaV
                thrust_force = 1.0  # N
                spacecraft_mass = 12.0 + self.prop_state['propellant_mass']
                burn_duration = 0.1  # seconds
                delta_v = (thrust_force * burn_duration) / spacecraft_mass
                self.prop_state['total_delta_v'] += delta_v
                
                # Force valve to OPEN state
                self.prop_state['valve_open'] = True
                
    def _update_tcs(self, elapsed_time):
        """Update Thermal Control System"""
        with self.lock:
            # Update radiator deployment timer
            if self.tcs_state['radiator_deploying']:
                self.tcs_state['radiator_deploy_time'] += 1.0
                if self.tcs_state['radiator_deploy_time'] >= 10.0:
                    self.tcs_state['radiator_deploying'] = False
                    self.tcs_state['radiator_deployed'] = not self.tcs_state['radiator_deployed']
                    status = "deployed" if self.tcs_state['radiator_deployed'] else "stowed"
                    self.add_evr(f"TCS: Radiator {status}")
            
            # Update voltage and current based on heater usage
            # Each heater draws ~5W = ~0.42A at 12V
            # COMPOUND ANOMALY: TCS heaters stuck ON - force all heaters ON
            if self.anomaly_states['tcs_heaters_stuck_on']:
                self.tcs_state['heater_avi_on'] = True
                self.tcs_state['heater_eps_on'] = True
                self.tcs_state['heater_prop_on'] = True
                
            num_heaters_on = sum([
                self.tcs_state['heater_avi_on'],
                self.tcs_state['heater_eps_on'],
                self.tcs_state['heater_prop_on']
            ])
            self.tcs_state['voltage'] = self.eps_state['bus_voltage']
            power_draw = num_heaters_on * 5.0  # 5W per heater
            self.tcs_state['current'] = power_draw / self.eps_state['bus_voltage'] if self.eps_state['bus_voltage'] > 0 else 0.0
            
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
            battery_heat = (abs(self.eps_state['battery_a_current']) + abs(self.eps_state['battery_b_current'])) * 1.0
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
            # Propellant tank temperature with heater and radiative cooling
            base_prop_temp = 10.0  # Base operating temperature
            heater_heat_prop = 25.0 if self.tcs_state['heater_prop_on'] else 0.0
            cooling_prop = (self.prop_state['tank_temp'] - self.tcs_state['temp_external']) * 0.05
            
            target_prop = base_prop_temp + heater_heat_prop - cooling_prop
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
            # Update voltage and current (~3W = ~0.25A at 12V)
            self.cdh_state['voltage'] = self.eps_state['bus_voltage']
            self.cdh_state['current'] = 3.0 / self.eps_state['bus_voltage'] if self.eps_state['bus_voltage'] > 0 else 0.0
            
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
            # Update voltage and current (~2W = ~0.17A at 12V)
            self.ttc_state['voltage'] = self.eps_state['bus_voltage']
            self.ttc_state['current'] = 2.0 / self.eps_state['bus_voltage'] if self.eps_state['bus_voltage'] > 0 else 0.0
            
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
        """Update Propulsion System - Heated Catalyst Monopropellant"""
        with self.lock:
            # Standby mode: 5V electronics only (~0.5W = 0.1A at 5V)
            if self.prop_state['standby_mode']:
                self.prop_state['voltage_5v'] = 5.0
                self.prop_state['current_5v'] = 0.1
            else:
                self.prop_state['voltage_5v'] = 0.0
                self.prop_state['current_5v'] = 0.0
                # If standby is off, system must also be off
                if self.prop_state['system_enabled']:
                    self.prop_state['system_enabled'] = False
                    self.add_evr("PROP: System disabled (standby power lost)")
            
            # Full operation mode: VBat for heater and valves
            if not self.prop_state['system_enabled']:
                # System disabled - no VBat power
                self.prop_state['voltage_vbat'] = 0.0
                self.prop_state['current_vbat'] = 0.0
                self.prop_state['catalyst_heater_on'] = False
                # Catalyst cools down when heater off
                cooling_rate = (self.prop_state['catalyst_temp'] - self.prop_state['tank_temp']) * 0.05
                self.prop_state['catalyst_temp'] -= cooling_rate
                self.prop_state['catalyst_ready'] = False
                return
            
            # System enabled - calculate VBat power draw
            self.prop_state['voltage_vbat'] = self.eps_state['bus_voltage']
            
            # Catalyst heater power (25W when on, heats catalyst bed to 350°C)
            if self.prop_state['catalyst_heater_on']:
                heater_current = 25.0 / self.eps_state['bus_voltage'] if self.eps_state['bus_voltage'] > 0 else 0.0
            else:
                heater_current = 0.0
            
            # Valve actuator power (5W when valve open)
            if self.prop_state['valve_open']:
                valve_current = 5.0 / self.eps_state['bus_voltage'] if self.eps_state['bus_voltage'] > 0 else 0.0
            else:
                valve_current = 0.0
            
            self.prop_state['current_vbat'] = heater_current + valve_current
            
            # Update catalyst temperature
            if self.prop_state['catalyst_heater_on']:
                # Heating towards operating temperature (350°C)
                target_temp = 350.0
                heating_rate = (target_temp - self.prop_state['catalyst_temp']) * 0.02  # Slow warm-up
                self.prop_state['catalyst_temp'] += heating_rate
                
                # Check if catalyst is ready (300°C minimum)
                if self.prop_state['catalyst_temp'] >= 300.0:
                    if not self.prop_state['catalyst_ready']:
                        self.prop_state['catalyst_ready'] = True
                        self.add_evr("PROP: Catalyst bed ready for operation (T > 300°C)")
                else:
                    self.prop_state['catalyst_ready'] = False
            else:
                # Cooling down when heater off
                cooling_rate = (self.prop_state['catalyst_temp'] - self.prop_state['tank_temp']) * 0.05
                self.prop_state['catalyst_temp'] -= cooling_rate
                if self.prop_state['catalyst_ready'] and self.prop_state['catalyst_temp'] < 300.0:
                    self.prop_state['catalyst_ready'] = False
                    self.add_evr("PROP: Catalyst bed too cold - heater required")
            
            # Update tank pressure based on propellant mass
            if self.prop_state['propellant_mass'] > 0:
                self.prop_state['tank_pressure'] = 200.0 * (self.prop_state['propellant_mass'] / 0.5)
            else:
                self.prop_state['tank_pressure'] = 0.0
            
            # Manual valve operation (user-commanded maneuvers)
            # Automatic desaturation valve control is handled in _update_adcs()
            if self.prop_state['valve_open']:
                # Check if catalyst is ready
                if not self.prop_state['catalyst_ready']:
                    self.prop_state['valve_open'] = False
                    self.add_evr("PROP: Valve closed - catalyst not ready (T < 300°C)")
                    return
                
                if not self.prop_state['desat_valve_auto']:
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
            
            # Update voltage and current based on mode
            self.comm_state['voltage'] = self.eps_state['bus_voltage']
            if self.comm_state['mode'] == 'TX':
                # TX only: ~30W (higher power for transmit)
                self.comm_state['current'] = 30.0 / self.eps_state['bus_voltage'] if self.eps_state['bus_voltage'] > 0 else 0.0
            elif self.comm_state['mode'] == 'TX_RX':
                # Full duplex: ~35W (TX + RX simultaneously)
                self.comm_state['current'] = 35.0 / self.eps_state['bus_voltage'] if self.eps_state['bus_voltage'] > 0 else 0.0
            elif self.comm_state['mode'] == 'RX':
                # RX only: ~5W (receive chain active)
                self.comm_state['current'] = 5.0 / self.eps_state['bus_voltage'] if self.eps_state['bus_voltage'] > 0 else 0.0
            else:  # STANDBY
                # STANDBY: ~1W (minimal keep-alive)
                self.comm_state['current'] = 1.0 / self.eps_state['bus_voltage'] if self.eps_state['bus_voltage'] > 0 else 0.0
            
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
        
        Safe Mode Actions:
        1. Shutdown all PROP systems
        2. Turn off heaters that are not normally on (keep critical heaters based on temperature)
        3. Deploy solar arrays A and B
        4. Deploy radiator
        5. Set ADCS to SUN_POINT (if not faulted)
        6. Enable cross-charge
        7. Cycle COMM: RX 60s → TX 60s → TX_RX 30s (repeating)
        """
        with self.lock:
            self.add_evr("=== ENTERING SAFE MODE ===")
            
            # Set system to SAFE mode
            self.eps_state['mode'] = 'SAFE'
            self.operational_mode = 'SAFE'
            
            # 1. Shutdown all PROP systems
            self.prop_state['standby_mode'] = False
            self.prop_state['system_enabled'] = False
            self.prop_state['valve_open'] = False
            self.prop_state['catalyst_heater_on'] = False
            self.prop_state['catalyst_ready'] = False
            self.add_evr("SAFE MODE: PROP completely shutdown (standby, main power, heater off)")
            
            # 2. Turn off heaters that are not normally on (only turn off if temps safe)
            # Keep heaters ON if temperatures are dangerously low
            if self.tcs_state['temp_avi'] > 5.0:
                self.tcs_state['heater_avi_on'] = False
                self.add_evr("SAFE MODE: AVI heater off (temp safe)")
            else:
                self.add_evr("SAFE MODE: AVI heater kept on (temp critical)")
                
            if self.tcs_state['temp_eps'] > 0.0:
                self.tcs_state['heater_eps_on'] = False
                self.add_evr("SAFE MODE: EPS heater off (temp safe)")
            else:
                self.add_evr("SAFE MODE: EPS heater kept on (temp critical)")
                
            # PROP heater always off in safe mode (PROP is off)
            self.tcs_state['heater_prop_on'] = False
            self.add_evr("SAFE MODE: PROP heater off")
            
            # 3. Deploy solar arrays if not already deployed
            if not self.eps_state['solar_array_a_deployed']:
                self.eps_state['solar_array_a_deploying'] = True
                self.eps_state['solar_array_a_deploy_time'] = 0.0
                self.add_evr("SAFE MODE: Deploying Solar Array A")
            
            if not self.eps_state['solar_array_b_deployed']:
                self.eps_state['solar_array_b_deploying'] = True
                self.eps_state['solar_array_b_deploy_time'] = 0.0
                self.add_evr("SAFE MODE: Deploying Solar Array B")
            
            # 4. Deploy radiator if not already deployed
            if not self.tcs_state['radiator_deployed']:
                self.tcs_state['radiator_deploying'] = True
                self.tcs_state['radiator_deploy_time'] = 0.0
                self.add_evr("SAFE MODE: Deploying radiator")
            
            # 5. Set ADCS to SUN_POINT (unless ADCS is faulted)
            if self.adcs_state['faulted']:
                self.adcs_state['enabled'] = False
                self.add_evr("SAFE MODE: ADCS disabled (FAULTED)")
            else:
                self.adcs_state['enabled'] = True
                self.adcs_state['mode'] = 'SUN_POINT'
                self.add_evr("SAFE MODE: ADCS enabled in SUN_POINT mode (maximize power)")
            
            # 6. Enable cross-charge for redundancy
            self.eps_state['cross_charging_enabled'] = True
            self.add_evr("SAFE MODE: Cross-charging ENABLED (redundant power path)")
            
            # 7. Initialize safe mode COMM cycling (RX 60s → TX 60s → TX_RX 30s)
            self.comm_state['mode'] = 'RX'  # Start with RX
            self.safe_mode_comm_timer = 0.0
            self.safe_mode_comm_phase = 0  # 0=RX, 1=TX, 2=TX_RX
            self.add_evr("SAFE MODE: COMM cycling initiated (60s RX → 60s TX → 30s TX_RX, repeating)")
            
            # Reset error counters
            self.cdh_state['error_count'] = 0
            
            self.add_evr("=== SAFE MODE ACTIVE - Awaiting ground intervention ===")
            
    def _update_star_tracker(self, elapsed_time):
        """Update Star Tracker navigation sensor - true attitude sensor with initialization"""
        with self.lock:
            if self.star_tracker_state['enabled'] and not self.star_tracker_state['faulted']:
                # Update voltage and current (2.5W at 12V = ~0.21A)
                self.star_tracker_state['voltage'] = self.eps_state['bus_voltage']
                self.star_tracker_state['current'] = 2.5 / self.eps_state['bus_voltage'] if self.eps_state['bus_voltage'] > 0 else 0.0
                
                # Handle initialization period
                if self.star_tracker_state['init_time_remaining'] > 0:
                    # Still initializing - count down timer
                    self.star_tracker_state['init_time_remaining'] -= 1.0  # 1 second per cycle
                    self.star_tracker_state['num_stars_tracked'] = 0
                    self.star_tracker_state['attitude_valid'] = False
                    self.star_tracker_state['attitude_accuracy'] = 0.0
                    self.star_tracker_state['roll'] = 0.0
                    self.star_tracker_state['pitch'] = 0.0
                    self.star_tracker_state['yaw'] = 0.0
                    
                    # Log completion of initialization
                    if self.star_tracker_state['init_time_remaining'] <= 0:
                        self.star_tracker_state['init_time_remaining'] = 0.0
                        self.add_evr("STAR_TRACKER: Initialization complete - attitude measurement active")
                else:
                    # Initialization complete - provide attitude measurements
                    # Star tracker measures actual spacecraft attitude with high precision
                    rate_error = abs(self.adcs_state['roll_rate']) + abs(self.adcs_state['pitch_rate']) + abs(self.adcs_state['yaw_rate'])
                    
                    if rate_error < 0.5:
                        # Low angular rates - excellent tracking
                        self.star_tracker_state['num_stars_tracked'] = random.randint(6, 8)
                        self.star_tracker_state['attitude_valid'] = True
                        self.star_tracker_state['attitude_accuracy'] = random.uniform(0.5, 2.0)  # arcseconds
                        
                        # Measure actual attitude with small noise
                        noise_roll = random.uniform(-0.01, 0.01)  # ±0.01° = ~36 arcsec max
                        noise_pitch = random.uniform(-0.01, 0.01)
                        noise_yaw = random.uniform(-0.01, 0.01)
                        
                        self.star_tracker_state['roll'] = self.adcs_state['roll'] + noise_roll
                        self.star_tracker_state['pitch'] = self.adcs_state['pitch'] + noise_pitch
                        self.star_tracker_state['yaw'] = self.adcs_state['yaw'] + noise_yaw
                        
                    elif rate_error < 2.0:
                        # Moderate rates - good tracking
                        self.star_tracker_state['num_stars_tracked'] = random.randint(4, 6)
                        self.star_tracker_state['attitude_valid'] = True
                        self.star_tracker_state['attitude_accuracy'] = random.uniform(2.0, 5.0)  # arcseconds
                        
                        # Measure with moderate noise
                        noise_roll = random.uniform(-0.05, 0.05)  # ±0.05° = ~180 arcsec max
                        noise_pitch = random.uniform(-0.05, 0.05)
                        noise_yaw = random.uniform(-0.05, 0.05)
                        
                        self.star_tracker_state['roll'] = self.adcs_state['roll'] + noise_roll
                        self.star_tracker_state['pitch'] = self.adcs_state['pitch'] + noise_pitch
                        self.star_tracker_state['yaw'] = self.adcs_state['yaw'] + noise_yaw
                        
                    elif rate_error < 5.0:
                        # High rates - degraded tracking
                        self.star_tracker_state['num_stars_tracked'] = random.randint(2, 4)
                        self.star_tracker_state['attitude_valid'] = True
                        self.star_tracker_state['attitude_accuracy'] = random.uniform(5.0, 15.0)  # arcseconds
                        
                        # Measure with higher noise
                        noise_roll = random.uniform(-0.2, 0.2)  # ±0.2° 
                        noise_pitch = random.uniform(-0.2, 0.2)
                        noise_yaw = random.uniform(-0.2, 0.2)
                        
                        self.star_tracker_state['roll'] = self.adcs_state['roll'] + noise_roll
                        self.star_tracker_state['pitch'] = self.adcs_state['pitch'] + noise_pitch
                        self.star_tracker_state['yaw'] = self.adcs_state['yaw'] + noise_yaw
                    else:
                        # Excessive tumbling - cannot track
                        self.star_tracker_state['num_stars_tracked'] = random.randint(0, 2)
                        self.star_tracker_state['attitude_valid'] = False
                        self.star_tracker_state['attitude_accuracy'] = 0.0
                        self.star_tracker_state['roll'] = 0.0
                        self.star_tracker_state['pitch'] = 0.0
                        self.star_tracker_state['yaw'] = 0.0
                
                # Temperature (warmer when powered)
                target_temp = 30.0 if not self.orbit_state['eclipse'] else 10.0
                self.star_tracker_state['sensor_temp'] += (target_temp - self.star_tracker_state['sensor_temp']) * 0.1
            else:
                # Disabled or faulted - reset initialization timer
                self.star_tracker_state['voltage'] = 0.0
                self.star_tracker_state['current'] = 0.0
                self.star_tracker_state['num_stars_tracked'] = 0
                self.star_tracker_state['attitude_valid'] = False
                self.star_tracker_state['attitude_accuracy'] = 0.0
                self.star_tracker_state['init_time_remaining'] = 180.0  # Reset to 3 minutes
                self.star_tracker_state['roll'] = 0.0
                self.star_tracker_state['pitch'] = 0.0
                self.star_tracker_state['yaw'] = 0.0
                
                # Temperature drifts toward ambient
                ambient_temp = -20.0 if self.orbit_state['eclipse'] else 5.0
                self.star_tracker_state['sensor_temp'] += (ambient_temp - self.star_tracker_state['sensor_temp']) * 0.05
    
    def _update_gps(self, elapsed_time):
        """Update GPS navigation sensor"""
        with self.lock:
            if self.gps_state['enabled'] and not self.gps_state['faulted']:
                # Update voltage and current (3.0W at 12V = ~0.25A)
                self.gps_state['voltage'] = self.eps_state['bus_voltage']
                self.gps_state['current'] = 3.0 / self.eps_state['bus_voltage'] if self.eps_state['bus_voltage'] > 0 else 0.0
                
                # Simulate GPS satellite acquisition (LEO GPS is challenging)
                if self.gps_state['time_to_first_fix'] < 180.0:
                    # Still acquiring
                    self.gps_state['time_to_first_fix'] += 1.0
                    self.gps_state['num_satellites'] = min(int(self.gps_state['time_to_first_fix'] / 30.0), 4)
                    self.gps_state['position_valid'] = False
                    self.gps_state['position_accuracy'] = 0.0
                    self.gps_state['velocity_accuracy'] = 0.0
                else:
                    # Fix acquired
                    self.gps_state['num_satellites'] = random.randint(4, 8)
                    self.gps_state['position_valid'] = True
                    # LEO GPS accuracy (worse than ground due to geometry)
                    self.gps_state['position_accuracy'] = random.uniform(15.0, 50.0)  # meters
                    self.gps_state['velocity_accuracy'] = random.uniform(0.1, 0.5)  # m/s
                
                # Temperature (warmer when powered)
                target_temp = 35.0 if not self.orbit_state['eclipse'] else 15.0
                self.gps_state['receiver_temp'] += (target_temp - self.gps_state['receiver_temp']) * 0.1
            else:
                # Disabled or faulted
                self.gps_state['voltage'] = 0.0
                self.gps_state['current'] = 0.0
                self.gps_state['num_satellites'] = 0
                self.gps_state['position_valid'] = False
                self.gps_state['position_accuracy'] = 0.0
                self.gps_state['velocity_accuracy'] = 0.0
                self.gps_state['time_to_first_fix'] = 0.0
                
                # Temperature drifts toward ambient
                ambient_temp = -20.0 if self.orbit_state['eclipse'] else 5.0
                self.gps_state['receiver_temp'] += (ambient_temp - self.gps_state['receiver_temp']) * 0.05
    
    def _check_cautions_warnings(self):
        """Check for caution and warning conditions"""
        with self.lock:
            self.cautions = []
            self.warnings = []
            
            # === EPS (Electrical Power System) ===
            # Battery A Voltage
            if self.eps_state['battery_a_voltage'] < self.thresholds['battery_voltage_min_warn']:
                self.warnings.append("Battery A voltage critical low")
                self.anomaly_states['battery_low'] = True
            elif self.eps_state['battery_a_voltage'] < self.thresholds['battery_voltage_min_caution']:
                self.cautions.append("Battery A voltage low")
            
            # Battery B Voltage
            if self.eps_state['battery_b_voltage'] < self.thresholds['battery_voltage_min_warn']:
                self.warnings.append("Battery B voltage critical low")
                self.anomaly_states['battery_low'] = True
            elif self.eps_state['battery_b_voltage'] < self.thresholds['battery_voltage_min_caution']:
                self.cautions.append("Battery B voltage low")
                
            # Battery A SOC (100% is allowed)
            if self.eps_state['battery_a_soc'] < self.thresholds['battery_soc_min_warn']:
                self.warnings.append("Battery A SOC critical")
                self.anomaly_states['battery_low'] = True
            elif self.eps_state['battery_a_soc'] < self.thresholds['battery_soc_min_caution']:
                self.cautions.append("Battery A SOC low")
            
            # Battery B SOC
            if self.eps_state['battery_b_soc'] < self.thresholds['battery_soc_min_warn']:
                self.warnings.append("Battery B SOC critical")
                self.anomaly_states['battery_low'] = True
            elif self.eps_state['battery_b_soc'] < self.thresholds['battery_soc_min_caution']:
                self.cautions.append("Battery B SOC low")
                
            # Battery A Temperature
            if self.eps_state['battery_a_temp'] > self.thresholds['battery_temp_max_warn']:
                self.warnings.append("Battery A temperature high")
                self.anomaly_states['eps_overtemp'] = True
            elif self.eps_state['battery_a_temp'] > self.thresholds['battery_temp_max_caution']:
                self.cautions.append("Battery A temperature elevated")
            if self.eps_state['battery_a_temp'] < self.thresholds['battery_temp_min_warn']:
                self.warnings.append("Battery A temperature critical low")
            elif self.eps_state['battery_a_temp'] < self.thresholds['battery_temp_min_caution']:
                self.cautions.append("Battery A temperature low")
            
            # Battery B Temperature
            if self.eps_state['battery_b_temp'] > self.thresholds['battery_temp_max_warn']:
                self.warnings.append("Battery B temperature high")
                self.anomaly_states['eps_overtemp'] = True
            elif self.eps_state['battery_b_temp'] > self.thresholds['battery_temp_max_caution']:
                self.cautions.append("Battery B temperature elevated")
            if self.eps_state['battery_b_temp'] < self.thresholds['battery_temp_min_warn']:
                self.warnings.append("Battery B temperature critical low")
            elif self.eps_state['battery_b_temp'] < self.thresholds['battery_temp_min_caution']:
                self.cautions.append("Battery B temperature low")
                
            # Battery A Current
            if abs(self.eps_state['battery_a_current']) > self.thresholds['battery_current_max_warn']:
                self.warnings.append("Battery A current exceeds maximum")
            elif abs(self.eps_state['battery_a_current']) > self.thresholds['battery_current_max_caution']:
                self.cautions.append("Battery A current high")
            
            # Battery B Current
            if abs(self.eps_state['battery_b_current']) > self.thresholds['battery_current_max_warn']:
                self.warnings.append("Battery B current exceeds maximum")
            elif abs(self.eps_state['battery_b_current']) > self.thresholds['battery_current_max_caution']:
                self.cautions.append("Battery B current high")
                
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
                    # SUN_POINT: Point -X axis (solar panels) at sun
                    # Target attitude is roll=180°, pitch=0°, yaw=0°
                    target_roll = 180.0
                    target_pitch = 0.0
                    target_yaw = 0.0
                    
                    # Calculate errors with angle wrapping
                    roll_error = abs((self.adcs_state['roll'] - target_roll + 180) % 360 - 180)
                    pitch_error = abs((self.adcs_state['pitch'] - target_pitch + 180) % 360 - 180)
                    yaw_error = abs((self.adcs_state['yaw'] - target_yaw + 180) % 360 - 180)
                    max_error = max(roll_error, pitch_error, yaw_error)
                    
                    if max_error > self.thresholds['attitude_sun_point_warn']:
                        self.warnings.append(f"Sun pointing error large ({max_error:.1f}°)")
                        self.anomaly_states['attitude_deviation'] = True
                    elif max_error > self.thresholds['attitude_sun_point_caution']:
                        self.cautions.append(f"Sun pointing error moderate ({max_error:.1f}°)")
                        
                elif self.adcs_state['mode'] == 'NADIR':
                    # NADIR: Point +X axis at Earth (nadir direction)
                    # Target attitude is roll=0°, pitch=-90°, yaw=0°
                    target_roll = 0.0
                    target_pitch = -90.0
                    target_yaw = 0.0
                    
                    # Calculate errors with angle wrapping
                    roll_error = abs((self.adcs_state['roll'] - target_roll + 180) % 360 - 180)
                    pitch_error = abs((self.adcs_state['pitch'] - target_pitch + 180) % 360 - 180)
                    yaw_error = abs((self.adcs_state['yaw'] - target_yaw + 180) % 360 - 180)
                    max_error = max(roll_error, pitch_error, yaw_error)
                    
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
            success = self._execute_command(command)
            # Store last command result for GUI to check
            self.last_command_success = success
            self.last_command = command
            
    def _execute_command(self, command):
        """Execute a spacecraft command - returns True if accepted, False if rejected"""
        # EXPERIMENTAL FAULT: CDH or AVI faulted - reject ALL commands except power cycle
        if self.experimental_fault_active and self.experimental_fault_type in ['CDH', 'AVI']:
            if command not in ['CDH_POWER_CYCLE', 'AVI_POWER_CYCLE']:
                self.add_evr(f"COMMAND REJECTED - {self.experimental_fault_type} FAULTED: {command} (power cycle required)")
                return False
        
        # EXPERIMENTAL FAULT: TTC faulted - track commands sent without telemetry
        if self.experimental_fault_active and self.experimental_fault_type == 'TTC':
            # Check if the command is for a subsystem whose telemetry is stale
            command_prefix = command.split('_')[0]  # Get subsystem prefix (ADCS, EPS, COMM, etc.)
            if command_prefix.lower() in self.experimental_fault_stale_subsystems:
                self.commanding_without_telemetry_count += 1
                self.add_evr(f"WARNING: Commanding {command_prefix} without telemetry (count: {self.commanding_without_telemetry_count})")
        
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
            if self.adcs_state['faulted']:
                self.add_evr("ADCS_NADIR rejected - ADCS faulted (power cycle required)")
            else:
                self.adcs_state['enabled'] = True
                self.adcs_state['mode'] = 'NADIR'
        elif command == "ADCS_SUN_POINT":
            if self.adcs_state['faulted']:
                self.add_evr("ADCS_SUN_POINT rejected - ADCS faulted (power cycle required)")
            else:
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
            if self.adcs_state['faulted']:
                self.add_evr("ADCS_INERTIAL rejected - ADCS faulted (power cycle required)")
            else:
                self.adcs_state['enabled'] = True
                self.adcs_state['mode'] = 'INERTIAL'
        elif command == "ADCS_DETUMBLE":
            if self.adcs_state['faulted']:
                self.add_evr("ADCS_DETUMBLE rejected - ADCS faulted (power cycle required)")
            else:
                self.adcs_state['enabled'] = True
                self.adcs_state['mode'] = 'DETUMBLE'
        elif command.startswith("ADCS_GO_TO"):
            # ADCS_GO_TO <roll> <pitch> <yaw>
            # Example: ADCS_GO_TO 45 30 -15
            # Slews spacecraft to specified attitude and holds
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
                    self.add_evr(f"ADCS: Slewing to R={target_roll:.1f}° P={target_pitch:.1f}° Y={target_yaw:.1f}°")
                except ValueError:
                    self.add_evr("ERROR: ADCS_GO_TO requires 3 numeric arguments (roll pitch yaw)")
            else:
                self.add_evr("ERROR: ADCS_GO_TO format: ADCS_GO_TO <roll> <pitch> <yaw>")
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
            # Reset attitude and rates to zero (calibration/initialization)
            self.adcs_state['roll'] = 0.0
            self.adcs_state['pitch'] = 0.0
            self.adcs_state['yaw'] = 0.0
            self.adcs_state['roll_rate'] = 0.0
            self.adcs_state['pitch_rate'] = 0.0
            self.adcs_state['yaw_rate'] = 0.0
            self.add_evr("ADCS attitude and rates reset to zero")
        elif command == "ADCS_POWER_CYCLE":
            # Power cycle ADCS subsystem - clears fault flag and resets all state
            self.adcs_state['enabled'] = False
            self.adcs_state['faulted'] = False
            self.adcs_state['mode'] = 'DETUMBLE'
            self.adcs_state['rws_rpm_x'] = 0.0
            self.adcs_state['rws_rpm_y'] = 0.0
            self.adcs_state['rws_rpm_z'] = 0.0
            self.adcs_state['rws_momentum_x'] = 0.0
            self.adcs_state['rws_momentum_y'] = 0.0
            self.adcs_state['rws_momentum_z'] = 0.0
            self.adcs_state['rws_saturated_x'] = False
            self.adcs_state['rws_saturated_y'] = False
            self.adcs_state['rws_saturated_z'] = False
            self.adcs_state['rws_desaturation_active'] = False
            self.adcs_state['desat_slewing'] = False
            self.adcs_state['desat_attitude_reached'] = False
            self.adcs_state['control_lost'] = False
            self.adcs_state['voltage'] = 0.0
            self.adcs_state['current'] = 0.0
            # Clear compound anomaly tumble rates
            self.adcs_state['roll_rate'] = 0.0
            self.adcs_state['pitch_rate'] = 0.0
            self.adcs_state['yaw_rate'] = 0.0
            self.add_evr("ADCS power cycled - fault cleared, system disabled")
        elif command == "ADCS_DESAT":
            # Manual desaturation command - triggers auto-desat sequence
            # This command initiates the same desaturation sequence as auto-desat:
            # 1. Slews to desaturation attitude based on momentum vector
            # 2. Fires thrusters to reduce wheel speeds to 500 RPM
            if not self.adcs_state['enabled']:
                self.add_evr("ADCS desaturation failed - ADCS not enabled")
            elif self.adcs_state['faulted']:
                self.add_evr("ADCS desaturation failed - ADCS faulted (power cycle required)")
            elif not self.prop_state['system_enabled']:
                self.add_evr("ADCS desaturation failed - PROP not enabled")
            elif not self.prop_state['catalyst_ready']:
                self.add_evr("ADCS desaturation failed - catalyst not ready (need 300°C+)")
            elif self.prop_state['propellant_mass'] < 0.01:
                self.add_evr("ADCS desaturation failed - insufficient fuel")
            elif self.adcs_state['rws_desaturation_active']:
                self.add_evr("ADCS desaturation already in progress")
            else:
                # Check if any wheel needs desaturation
                needs_desat_x = abs(self.adcs_state['rws_rpm_x']) > 1100
                needs_desat_y = abs(self.adcs_state['rws_rpm_y']) > 1100
                needs_desat_z = abs(self.adcs_state['rws_rpm_z']) > 1100
                
                if not (needs_desat_x or needs_desat_y or needs_desat_z):
                    self.add_evr("ADCS_DESAT: No wheels require desaturation (all < 1100 RPM)")
                else:
                    # Manually trigger desaturation sequence
                    # Store previous mode to restore after desaturation
                    if not hasattr(self.adcs_state, 'pre_desat_mode'):
                        self.adcs_state['pre_desat_mode'] = self.adcs_state['mode']
                    self.adcs_state['mode'] = 'DESAT'
                    self.adcs_state['rws_desaturation_active'] = True
                    self.adcs_state['desat_slewing'] = True
                    self.adcs_state['desat_attitude_reached'] = False
                    axes_desat = []
                    if needs_desat_x: axes_desat.append('X')
                    if needs_desat_y: axes_desat.append('Y')
                    if needs_desat_z: axes_desat.append('Z')
                    self.add_evr(f"Manual RWS desaturation initiated - axes: {', '.join(axes_desat)}")
                    self.add_evr("Target: 500 RPM ± 100 RPM")
        elif command == "ADCS_RWS_DESAT":
            # Manual reaction wheel desaturation command
            # User must have PROP enabled AND catalyst ready for this to work
            if (self.prop_state['system_enabled'] and 
                self.prop_state['catalyst_ready'] and 
                self.prop_state['propellant_mass'] > 0.05):
                # Reduce all wheel speeds to nominal range (500 RPM)
                target_rpm = 500.0
                self.adcs_state['rws_rpm_x'] = target_rpm if self.adcs_state['rws_rpm_x'] > 0 else -target_rpm
                self.adcs_state['rws_rpm_y'] = target_rpm if self.adcs_state['rws_rpm_y'] > 0 else -target_rpm
                self.adcs_state['rws_rpm_z'] = target_rpm if self.adcs_state['rws_rpm_z'] > 0 else -target_rpm
                # Consume propellant for maneuver
                self.prop_state['propellant_mass'] -= 0.02  # kg
                self.adcs_state['control_lost'] = False
                self.add_evr("RWS desaturation complete - wheels at nominal RPM")
            elif not self.prop_state['system_enabled']:
                self.add_evr("RWS desaturation failed - PROP not enabled")
            elif not self.prop_state['catalyst_ready']:
                self.add_evr("RWS desaturation failed - catalyst not ready (need 300°C+)")
            else:
                self.add_evr("RWS desaturation failed - insufficient fuel")
            
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
            # Reset EPS battery parameters to nominal values
            self.eps_state['battery_a_voltage'] = 11.8
            self.eps_state['battery_a_soc'] = 85.0
            self.eps_state['battery_a_current'] = 0.0
            self.eps_state['battery_a_temp'] = 15.0
            self.eps_state['battery_b_voltage'] = 11.8
            self.eps_state['battery_b_soc'] = 85.0
            self.eps_state['battery_b_current'] = 0.0
            self.eps_state['battery_b_temp'] = 15.0
            self.eps_state['bus_voltage'] = 11.8
            self.anomaly_states['battery_low'] = False
            self.add_evr("EPS reset - batteries restored to 85% SOC, 11.8V")
        elif command == "EPS_POWER_CYCLE":
            # Power cycle EPS subsystem - clears fault flag and compound anomalies
            self.eps_state['faulted'] = False
            self.eps_state['mode'] = 'NOMINAL'
            # Clear compound anomaly flags
            self.anomaly_states['battery_a_drain'] = False
            self.anomaly_states['battery_b_drain'] = False
            self.anomaly_states['solar_array_a_fault'] = False
            self.anomaly_states['solar_array_b_fault'] = False
            # Reset array status if faulted
            if self.eps_state['solar_array_a_status'] == 'FAULTED':
                self.eps_state['solar_array_a_status'] = 'NOMINAL'
            if self.eps_state['solar_array_b_status'] == 'FAULTED':
                self.eps_state['solar_array_b_status'] = 'NOMINAL'
            self.add_evr("EPS power cycled - fault cleared, compound anomalies cleared")
        elif command == "EPS_DEPLOY_ARRAY_A":
            if self.eps_state['solar_array_a_deployed']:
                self.add_evr("EPS: Solar Array A already deployed")
            elif self.eps_state['solar_array_a_deploying']:
                self.add_evr("EPS: Solar Array A deployment in progress")
            else:
                self.eps_state['solar_array_a_deploying'] = True
                self.eps_state['solar_array_a_deploy_time'] = 0.0
                self.add_evr("EPS: Solar Array A deployment started (10s)")
        elif command == "EPS_UNDEPLOY_ARRAY_A":
            if not self.eps_state['solar_array_a_deployed']:
                self.add_evr("EPS: Solar Array A already stowed")
            elif self.eps_state['solar_array_a_deploying']:
                self.add_evr("EPS: Solar Array A stowing in progress")
            else:
                self.eps_state['solar_array_a_deploying'] = True
                self.eps_state['solar_array_a_deploy_time'] = 0.0
                self.add_evr("EPS: Solar Array A stowing started (10s)")
        elif command == "EPS_DEPLOY_ARRAY_B":
            if self.eps_state['solar_array_b_deployed']:
                self.add_evr("EPS: Solar Array B already deployed")
            elif self.eps_state['solar_array_b_deploying']:
                self.add_evr("EPS: Solar Array B deployment in progress")
            else:
                self.eps_state['solar_array_b_deploying'] = True
                self.eps_state['solar_array_b_deploy_time'] = 0.0
                self.add_evr("EPS: Solar Array B deployment started (10s)")
        elif command == "EPS_UNDEPLOY_ARRAY_B":
            if not self.eps_state['solar_array_b_deployed']:
                self.add_evr("EPS: Solar Array B already stowed")
            elif self.eps_state['solar_array_b_deploying']:
                self.add_evr("EPS: Solar Array B stowing in progress")
            else:
                self.eps_state['solar_array_b_deploying'] = True
                self.eps_state['solar_array_b_deploy_time'] = 0.0
                self.add_evr("EPS: Solar Array B stowing started (10s)")
        elif command == "TCS_DEPLOY_RADIATOR":
            if self.tcs_state['radiator_deployed']:
                self.add_evr("TCS: Radiator already deployed")
            elif self.tcs_state['radiator_deploying']:
                self.add_evr("TCS: Radiator deployment in progress")
            else:
                self.tcs_state['radiator_deploying'] = True
                self.tcs_state['radiator_deploy_time'] = 0.0
                self.add_evr("TCS: Radiator deployment started (10s)")
        elif command == "TCS_UNDEPLOY_RADIATOR":
            if not self.tcs_state['radiator_deployed']:
                self.add_evr("TCS: Radiator already stowed")
            elif self.tcs_state['radiator_deploying']:
                self.add_evr("TCS: Radiator stowing in progress")
            else:
                self.tcs_state['radiator_deploying'] = True
                self.tcs_state['radiator_deploy_time'] = 0.0
                self.add_evr("TCS: Radiator stowing started (10s)")
        elif command == "EPS_CROSS_CHARGE_ENABLE":
            self.eps_state['cross_charging_enabled'] = True
            self.add_evr("EPS: Cross-charging enabled - Solar A/B can charge Battery A/B (redundant power path)")
        elif command == "EPS_CROSS_CHARGE_DISABLE":
            self.eps_state['cross_charging_enabled'] = False
            self.add_evr("EPS: Cross-charging disabled - Solar A charges Battery A, Solar B charges Battery B")
            
        # COMM commands (order matters - check TX_RX before TX!)
        elif command == "COMM_TX_RX":
            if self.comm_state['faulted']:
                self.add_evr("COMM_TX_RX rejected - COMM faulted (power cycle required)")
            else:
                self.comm_state['mode'] = 'TX_RX'
                self.comm_state['tx_timeout'] = 0.0
                self.comm_state['tx_time_remaining'] = 0.0
                self.add_evr("COMM: TX_RX mode enabled (full duplex - spacecraft COMMANDABLE and transmitting)")
            
            # Check if this exits safe mode
            if self.operational_mode == 'SAFE':
                self.operational_mode = 'NOMINAL'
                self.eps_state['mode'] = 'NOMINAL'
                self.safe_mode_comm_timer = 0.0
                self.safe_mode_comm_phase = 0
                self.add_evr("Safe mode EXITED - COMM set to TX_RX (full duplex)")
                
        elif command.startswith("COMM_TX"):
            if self.comm_state['faulted']:
                self.add_evr("COMM_TX rejected - COMM faulted (power cycle required)")
            else:
                # COMM_TX [timeout_seconds]
                # Example: COMM_TX 60 (TX for 60 seconds then return to RX)
                # Timeout is REQUIRED for safety
                parts = command.split()
                if len(parts) == 2:
                    try:
                        timeout = float(parts[1])
                        self.comm_state['tx_timeout'] = timeout
                        self.comm_state['tx_time_remaining'] = timeout
                        self.comm_state['mode'] = 'TX'
                        self.add_evr(f"COMM: TX mode enabled for {timeout:.0f} seconds (will auto-revert to RX)")
                    except ValueError:
                        self.add_evr("ERROR: COMM_TX timeout must be numeric. Usage: COMM_TX <seconds>")
                        return
                else:
                    self.add_evr("ERROR: COMM_TX requires timeout parameter. Usage: COMM_TX <seconds>")
                    return
            
        elif command == "COMM_RX":
            if self.comm_state['faulted']:
                self.add_evr("COMM_RX rejected - COMM faulted (power cycle required)")
            else:
                self.comm_state['mode'] = 'RX'
                self.comm_state['tx_timeout'] = 0.0
                self.comm_state['tx_time_remaining'] = 0.0
                self.add_evr("COMM: RX mode enabled (spacecraft COMMANDABLE, not transmitting)")
            
        elif command == "COMM_STANDBY":
            if self.comm_state['faulted']:
                self.add_evr("COMM_STANDBY rejected - COMM faulted (power cycle required)")
            else:
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
            # Reset COMM parameters to default values
            self.comm_state['mode'] = 'RX'  # Reset to RX mode (default)
            self.comm_state['tx_power'] = 30.0
            self.comm_state['temp'] = 30.0
            self.comm_state['packets_tx'] = 0
            self.comm_state['packets_rx'] = 0
            self.comm_state['tx_time_remaining'] = 0.0
            self.anomaly_states['comm_overtemp'] = False
            self.add_evr("COMM reset - mode RX, power 30W, temp reset")
        elif command == "COMM_POWER_CYCLE":
            # Power cycle COMM subsystem - clears fault flag and resets all state
            self.comm_state['faulted'] = False
            self.comm_state['mode'] = 'RX'
            self.comm_state['tx_power'] = 30.0
            self.comm_state['temp'] = 30.0
            self.add_evr("COMM power cycled - fault cleared, mode RX")
            self.comm_state['packets_tx'] = 0
            self.comm_state['packets_rx'] = 0
            self.comm_state['tx_time_remaining'] = 0.0
            self.comm_state['voltage'] = 12.0
            self.comm_state['current'] = 0.42  # RX mode current
            self.anomaly_states['comm_overtemp'] = False
            self.add_evr("COMM power cycled - fault cleared")
            
        # PROP commands - Heated Catalyst Monopropellant System
        elif command == "PROP_STANDBY":
            if self.prop_state['faulted']:
                self.add_evr("PROP_STANDBY rejected - PROP faulted (power cycle required)")
            else:
                self.prop_state['standby_mode'] = True
                self.add_evr("PROP: Standby mode enabled (5V electronics power on)")
        elif command == "PROP_ON":
            if self.prop_state['faulted']:
                self.add_evr("PROP_ON rejected - PROP faulted (power cycle required)")
            elif not self.prop_state['standby_mode']:
                self.add_evr("PROP_ON rejected - standby mode required first")
            else:
                self.prop_state['system_enabled'] = True
                self.add_evr("PROP: System enabled (VBat power on)")
        elif command == "PROP_OFF":
            self.prop_state['system_enabled'] = False
            self.prop_state['standby_mode'] = False
            self.prop_state['valve_open'] = False
            self.prop_state['catalyst_heater_on'] = False
            self.add_evr("PROP: System and standby disabled")
        elif command == "PROP_HEATER_ON":
            if self.prop_state['system_enabled']:
                self.prop_state['catalyst_heater_on'] = True
                self.add_evr("PROP: Catalyst heater ON (warming to 350°C)")
            else:
                self.add_evr("PROP_HEATER_ON rejected - system must be enabled (PROP_ON)")
        elif command == "PROP_HEATER_OFF":
            self.prop_state['catalyst_heater_on'] = False
            self.add_evr("PROP: Catalyst heater OFF")
        elif command == "PROP_POWER_CYCLE":
            # Power cycle PROP subsystem - clears fault flag and resets all state
            self.prop_state['faulted'] = False
            self.prop_state['standby_mode'] = False
            self.prop_state['system_enabled'] = False
            self.prop_state['valve_open'] = False
            self.prop_state['catalyst_heater_on'] = False
            self.prop_state['catalyst_ready'] = False
            self.prop_state['catalyst_temp'] = 20.0  # Reset to ambient
            self.prop_state['voltage_5v'] = 0.0
            self.prop_state['current_5v'] = 0.0
            self.prop_state['voltage_vbat'] = 0.0
            self.prop_state['current_vbat'] = 0.0
            self.prop_state['desat_valve_auto'] = False
            # Clear compound anomaly flag
            self.anomaly_states['prop_valve_stuck_open'] = False
            self.add_evr("PROP power cycled - fault cleared, all systems disabled")
        elif command == "PROP_VALVE_OPEN":
            if not self.prop_state['system_enabled']:
                self.add_evr("PROP_VALVE_OPEN rejected - system disabled")
            elif not self.prop_state['catalyst_ready']:
                self.add_evr("PROP_VALVE_OPEN rejected - catalyst not ready (T < 300°C)")
            else:
                self.prop_state['valve_open'] = True
                self.add_evr("PROP: Valve opened - thrust enabled")
        elif command == "PROP_VALVE_CLOSE":
            # COMPOUND ANOMALY: Valve stuck open - command ignored until power cycle
            if self.anomaly_states['prop_valve_stuck_open']:
                self.add_evr("PROP_VALVE_CLOSE rejected - valve mechanically stuck OPEN (power cycle required)")
            else:
                self.prop_state['valve_open'] = False
                self.add_evr("PROP: Valve closed")
            
        # TCS commands
        elif command == "TCS_HEATER_AVI_ON":
            self.tcs_state['heater_avi_on'] = True
        elif command == "TCS_HEATER_AVI_OFF":
            # COMPOUND ANOMALY: Heaters stuck ON - command ignored until power cycle
            if self.anomaly_states['tcs_heaters_stuck_on']:
                self.add_evr("TCS_HEATER_AVI_OFF rejected - heaters stuck ON (power cycle required)")
            else:
                self.tcs_state['heater_avi_on'] = False
        elif command == "TCS_HEATER_EPS_ON":
            self.tcs_state['heater_eps_on'] = True
        elif command == "TCS_HEATER_EPS_OFF":
            # COMPOUND ANOMALY: Heaters stuck ON - command ignored until power cycle
            if self.anomaly_states['tcs_heaters_stuck_on']:
                self.add_evr("TCS_HEATER_EPS_OFF rejected - heaters stuck ON (power cycle required)")
            else:
                self.tcs_state['heater_eps_on'] = False
        elif command == "TCS_HEATER_PROP_ON":
            self.tcs_state['heater_prop_on'] = True
        elif command == "TCS_HEATER_PROP_OFF":
            # COMPOUND ANOMALY: Heaters stuck ON - command ignored until power cycle
            if self.anomaly_states['tcs_heaters_stuck_on']:
                self.add_evr("TCS_HEATER_PROP_OFF rejected - heaters stuck ON (power cycle required)")
            else:
                self.tcs_state['heater_prop_on'] = False
        elif command == "TCS_POWER_CYCLE":
            # Power cycle TCS subsystem - clears fault flag and resets all state
            self.tcs_state['faulted'] = False
            self.tcs_state['heater_avi_on'] = False
            self.tcs_state['heater_eps_on'] = False
            self.tcs_state['heater_prop_on'] = False
            self.tcs_state['voltage'] = 12.0
            self.tcs_state['current'] = 0.0
            # Clear compound anomaly flag
            self.anomaly_states['tcs_heaters_stuck_on'] = False
            self.add_evr("TCS power cycled - fault cleared, all heaters off")
            
        # AVI commands
        elif command == "AVI_REBOOT":
            # Soft reboot - increment boot count and reset loads
            self.avi_state['boot_count'] += 1
            self.avi_state['cpu_load'] = 30.0
            self.avi_state['memory_usage'] = 40.0
            self.avi_state['uptime'] = 0.0
            self.add_evr("AVI rebooted - boot count incremented")
        elif command == "AVI_POWER_CYCLE":
            # Full power cycle - reset all AVI state
            self.avi_state['boot_count'] += 1
            self.avi_state['uptime'] = 0.0
            self.avi_state['cpu_load'] = 30.0
            self.avi_state['memory_usage'] = 40.0
            self.avi_state['cpu_temp'] = 25.0
            self.avi_state['voltage'] = 12.0
            self.avi_state['current'] = 1.25
            self.avi_state['faulted'] = False  # Clear AVI fault flag
            self.anomaly_states['avi_overtemp'] = False
            self.epoch_time = time.time()
            self.clear_experimental_fault()  # Clear experimental fault if active
            self.add_evr("AVI power cycled - all parameters reset")
        
        # CDH commands
        elif command == "CDH_POWER_CYCLE":
            # Power cycle CDH subsystem - clears fault flag and resets counters
            self.cdh_state['faulted'] = False
            self.cdh_state['error_count'] = 0
            self.cdh_state['cmd_count'] = 0
            self.cdh_state['tlm_count'] = 0
            self.cdh_state['voltage'] = 12.0
            self.cdh_state['current'] = 0.25
            self.anomaly_states['storage_full'] = False
            self.clear_experimental_fault()  # Clear experimental fault if active
            self.add_evr("CDH power cycled - fault cleared")
        
        # TTC commands
        elif command == "TTC_POWER_CYCLE":
            # Power cycle TTC subsystem - clears fault flag and resets all state
            self.ttc_state['faulted'] = False
            self.ttc_state['link_status'] = 'NO_LINK'
            self.ttc_state['signal_strength'] = -105.0
            self.ttc_state['uplink_rate'] = 0.0
            self.ttc_state['downlink_rate'] = 0.0
            self.ttc_state['pass_elevation'] = 0.0
            self.ttc_state['doppler_shift'] = 0.0
            self.ttc_state['voltage'] = 12.0
            self.ttc_state['current'] = 0.17
            self.add_evr("TTC power cycled - fault cleared")
            
        # Star Tracker commands
        elif command == "STAR_TRACKER_ON":
            if self.star_tracker_state['faulted']:
                self.add_evr("STAR_TRACKER_ON rejected - Star Tracker faulted (power cycle required)")
            else:
                self.star_tracker_state['enabled'] = True
                self.star_tracker_state['init_time_remaining'] = 180.0  # Reset 3-minute initialization timer
                self.add_evr("Star Tracker enabled - initializing (180s)")
        elif command == "STAR_TRACKER_OFF":
            self.star_tracker_state['enabled'] = False
            self.add_evr("Star Tracker disabled")
        elif command == "STAR_TRACKER_POWER_CYCLE":
            # Power cycle Star Tracker - clear fault and reset all state
            self.star_tracker_state['faulted'] = False
            self.star_tracker_state['enabled'] = False
            self.star_tracker_state['num_stars_tracked'] = 0
            self.star_tracker_state['attitude_valid'] = False
            self.star_tracker_state['attitude_accuracy'] = 0.0
            self.star_tracker_state['init_time_remaining'] = 180.0  # Reset initialization timer
            self.star_tracker_state['roll'] = 0.0
            self.star_tracker_state['pitch'] = 0.0
            self.star_tracker_state['yaw'] = 0.0
            self.star_tracker_state['voltage'] = 0.0
            self.star_tracker_state['current'] = 0.0
            self.add_evr("Star Tracker power cycled - fault cleared")
            
        # GPS commands
        elif command == "GPS_ON":
            if self.gps_state['faulted']:
                self.add_evr("GPS_ON rejected - GPS faulted (power cycle required)")
            else:
                self.gps_state['enabled'] = True
                self.gps_state['time_to_first_fix'] = 0.0
                self.add_evr("GPS enabled - acquiring satellites")
        elif command == "GPS_OFF":
            self.gps_state['enabled'] = False
            self.add_evr("GPS disabled")
        elif command == "GPS_POWER_CYCLE":
            # Power cycle GPS - clear fault and reset all state
            self.gps_state['faulted'] = False
            self.gps_state['enabled'] = False
            self.gps_state['num_satellites'] = 0
            self.gps_state['position_valid'] = False
            self.gps_state['time_to_first_fix'] = 0.0
            self.gps_state['voltage'] = 0.0
            self.gps_state['current'] = 0.0
            self.add_evr("GPS power cycled - fault cleared")
            
        # System commands
        elif command == "FAULT_INJECTION_ENABLE":
            self.fault_injection_enabled = True
            self.fault_scheduled_time = None
            self.fault_injected = False
            self.add_evr("Fault injection ENABLED - fault will occur between 2-5 minutes")
        elif command == "FAULT_INJECTION_DISABLE":
            self.fault_injection_enabled = False
            self.add_evr("Fault injection DISABLED")
        elif command == "SAFE_MODE":
            self._enter_safe_mode()
        
        # ===== COMPOUND ANOMALY INJECTION COMMANDS (OFF-NOMINAL SCENARIO) =====
        elif command == "INJECT_COMPOUND_ADCS_FAULT":
            # ADCS faults during maneuver - spacecraft tumbles, telemetry stale except fault flag
            self.adcs_state['faulted'] = True
            self.adcs_state['enabled'] = False
            # Set high tumble rates
            self.adcs_state['roll_rate'] = random.uniform(5.0, 10.0)
            self.adcs_state['pitch_rate'] = random.uniform(5.0, 10.0)
            self.adcs_state['yaw_rate'] = random.uniform(5.0, 10.0)
            self.add_evr("COMPOUND ANOMALY: ADCS FAULT during maneuver - spacecraft tumbling")
        
        elif command == "INJECT_COMPOUND_BATTERY_DRAIN":
            # Random battery starts draining despite working solar arrays
            drain_battery = random.choice(['A', 'B'])
            self.anomaly_states[f'battery_{drain_battery.lower()}_drain'] = True
            self.add_evr(f"COMPOUND ANOMALY: Battery {drain_battery} draining despite solar arrays working")
        
        elif command == "INJECT_COMPOUND_SOLAR_FAULT":
            # Random solar array faults
            fault_array = random.choice(['A', 'B'])
            if fault_array == 'A':
                self.eps_state['solar_array_a_deployed'] = 'FAULTED'
                self.anomaly_states['solar_array_a_fault'] = True
            else:
                self.eps_state['solar_array_b_deployed'] = 'FAULTED'
                self.anomaly_states['solar_array_b_fault'] = True
            self.add_evr(f"COMPOUND ANOMALY: Solar Array {fault_array} FAULTED")
        
        elif command == "INJECT_COMPOUND_COMM_FAULT":
            # COMM faults - all telemetry goes stale, returns to RX mode
            self.comm_state['faulted'] = True
            self.comm_state['mode'] = 'RX'
            self.comm_state['link_status'] = 'NO_LINK'
            self.add_evr("COMPOUND ANOMALY: COMM FAULT - all telemetry stale, RX mode only")
        
        elif command == "INJECT_COMPOUND_PROP_FAULT":
            # PROP faults during maneuver - valve stuck open, propellant leaking
            self.prop_state['faulted'] = True
            self.prop_state['valve_status'] = 'OPEN'
            self.anomaly_states['prop_valve_stuck_open'] = True
            self.add_evr("COMPOUND ANOMALY: PROP FAULT - valve stuck OPEN, propellant leaking")
        
        elif command == "INJECT_COMPOUND_TCS_FAULT":
            # TCS faults - all heaters stuck ON
            self.tcs_state['faulted'] = True
            self.anomaly_states['tcs_heaters_stuck_on'] = True
            self.add_evr("COMPOUND ANOMALY: TCS FAULT - all heaters stuck ON")
        
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
        else:
            # Unknown command
            self.add_evr(f"COMMAND REJECTED: Unknown command '{command}'")
            return False
        
        # Command accepted and executed
        return True
                
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
        """Queue a command for execution - returns True if queued, False if rejected"""
        with self.lock:
            # Block commands if in TX-only mode (spacecraft cannot receive)
            if self.comm_state['mode'] == 'TX' and self.operational_mode != 'SAFE':
                self.add_evr(f"Command REJECTED: {command} - spacecraft in TX-only mode (not commandable)")
                return False
            
            self.command_queue.append(command)
            self.add_evr(f"Command queued: {command}")
            return True
            
    def get_evr_log(self):
        """Get EVR log"""
        with self.lock:
            return self.evr_log.copy()
            
    def get_cautions_warnings(self):
        """Get current cautions and warnings"""
        with self.lock:
            return self.cautions.copy(), self.warnings.copy()
            
    def get_telemetry(self):
        """Get current telemetry organized by subsystem - gated by COMM downlink mode"""
        with self.lock:
            # Create stale telemetry dict for disabled subsystems or when not downlinking
            def make_stale(state_dict):
                """Mark telemetry as stale by appending ' S' to each value"""
                stale_dict = {}
                for key, value in state_dict.items():
                    if isinstance(value, (int, float, bool)):
                        stale_dict[key] = f"{value} S"
                    else:
                        stale_dict[key] = f"{str(value)} S"
                return stale_dict
            
            # Get current telemetry from all subsystems
            current_telemetry = {
                'orbit': self.orbit_state.copy(),
                'eps': self.eps_state.copy(),
                'adcs': self.adcs_state.copy() if self.adcs_state['enabled'] else make_stale(self.adcs_state),
                'tcs': self.tcs_state.copy(),
                'avi': self.avi_state.copy(),
                'cdh': self.cdh_state.copy(),
                'ttc': self.ttc_state.copy(),
                'star_tracker': self.star_tracker_state.copy(),
                'gps': self.gps_state.copy(),
                'prop': self.prop_state.copy() if self.prop_state['standby_mode'] else make_stale(self.prop_state),
                'comm': self.comm_state.copy()
            }
            
            # EXPERIMENTAL FAULT: Apply stale telemetry based on fault type
            if self.experimental_fault_active:
                for subsystem in self.experimental_fault_stale_subsystems:
                    if subsystem in current_telemetry:
                        # Special case: For TTC fault, never make the TTC fault flag stale
                        if self.experimental_fault_type == 'TTC' and subsystem == 'ttc':
                            # Keep TTC fault flag visible, make everything else stale
                            temp_dict = current_telemetry['ttc'].copy()
                            fault_flag = temp_dict.get('faulted', False)
                            current_telemetry['ttc'] = make_stale(temp_dict)
                            current_telemetry['ttc']['faulted'] = fault_flag
                        else:
                            current_telemetry[subsystem] = make_stale(current_telemetry[subsystem])
            
            # COMPOUND ANOMALY: Apply stale telemetry based on compound fault types
            # ADCS compound fault - all ADCS telemetry stale except 'faulted' flag
            if self.adcs_state.get('faulted', False):
                temp_dict = current_telemetry['adcs'].copy()
                fault_flag = temp_dict.get('faulted', False)
                current_telemetry['adcs'] = make_stale(temp_dict)
                current_telemetry['adcs']['faulted'] = fault_flag
            
            # PROP compound fault - all PROP telemetry stale except 'faulted' flag
            if self.prop_state.get('faulted', False):
                temp_dict = current_telemetry['prop'].copy()
                fault_flag = temp_dict.get('faulted', False)
                current_telemetry['prop'] = make_stale(temp_dict)
                current_telemetry['prop']['faulted'] = fault_flag
            
            # COMM compound fault - ALL subsystems telemetry stale (complete telemetry loss)
            if self.comm_state.get('faulted', False):
                for subsystem in current_telemetry:
                    current_telemetry[subsystem] = make_stale(current_telemetry[subsystem])
            
            # Check if spacecraft is transmitting telemetry
            comm_mode = self.comm_state['mode']
            is_downlinking = comm_mode in ['TX', 'TX_RX']
            
            if is_downlinking:
                # Spacecraft is transmitting - update cache and return live telemetry
                self.last_telemetry_cache = current_telemetry
                return current_telemetry
            else:
                # Spacecraft NOT transmitting (RX or STANDBY mode) - return stale telemetry
                if not self.last_telemetry_cache:
                    # Initial state - no prior telemetry downlinked
                    # Mark all current telemetry as stale
                    stale_telemetry = {}
                    for subsystem, data in current_telemetry.items():
                        stale_telemetry[subsystem] = make_stale(data)
                    return stale_telemetry
                else:
                    # Return last downlinked values marked as stale
                    stale_telemetry = {}
                    for subsystem, data in self.last_telemetry_cache.items():
                        stale_telemetry[subsystem] = make_stale(data)
                    return stale_telemetry
            
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
                ('star_tracker', self.star_tracker_state),
                ('gps', self.gps_state),
                ('prop', self.prop_state),
                ('comm', self.comm_state)
            ]:
                # Check if this subsystem's telemetry is stale
                is_stale = subsys_name in self.experimental_fault_stale_subsystems
                
                for key, value in subsys_data.items():
                    channel_name = f"{subsys_name.upper()}_{key.upper()}"
                    if isinstance(value, float):
                        data = f"{value:.2f}"
                    elif isinstance(value, bool):
                        data = "YES" if value else "NO"
                    else:
                        data = str(value)
                    channels.append((channel_name, data, is_stale))
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
            print(f"Mode: {tlm['eps']['mode']}  Arrays: A={tlm['eps']['solar_array_a_deployed']} B={tlm['eps']['solar_array_b_deployed']}")
            print(f"Battery A: {tlm['eps']['battery_a_voltage']:.2f}V @ {tlm['eps']['battery_a_soc']:.1f}%")
            print(f"Battery B: {tlm['eps']['battery_b_voltage']:.2f}V @ {tlm['eps']['battery_b_soc']:.1f}%")
            print(f"Solar: A={tlm['eps']['solar_power_a']:.1f}W B={tlm['eps']['solar_power_b']:.1f}W  Bus: {tlm['eps']['bus_voltage']:.2f}V @ {tlm['eps']['bus_current']:.2f}A")
            print(f"Cross-Charging: {tlm['eps']['cross_charging_enabled']}")
            
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
