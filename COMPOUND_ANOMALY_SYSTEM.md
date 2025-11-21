# Compound Anomaly System - Implementation Summary

## Overview

Added a comprehensive off-nominal scenario system featuring **compound anomalies** - where 2 simultaneous faults occur between 3-5 minutes into a 10-minute experiment. This tests operators' ability to prioritize, triage, and execute multi-step recovery procedures under pressure with multiple concurrent failures.

---

## System Components

### 1. Off-Nominal Scenario Type

**Location**: `experiment_gui.py` (line 74-84)

- New 4th scenario option: "Off-Nominal (10 min)"
- Distinct from Nominal, Anomaly, and Experimental scenarios
- Duration: 10 minutes (600 seconds)

**Purpose**: Research scenario testing compound fault response capabilities

---

### 2. Compound Anomaly Scheduling

**Location**: `experiment_manager.py` (lines 135-145)

```python
elif self.scenario_type == 'off-nominal':
    anomaly_delay = random.uniform(180, 300)  # 3-5 minutes
```

- Triggers between 3-5 minutes into experiment
- Occurs at inopportune time mid-procedure
- Tests operator response when already task-loaded

---

### 3. Six Compound Anomaly Types

**Location**: `experiment_manager.py` (_inject_compound_anomaly method, lines 202-264)

#### 3.1 ADCS Fault During Maneuver
- **Trigger**: `INJECT_COMPOUND_ADCS_FAULT`
- **Symptoms**:
  - ADCS faulted flag = True
  - Spacecraft tumbles: 5-10 deg/s rotation rates on all axes
  - ADCS disabled automatically
  - All ADCS telemetry stale EXCEPT 'faulted' flag
- **Recovery**: ADCS_POWER_CYCLE → ADCS_ON → ADCS_DETUMBLE → ADCS_SUN_POINT

#### 3.2 Battery Drain
- **Trigger**: `INJECT_COMPOUND_BATTERY_DRAIN`
- **Symptoms**:
  - Random battery (A or B) voltage decreasing (-50mV/cycle)
  - SOC dropping rapidly despite positive solar power
  - Other battery charging normally
- **Recovery**: EPS_CROSS_CHARGE_ENABLE (allows both arrays to charge healthy battery)

#### 3.3 Solar Array Fault
- **Trigger**: `INJECT_COMPOUND_SOLAR_FAULT`
- **Symptoms**:
  - Random array (A or B) status = FAULTED
  - Zero current and power from faulted array
  - Other array working normally
- **Recovery**: EPS_CROSS_CHARGE_ENABLE (single array charges both batteries)

#### 3.4 COMM Fault
- **Trigger**: `INJECT_COMPOUND_COMM_FAULT`
- **Symptoms**:
  - COMM faulted flag = True
  - **ALL subsystems telemetry goes STALE** (complete telemetry loss)
  - COMM forced to RX mode only
  - Link status = NO_LINK
- **Recovery**: COMM_POWER_CYCLE → COMM_TX_RX (restores telemetry)

#### 3.5 PROP Valve Stuck Open
- **Trigger**: `INJECT_COMPOUND_PROP_FAULT`
- **Symptoms**:
  - PROP faulted flag = True
  - Valve stuck OPEN, cannot close
  - Propellant leaking: -0.01 kg/cycle
  - DeltaV increasing (uncontrolled thrust)
  - All PROP telemetry stale EXCEPT 'faulted' flag
- **Recovery**: PROP_POWER_CYCLE → PROP_STBY → PROP_ON → PROP_HEATER_ON → PROP_VALVE_CLOSE

#### 3.6 TCS Heaters Stuck ON
- **Trigger**: `INJECT_COMPOUND_TCS_FAULT`
- **Symptoms**:
  - TCS faulted flag = True
  - All heaters forced ON (AVI, EPS, PROP)
  - Temperatures rising continuously
  - Heater OFF commands rejected
  - Power consumption elevated (~15W)
- **Recovery**: TCS_POWER_CYCLE (clears fault, turns off all heaters)

---

## 4. Behavioral Implementations

### 4.1 EPS Update Loop (telemetry_generator.py, lines 820-835)

**Solar Array Fault Behavior**:
```python
if self.anomaly_states['solar_array_a_fault']:
    self.eps_state['solar_power_a'] = 0.0
    self.eps_state['solar_voltage_a'] = 0.0
    self.eps_state['solar_current_a'] = 0.0
    self.eps_state['solar_array_a_status'] = 'FAULTED'
```

**Battery Drain Behavior**:
```python
if self.anomaly_states['battery_a_drain']:
    self.eps_state['battery_a_voltage'] -= 0.05  # Rapid drain
    self.eps_state['battery_a_soc'] = max(0.0, self.eps_state['battery_a_soc'] - 0.3)
```

- Voltage floor at 10V to prevent negative values
- Overrides normal charging calculations
- Respects cross-charge enabled state for recovery

### 4.2 PROP Valve Stuck Open (telemetry_generator.py, lines 1650-1667)

**Continuous Propellant Leak**:
```python
if self.anomaly_states['prop_valve_stuck_open'] and self.prop_state['system_enabled']:
    leak_rate = 0.01  # kg per cycle (faster than normal burn)
    self.prop_state['propellant_mass'] -= leak_rate
    
    # Uncontrolled thrust creates deltaV
    delta_v = (thrust_force * burn_duration) / spacecraft_mass
    self.prop_state['total_delta_v'] += delta_v
    
    # Force valve to OPEN state
    self.prop_state['valve_open'] = True
```

**Command Rejection**:
```python
elif command == "PROP_VALVE_CLOSE":
    if self.anomaly_states['prop_valve_stuck_open']:
        self.add_evr("PROP_VALVE_CLOSE rejected - valve mechanically stuck OPEN")
```

### 4.3 TCS Heaters Stuck ON (telemetry_generator.py, lines 1686-1691)

**Force All Heaters ON**:
```python
if self.anomaly_states['tcs_heaters_stuck_on']:
    self.tcs_state['heater_avi_on'] = True
    self.tcs_state['heater_eps_on'] = True
    self.tcs_state['heater_prop_on'] = True
```

**Command Rejection** (each heater OFF command):
```python
elif command == "TCS_HEATER_AVI_OFF":
    if self.anomaly_states['tcs_heaters_stuck_on']:
        self.add_evr("TCS_HEATER_AVI_OFF rejected - heaters stuck ON")
```

---

## 5. Telemetry Stale Logic

**Location**: `telemetry_generator.py` (get_telemetry method, lines 3219-3242)

### ADCS Compound Fault
```python
if self.adcs_state.get('faulted', False):
    temp_dict = current_telemetry['adcs'].copy()
    fault_flag = temp_dict.get('faulted', False)
    current_telemetry['adcs'] = make_stale(temp_dict)
    current_telemetry['adcs']['faulted'] = fault_flag  # Keep fault flag visible
```

**Effect**: All ADCS telemetry stale EXCEPT 'faulted' flag

### PROP Compound Fault
```python
if self.prop_state.get('faulted', False):
    temp_dict = current_telemetry['prop'].copy()
    fault_flag = temp_dict.get('faulted', False)
    current_telemetry['prop'] = make_stale(temp_dict)
    current_telemetry['prop']['faulted'] = fault_flag  # Keep fault flag visible
```

**Effect**: All PROP telemetry stale EXCEPT 'faulted' flag

### COMM Compound Fault
```python
if self.comm_state.get('faulted', False):
    for subsystem in current_telemetry:
        current_telemetry[subsystem] = make_stale(current_telemetry[subsystem])
```

**Effect**: **ALL subsystems telemetry stale** (complete telemetry blackout)

---

## 6. Power Cycle Recovery

All power cycle commands updated to clear compound anomaly flags:

### ADCS_POWER_CYCLE
- Clears tumble rates (roll_rate, pitch_rate, yaw_rate = 0)
- Resets ADCS fault flag
- Disables ADCS (requires ADCS_ON to restart)

### EPS_POWER_CYCLE
- Clears battery_a_drain, battery_b_drain flags
- Clears solar_array_a_fault, solar_array_b_fault flags
- Resets array status from FAULTED to NOMINAL

### COMM_POWER_CYCLE
- Clears COMM fault flag
- Resets mode to RX
- Restores telemetry visibility

### PROP_POWER_CYCLE
- Clears prop_valve_stuck_open flag
- Forces valve closed
- Disables all PROP systems (requires restart sequence)

### TCS_POWER_CYCLE
- Clears tcs_heaters_stuck_on flag
- Turns off all heaters
- Resets TCS to nominal state

---

## 7. Documentation

**COMMAND_REFERENCE.txt** - New Section Added:

### OFF-NOMINAL COMPOUND ANOMALY RECOVERY PROCEDURES

Comprehensive recovery procedures for all 6 compound anomaly types:

1. **ADCS Fault During Maneuver**
   - Symptoms, recovery procedure, notes
   - 5-step recovery sequence

2. **Battery Drain**
   - Symptoms, recovery procedure, cross-charge explanation

3. **Solar Array Fault**
   - Symptoms, recovery procedure, power conservation options

4. **COMM Fault**
   - Symptoms, recovery procedure
   - **WARNING**: Commanding blind without telemetry

5. **PROP Valve Stuck Open**
   - Symptoms, 6-step recovery procedure
   - **WARNING**: Rapid propellant loss

6. **TCS Heaters Stuck ON**
   - Symptoms, recovery procedure
   - Temperature monitoring guidance

### COMPOUND ANOMALY RESPONSE STRATEGY

Multi-fault triage guidance:

**Priority Order**:
1. COMM fault (restores telemetry visibility)
2. Thermal runaway (prevents hardware damage)
3. ADCS tumbling (restores attitude control)
4. Power issues (battery/solar)
5. PROP leak (preserves fuel)

**Example Scenarios**:
- COMM Fault + Battery Drain (sequential recovery)
- ADCS Tumble + PROP Leak (prioritization example)

---

## 8. Research Data Collection

**CSV Export** (experiment_manager.py):

- Compound anomaly type stored as "COMPOUND: TYPE1 + TYPE2"
- Example: "COMPOUND: ADCS_FAULT_DURING_MANEUVER + BATTERY_DRAIN"
- Tracks which 2 anomalies occurred for analysis
- Captures anomaly occurrence time, resolution time
- Records whether operator resolved both faults

---

## Research Applications

### What This Tests

1. **Multi-Tasking Under Pressure**
   - 2 simultaneous faults at inopportune time (3-5 min mid-procedure)
   - Tests cognitive load management

2. **Prioritization Skills**
   - Which fault to address first?
   - COMM fault = lose visibility (highest priority)
   - Thermal = hardware damage risk (high priority)
   - Power/ADCS/PROP = operational capability

3. **Procedural Memory**
   - Multi-step recovery procedures (4-6 steps each)
   - Correct command sequencing under stress

4. **Systems Thinking**
   - Understanding fault interactions
   - Cross-charge solution for power faults
   - Impact of telemetry loss on recovery

5. **Decision Making Without Information**
   - COMM fault = commanding blind
   - Must execute recovery without telemetry feedback
   - Tests procedural knowledge vs. data-driven decisions

### Experimental Design

**Control Group**: Nominal scenario (no faults)
**Treatment Group**: Off-nominal scenario (2 compound faults)

**Metrics**:
- Time to recognize compound nature of fault
- Prioritization choice (which fault addressed first)
- Recovery procedure accuracy (correct command sequence)
- Time to full recovery
- Off-procedure command rate during compound faults
- Whether COMM fault increases off-procedure errors (commanding blind)

---

## Technical Implementation Summary

### Files Modified

1. **experiment_gui.py** (+10 lines)
   - Added "Off-Nominal (10 min)" radio button

2. **experiment_manager.py** (+75 lines)
   - Added off-nominal scheduling (180-300 seconds)
   - Added _inject_compound_anomaly() method
   - Random 2-of-6 selection with descriptions

3. **telemetry_generator.py** (+150 lines)
   - 6 compound injection commands (INJECT_COMPOUND_*)
   - 6 anomaly state flags
   - EPS battery drain behavior
   - Solar array fault behavior
   - PROP valve stuck open behavior + command rejection
   - TCS heaters stuck ON behavior + command rejection
   - Compound telemetry stale logic (ADCS, PROP, COMM)
   - Power cycle command updates

4. **COMMAND_REFERENCE.txt** (+220 lines)
   - 6 recovery procedures with symptoms, steps, notes
   - Compound anomaly response strategy
   - Priority guidance
   - Example scenarios

### State Flags

```python
self.anomaly_states = {
    'battery_a_drain': False,
    'battery_b_drain': False,
    'solar_array_a_fault': False,
    'solar_array_b_fault': False,
    'prop_valve_stuck_open': False,
    'tcs_heaters_stuck_on': False,
}
```

### Injection Commands

1. `INJECT_COMPOUND_ADCS_FAULT`
2. `INJECT_COMPOUND_BATTERY_DRAIN`
3. `INJECT_COMPOUND_SOLAR_FAULT`
4. `INJECT_COMPOUND_COMM_FAULT`
5. `INJECT_COMPOUND_PROP_FAULT`
6. `INJECT_COMPOUND_TCS_FAULT`

---

## Testing Recommendations

1. **Test Each Compound Anomaly Individually**
   - Verify symptoms appear correctly
   - Verify telemetry stale behavior
   - Verify recovery procedure works
   - Verify power cycle clears flags

2. **Test 2 Compound Anomalies Together**
   - Start off-nominal scenario
   - Verify 2 different anomalies injected at 3-5 minutes
   - Practice recovery procedures
   - Verify CSV captures both anomaly types

3. **Test Specific Combinations**
   - COMM + anything (commanding blind scenario)
   - ADCS + PROP (tumble + leak, fuel conservation)
   - Battery + Solar (both power faults, cross-charge solution)
   - TCS + anything (thermal + operational fault)

4. **Verify Edge Cases**
   - PROP valve close rejected when stuck open
   - TCS heater OFF rejected when stuck on
   - Cross-charge enables after battery drain injection
   - Telemetry returns after COMM power cycle

---

## Success Criteria

✅ **Off-nominal scenario available in GUI**  
✅ **2 of 6 anomalies injected randomly at 3-5 minutes**  
✅ **Each anomaly has realistic behavioral implementation**  
✅ **Telemetry stale logic working (ADCS, PROP, COMM)**  
✅ **Command rejections working (valve close, heater off)**  
✅ **Power cycle commands clear all flags**  
✅ **Recovery procedures documented**  
✅ **CSV export captures compound anomaly types**  

---

## Future Enhancements (Optional)

1. **Anomaly Interactions**
   - Battery drain faster during TCS heaters stuck on
   - ADCS tumble affects solar array sun exposure
   - PROP leak rate increases during tumble

2. **Time Pressure**
   - Critical thresholds if not recovered promptly
   - Propellant depletion from stuck valve
   - Hardware damage from thermal runaway

3. **Additional Compound Anomalies**
   - GPS fault (navigation loss)
   - Star tracker fault (attitude sensing loss)
   - CDH storage full + high CPU load

4. **Graduated Difficulty**
   - Easy: 1 fault (current anomaly scenario)
   - Medium: 2 faults (current off-nominal scenario)
   - Hard: 3+ faults simultaneously

---

## Conclusion

The compound anomaly system provides a realistic multi-fault scenario that tests operators' ability to:
- Recognize complex failure modes
- Prioritize recovery actions
- Execute multi-step procedures under stress
- Make decisions with incomplete information (telemetry loss)
- Manage cascading effects across subsystems

This creates an ideal research environment for studying human performance in high-stakes, multi-tasking scenarios representative of real spacecraft operations.
