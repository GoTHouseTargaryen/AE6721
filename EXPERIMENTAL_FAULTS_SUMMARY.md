# Experimental Fault System - Implementation Summary

## Overview
Added a new experimental fault system for controlled research scenarios with three distinct fault types that test different operator capabilities under specific constraints.

## Features Implemented

### 1. Three Experimental Fault Types

#### CDH (Command & Data Handling) Fault
- **Blocks ALL commanding** except `CDH_POWER_CYCLE`
- All telemetry remains visible and current
- Tests operator's ability to diagnose issues with full visibility but limited control
- Recovery: Power cycle CDH subsystem

#### TTC (Telemetry Tracking & Command) Fault
- **75-100% of telemetry channels go stale** (randomly selected 8-10 out of 10 subsystems)
- TTC fault flag always remains visible (never goes stale)
- All commanding still works normally
- **Tracks "commanding without telemetry" count** as research metric
- Tests operator's ability to command effectively with limited visibility
- Recovery: Power cycle TTC subsystem

#### AVI (Avionics) Fault
- **Blocks ALL commanding** except `AVI_POWER_CYCLE`
- **ALL telemetry goes stale** except the AVI subsystem (including fault flag)
- Tests operator's ability to diagnose with minimal information
- Recovery: Power cycle AVI subsystem

### 2. Timing and Scheduling
- Experimental faults occur randomly between **1-5 minutes** into the scenario
- Earlier than standard anomalies (2-9 minutes) for controlled testing
- One fault guaranteed per experimental scenario

### 3. Command Execution Logic
**File: `telemetry_generator.py` (lines 2437-2453)**
- Pre-execution checks before processing any command
- CDH/AVI faults: Reject all commands except corresponding power cycle
- TTC fault: Track commands sent to subsystems with stale telemetry
- EVR messages for rejected commands and warnings

### 4. Telemetry Stale Logic
**File: `telemetry_generator.py` (lines 3066-3082)**
- Applied in `get_telemetry()` method
- Makes subsystems stale based on `experimental_fault_stale_subsystems` list
- Special case: TTC fault flag never goes stale during TTC fault
- Uses existing `make_stale()` function (appends " S" to values)

### 5. Power Cycle Integration
**Files: `telemetry_generator.py`**
- `CDH_POWER_CYCLE` (line 2927): Calls `clear_experimental_fault()`
- `AVI_POWER_CYCLE` (line 2914): Calls `clear_experimental_fault()` and resets `avi_state['faulted']`
- Both commands clear experimental fault state immediately

### 6. Experiment Manager Integration
**File: `experiment_manager.py`**
- New scenario type: `'experimental'`
- `_inject_experimental_fault()` method (lines 167-187)
- `_run_scenario()` modified to handle experimental scenarios (lines 107-138)
- Random fault type selection from ['CDH', 'TTC', 'AVI']
- EVR logging for experimental fault injection

### 7. Metrics Collection
**File: `experiment_manager.py` (lines 370-384)**
- `experimental_fault_injection_time`: When fault occurred
- `experimental_fault_type`: Which fault type (CDH/TTC/AVI)
- `fault_detection_delay`: Time to detect
- `fault_response_time`: Time to resolve after detection
- `fault_total_time`: Total time from injection to resolution
- `commanding_without_telemetry_count`: TTC-specific metric

### 8. GUI Updates
**File: `experiment_gui.py`**
- Added third radio button: "Experimental (10 min)"
- Orange color indicator (`selectcolor='#f39c12'`)
- 10-minute duration (same as anomaly scenarios)

### 9. Documentation
**File: `EXPERIMENT_GUIDE.md`**
- Complete documentation of experimental fault system
- Detailed descriptions of each fault type
- Recovery procedures
- Research focus explanation
- Metrics documentation

## Code Changes Summary

### `telemetry_generator.py`
1. **Lines 372-384**: Added experimental fault tracking variables
   - `experimental_fault_active`
   - `experimental_fault_type`
   - `experimental_fault_stale_subsystems`
   - `commanding_without_telemetry_count`

2. **Lines 710-760**: Added fault injection/clearing methods
   - `inject_experimental_fault(fault_type)`
   - `clear_experimental_fault()`

3. **Lines 2437-2453**: Modified `_execute_command()`
   - Pre-execution checks for CDH/AVI faults
   - Commanding without telemetry tracking for TTC

4. **Lines 3066-3082**: Modified `get_telemetry()`
   - Apply stale to subsystems in experimental_fault_stale_subsystems
   - Special handling for TTC fault flag

5. **Line 2914**: Modified `AVI_POWER_CYCLE`
   - Added `clear_experimental_fault()` call
   - Added `avi_state['faulted'] = False`

6. **Line 2927**: Modified `CDH_POWER_CYCLE`
   - Added `clear_experimental_fault()` call

### `experiment_manager.py`
1. **Lines 107-138**: Modified `_run_scenario()`
   - Added experimental scenario handling
   - 1-5 minute fault timing

2. **Lines 167-187**: Added `_inject_experimental_fault()`
   - Random fault type selection
   - Direct spacecraft.inject_experimental_fault() call
   - EVR logging

3. **Lines 370-384**: Modified `_end_scenario()`
   - Added experimental fault metrics
   - commanding_without_telemetry_count
   - experimental_fault_type

### `experiment_gui.py`
1. **Lines 46-61**: Added experimental radio button

### `EXPERIMENT_GUIDE.md`
1. Updated scenario types
2. Added experimental fault section
3. Added metrics documentation

## Testing Checklist

### CDH Fault Testing
- [ ] CDH fault blocks all commands except CDH_POWER_CYCLE
- [ ] All telemetry remains visible during CDH fault
- [ ] CDH_POWER_CYCLE clears the fault
- [ ] EVR shows "COMMAND REJECTED - CDH FAULTED" messages

### TTC Fault Testing
- [ ] TTC fault makes 8-10 subsystems go stale
- [ ] TTC fault flag remains visible (not stale)
- [ ] All commands still work during TTC fault
- [ ] commanding_without_telemetry_count increments correctly
- [ ] EVR shows "WARNING: Commanding X without telemetry" messages
- [ ] TTC_POWER_CYCLE clears the fault

### AVI Fault Testing
- [ ] AVI fault blocks all commands except AVI_POWER_CYCLE
- [ ] All telemetry goes stale except AVI subsystem
- [ ] AVI fault flag remains visible
- [ ] AVI_POWER_CYCLE clears the fault
- [ ] EVR shows "COMMAND REJECTED - AVI FAULTED" messages

### Experiment Manager Testing
- [ ] Experimental scenario starts successfully
- [ ] Fault occurs between 1-5 minutes
- [ ] Random fault type selected each run
- [ ] Metrics logged correctly in JSON file
- [ ] commanding_without_telemetry_count logged for TTC faults

### GUI Testing
- [ ] "Experimental (10 min)" radio button appears
- [ ] Orange color indicator displays
- [ ] Scenario runs for 10 minutes
- [ ] Log file created with "experimental" in filename

## Usage Example

1. Launch experiment GUI
2. Select "Experimental (10 min)" scenario
3. Define nominal procedure
4. Start scenario
5. Wait 1-5 minutes for fault to occur
6. Observe fault symptoms (command rejection, stale telemetry)
7. Diagnose fault type from available information
8. Send appropriate power cycle command
9. Verify fault cleared
10. Check metrics in JSON log file

## Research Applications

### CDH Fault Research Questions
- How quickly can operators diagnose commanding failures?
- What alternative strategies do operators try?
- How effective is telemetry analysis for diagnosis?

### TTC Fault Research Questions
- How do operators adapt to partial telemetry loss?
- Do operators command "blindly" or wait for telemetry?
- What is the commanding_without_telemetry rate?
- How does partial information affect decision-making?

### AVI Fault Research Questions
- Can operators diagnose with minimal information?
- How long does it take to identify AVI as the source?
- What diagnostic strategies emerge?
- How does near-total telemetry loss affect confidence?

## Future Enhancements

Potential additions:
- Adjustable fault timing windows
- Combined fault scenarios (multiple simultaneous faults)
- Partial recovery scenarios
- Additional subsystem-specific faults
- Configurable telemetry stale percentages
- Real-time performance feedback
