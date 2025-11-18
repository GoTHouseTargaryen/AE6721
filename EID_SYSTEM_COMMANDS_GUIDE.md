# EID System-Specific Commands Guide

## Overview

The EID (Electronic Interface Display) now features **system-specific command menus** that appear when you click on any telemetry display. Instead of showing generic commands, the system intelligently displays only the commands relevant to the subsystem you're interacting with.

## How It Works

### 1. Click on Any Telemetry Display
Each of the 15 telemetry displays in the EID shows a specific spacecraft parameter (battery voltage, attitude roll, temperature, etc.)

### 2. System-Specific Commands Appear
When you click a display, a popup window shows:
- **System Name** (e.g., "Attitude Control (Roll)", "Power System (Battery)")
- **Current Value** of that telemetry parameter
- **Relevant Commands** for that specific subsystem
- **Emergency Safe Mode** button (always available)

### 3. Execute Commands
Click any command button to immediately execute it. The command is sent to the spacecraft and logged (if experiment is active).

## Subsystem Command Mappings

### Attitude Control System
**Telemetry**: Roll, Pitch, Yaw

**Available Commands**:
- 🎯 Reset Attitude (`ATT_RESET`)
- 🌍 Nadir Point (`ATT_NADIR`)
- ☀️ Sun Point (`ATT_SUN_POINT`)
- 📍 Inertial Hold (`ATT_INERTIAL`)
- ✓ Clear Cautions (`CLEAR_CAUTIONS`)

**When to Use**:
- Attitude deviation detected
- Spacecraft pointing is off-target
- Need to change attitude mode

---

### Power System - Battery
**Telemetry**: Battery Voltage

**Available Commands**:
- ⚡ Power Reset (`PWR_RESET`)
- 🔋 Battery Charge (`BATT_CHARGE`)
- ☀️ Sun Point (Charge) (`ATT_SUN_POINT`)
- 🔌 Low Power Mode (`MODE_LOW_POWER`)
- ⚠️ Emergency Shutdown (`EMERG_SHUTDOWN`)
- ✓ Clear Cautions (`CLEAR_CAUTIONS`)

**When to Use**:
- Battery voltage low or dropping
- Need to conserve power
- Battery anomaly detected

---

### Power System - Current
**Telemetry**: Battery Current

**Available Commands**:
- ⚡ Power Reset (`PWR_RESET`)
- 🔌 Low Power Mode (`MODE_LOW_POWER`)
- ⚠️ Emergency Shutdown (`EMERG_SHUTDOWN`)
- 🔬 Science Mode (`MODE_SCIENCE`)
- 📊 Nominal Mode (`MODE_NOMINAL`)

**When to Use**:
- Current draw is too high or too low
- Need to change power consumption mode
- Switching between operational modes

---

### Power System - Solar
**Telemetry**: Solar Array Current

**Available Commands**:
- ☀️ Sun Point (`ATT_SUN_POINT`)
- ⚡ Power Reset (`PWR_RESET`)
- 🔋 Battery Charge (`BATT_CHARGE`)
- 📊 Nominal Mode (`MODE_NOMINAL`)

**When to Use**:
- Solar array not generating enough power
- In eclipse and need to prepare for sunlight
- Maximizing power generation

---

### Thermal Control - Internal
**Telemetry**: Internal Temperature

**Available Commands**:
- 🌡️ Thermal Reset (`THERM_RESET`)
- 🔌 Low Power Mode (`MODE_LOW_POWER`)
- 📊 Nominal Mode (`MODE_NOMINAL`)
- ✓ Clear Cautions (`CLEAR_CAUTIONS`)

**When to Use**:
- Internal temperature out of range
- Overheating or too cold inside spacecraft
- Thermal anomaly detected

---

### Thermal Control - External
**Telemetry**: External Temperature

**Available Commands**:
- 🌡️ Thermal Reset (`THERM_RESET`)
- 🎯 Attitude Adjust (`ATT_RESET`)
- ☀️ Sun Point (`ATT_SUN_POINT`)
- ✓ Clear Cautions (`CLEAR_CAUTIONS`)

**When to Use**:
- External skin temperature abnormal
- Need to change thermal orientation
- Adjusting spacecraft thermal state

---

### Communication System
**Telemetry**: Communication Signal

**Available Commands**:
- 📡 Comm Reset (`COMM_RESET`)
- 🎯 Nadir Point (`ATT_NADIR`)
- 🔧 Diagnostic Mode (`DIAG_MODE`)
- ✓ Clear Cautions (`CLEAR_CAUTIONS`)

**When to Use**:
- Communication signal degraded
- Signal strength below threshold
- Antenna pointing issue

---

### Environmental Control
**Telemetry**: Pressure

**Available Commands**:
- 🛡️ Safe Mode (`SAFE_MODE`)
- 🔧 Diagnostic Mode (`DIAG_MODE`)
- ✓ Clear Cautions (`CLEAR_CAUTIONS`)

**When to Use**:
- Cabin pressure anomaly
- Environmental control system fault
- Critical life support issue

---

### Orbital Position Parameters
**Telemetry**: Altitude, Velocity, Latitude, Longitude

**Available Commands**:
- 🔧 Diagnostic Mode (`DIAG_MODE`)
- 💊 Health Check (`CMD6`)

**Note**: These are informational telemetry values. Orbital parameters cannot be directly commanded (they result from orbital mechanics), so only diagnostic commands are available.

---

## Emergency Safe Mode

**Always Available**: Every system-specific command window includes an emergency SAFE MODE button at the bottom.

**Safe Mode** (`SAFE_MODE`):
- Sets operational mode to SAFE
- Points spacecraft toward sun (power generation)
- Resets all attitudes to 0°
- Minimizes power consumption
- Clears all anomaly flags

**Use When**:
- Multiple system failures
- Critical battery situation
- Unknown anomaly state
- Need to stabilize spacecraft immediately

## Command Logging

All commands executed through the system-specific menus are:
1. Sent to the spacecraft immediately
2. Logged in the EVR (Event Record) log
3. Tracked by the Experiment Manager (if active)
4. Verified against loaded procedures (if procedure is active)

## Integration with Experiment Control

When running experiments:
- Commands executed through EID are logged with timestamps
- If a procedure is loaded, commands are verified against expected steps
- Errors are tracked if wrong commands are executed
- Anomaly resolution is automatically marked when response commands are used

## Benefits

### For Operators:
- ✅ Only see relevant commands for each system
- ✅ Faster response to anomalies
- ✅ Reduced cognitive load
- ✅ Clear system context with current values
- ✅ Less chance of executing wrong command

### For Experimenters:
- ✅ More realistic operational environment
- ✅ Better data on command selection patterns
- ✅ Track which systems operators interact with
- ✅ Measure response time per subsystem
- ✅ Analyze command appropriateness

## Example Scenarios

### Scenario 1: Battery Voltage Low
1. Battery voltage display turns **YELLOW** (caution)
2. Operator clicks on **Battery Voltage** display
3. System shows: "Power System (Battery)" with current voltage
4. Operator sees battery-specific commands
5. Operator selects "☀️ Sun Point (Charge)" to maximize solar input
6. Command executed and logged

### Scenario 2: Attitude Deviation
1. Attitude roll display turns **RED** (warning) 
2. Operator clicks on **Attitude Roll** display
3. System shows: "Attitude Control (Roll)" with current deviation
4. Operator sees attitude control commands
5. Operator selects "🎯 Reset Attitude"
6. Then selects "🌍 Nadir Point" to resume normal pointing
7. Both commands executed and logged

### Scenario 3: Communication Signal Degraded
1. Comm signal display turns **YELLOW**
2. Operator clicks on **Communication Signal** display
3. System shows: "Communication System" with signal strength
4. Operator sees comm-specific commands
5. Operator tries "🎯 Nadir Point" to improve ground station link
6. If that doesn't work, operator tries "📡 Comm Reset"
7. All attempts logged for post-experiment analysis

## Tips

- **Check Current Value**: The popup shows the current telemetry value - verify the issue before commanding
- **Emergency Button**: Safe Mode is always available at the bottom of every menu
- **Traffic Lights**: Use the color coding (Green/Yellow/Red) to prioritize which systems need attention
- **Multiple Steps**: You can execute multiple commands - the window stays open until you close it or execute a command
- **Experiment Mode**: Your command choices are tracked - choose wisely!

## Future Enhancements

Planned improvements:
- Command confirmation for critical commands
- Command history per subsystem
- Recommended command suggestions based on telemetry state
- Command effectiveness tracking
- Undo last command functionality
- Multi-step command sequences
