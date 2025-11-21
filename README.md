# Human-in-the-Loop Spacecraft Ground Command Display System

A comprehensive Human-in-the-Loop (HiTL) research platform for studying operator performance with different ground command display interfaces in spacecraft operations.

## Overview

This system provides a complete experimental environment for human factors research in spacecraft ground control operations. It includes:

1. **Telemetry Generator** (`telemetry_generator.py`) - High-fidelity spacecraft simulator with multiple subsystems
2. **Control GUI** (`control_gui.py`) - Traditional text-based command interface with telemetry channels
3. **EID GUI** (`eid_gui.py`) - Modern graphical interface with traffic light displays and command buttons
4. **Experiment Manager** (`experiment_manager.py`) - Automated experiment control and data logging
5. **Experiment Control** (`experiment_gui.py`) - Researcher interface for managing experimental sessions

## Key Features

### Experimental Research Platform
- **Automated Experiment Control**: Configurable nominal and off-nominal scenarios
- **Performance Metrics**: Automatic tracking of:
  - Command count and error rates
  - Procedure adherence and timing
  - Anomaly detection and resolution times
  - Off-procedure command validation
- **Data Logging**: Comprehensive JSON logs of all experimental sessions
- **Researcher Interface**: Clean control panel for managing test subjects

### Spacecraft Simulator
- **Realistic LEO Orbital Mechanics**: 400km circular orbit, 51.6° inclination
- **Multiple Subsystems**:
  - ADCS (Attitude Determination and Control System) with reaction wheels
  - EPS (Electrical Power System) with dual batteries and solar arrays
  - COMM (Communications) with TX/RX/TX_RX modes and timeout safety
  - PROP (Propulsion) with heated catalyst monopropellant system
  - TCS (Thermal Control System) with deployable radiator and heaters
  - AVI (Avionics), CDH (Command & Data Handling), TTC (Telemetry, Tracking & Command)
  - Star Tracker and GPS navigation
- **Compound Anomaly Scenarios**: Realistic multi-fault scenarios for off-nominal operations
- **Safe Mode Logic**: Comprehensive 7-step safe mode sequence with COMM cycling
- **Event Recording**: Complete EVR log of all spacecraft events

### Control GUI (Traditional Interface)
- Real-time telemetry channel display (14+ channels)
- Free-form text command entry
- EVR log viewer with auto-scroll
- Caution & Warning display with color coding
- Command history tracking

### EID GUI (Enhanced Interface)
- Subsystem-organized telemetry displays with traffic light system
- Clickable displays for detailed subsystem commands
- Button-based command interface (no typing required)
- Safe mode and emergency response buttons
- Visual feedback for all operations

## Research Scenarios

### Nominal Scenarios (5 minutes)
- Baseline performance measurement
- No faults injected
- Standard procedure execution
- Available for both Control and EID interfaces

### Off-Nominal Scenarios (10 minutes)
- **Compound Anomaly Injection**: 2 simultaneous faults at 3-5 minutes
- **Six Fault Types**:
  1. **ADCS Tumble**: Spacecraft tumbling at 5-10°/s, requires power cycle and detumble
  2. **Battery Drain**: One battery draining despite working solar arrays
  3. **Solar Array Fault**: One array fails, requires cross-charge enable
  4. **COMM Fault**: Complete telemetry loss, blind commanding required
  5. **PROP Valve Stuck**: Propellant leak, valve stuck open
  6. **TCS Heaters Stuck**: All heaters stuck on, thermal runaway risk
- **Recovery Procedures**: Each fault requires multi-step recovery
- **Performance Metrics**: Measures operator response to complex failures

## Requirements

- Python 3.6 or higher
- tkinter (included with Python standard installation)
- No additional packages required

## Installation

1. Ensure Python is installed:
   ```bash
   python --version
   ```

2. Clone or download all files to the same directory

3. Run the launcher:
   ```bash
   python launcher.py
   ```
   Or use the provided batch file (Windows):
   ```bash
   run.bat
   ```

## Usage for Researchers

### Setting Up an Experiment

1. **Launch the Experiment Control Panel**:
   ```bash
   python experiment_gui.py
   ```

2. **Configure the Experiment**:
   - Enter Subject Number (e.g., "S001")
   - Select Run Type:
     - **Control Nominal**: Traditional interface, no faults
     - **EID Nominal**: Enhanced interface, no faults
     - **Control Off-Nominal**: Traditional interface, 2 compound faults
     - **EID Off-Nominal**: Enhanced interface, 2 compound faults
   - Review/Edit Nominal Procedure (default provided)

3. **Start Experiment**:
   - Click "Start Experiment"
   - Appropriate GUI launches automatically
   - Procedure is auto-loaded for verification
   - For off-nominal: 2 random faults inject at 3-5 minutes

4. **During Experiment**:
   - Monitor status in Experiment Control Panel
   - All metrics are automatically tracked
   - Anomalies are auto-detected
   - Optional: Use procedure timing buttons

5. **Complete Experiment**:
   - Click "Stop Experiment" when finished
   - Data is automatically saved to JSON file
   - Review metrics in log file

### Data Collection

All experimental data is saved to JSON files with naming format:
```
experiment_log_<scenario>_<timestamp>.json
```

Includes:
- Subject demographics and run type
- Complete command history with timestamps
- Procedure errors and off-procedure commands
- Anomaly timing (injection, detection, resolution)
- Performance metrics and event log

## Command Reference

### Critical Safety Commands
- `SAFE_MODE` - Enter safe mode (7-step sequence)
- `EXIT_SAFE_MODE` - Return to nominal operations
- `SYS_RESET` - Full system reset
- `COMM_TX <seconds>` - TX mode with required timeout (safety feature)

### ADCS Commands
- `ADCS_NADIR` - Nadir pointing mode
- `ADCS_SUN_POINT` - Sun pointing for power generation
- `ADCS_DETUMBLE` - Detumble mode (high rotation rates)
- `ADCS_POWER_CYCLE` - Power cycle ADCS (clears faults)

### EPS Commands
- `EPS_MODE_NOMINAL`, `EPS_MODE_LOW_POWER`, `EPS_MODE_SCIENCE`
- `EPS_CROSS_CHARGE_ENABLE` - Enable cross-charging (critical for battery/solar faults)
- `EPS_DEPLOY_ARRAY_A`, `EPS_DEPLOY_ARRAY_B` - Deploy solar arrays
- `EPS_POWER_CYCLE` - Power cycle EPS

### COMM Commands
- `COMM_TX <seconds>` - Transmit mode (requires timeout)
- `COMM_RX` - Receive mode (commandable)
- `COMM_TX_RX` - Full duplex mode
- `COMM_POWER_CYCLE` - Power cycle COMM (only recovery for COMM fault)

### PROP Commands
- `PROP_STANDBY`, `PROP_ON`, `PROP_OFF` - Power modes
- `PROP_VALVE_OPEN`, `PROP_VALVE_CLOSE` - Valve control
- `PROP_HEATER_ON`, `PROP_HEATER_OFF` - Catalyst heater control
- `PROP_POWER_CYCLE` - Power cycle PROP (clears valve stuck fault)

### TCS Commands
- `TCS_HEATER_AVI_ON/OFF`, `TCS_HEATER_EPS_ON/OFF`, `TCS_HEATER_PROP_ON/OFF`
- `TCS_DEPLOY_RADIATOR`, `TCS_UNDEPLOY_RADIATOR`
- `TCS_POWER_CYCLE` - Power cycle TCS (clears heaters stuck fault)

See `COMMAND_REFERENCE.txt` for complete command documentation including compound anomaly recovery procedures.

## Safe Mode Sequence

When `SAFE_MODE` is executed:
1. Shutdown all PROP systems (standby, valve, heaters)
2. Turn off heaters conditionally (AVI >5°C, EPS >0°C, PROP always)
3. Deploy Solar Array A (if not deployed)
4. Deploy Solar Array B (if not deployed)
5. Deploy Radiator (if not deployed)
6. ADCS to SUN_POINT mode (maximize power)
7. Enable cross-charging (battery redundancy)
8. COMM cycling: RX 60s → TX 60s → TX_RX 30s (repeating)

## Caution & Warning System

### Warnings (Red) - Immediate Attention Required
- Battery voltage < 24.5V or critical SOC
- Attitude rates > critical thresholds
- Temperature outside safe limits
- Subsystem faulted

### Cautions (Yellow) - Monitor Closely  
- Battery voltage < 26V
- Attitude pointing errors > 2°
- Moderate temperature elevation
- Wheel saturation approaching

## Architecture

```
┌──────────────────┐
│ Experiment GUI   │  ← Researcher control panel
└────────┬─────────┘
         │
         ├─ Launches appropriate GUI
         ├─ Auto-loads procedure
         └─ Tracks all metrics
         
┌──────────────────┐
│ Experiment Mgr   │  ← Automated experiment control
└────────┬─────────┘
         │
         ├─ Scenario timing
         ├─ Fault injection
         ├─ Procedure verification
         └─ Data logging
         
┌──────────────────┐
│ Telemetry Gen    │  ← Spacecraft simulator
└────────┬─────────┘
         │
    ┌────┴────┐
    │         │
┌───▼───┐ ┌──▼────┐
│Control│ │  EID  │  ← Operator interfaces
│  GUI  │ │  GUI  │
└───────┘ └───────┘
```

## Key Research Features

### Automated Experiment Control
- No manual fault injection needed
- Procedure automatically loaded and verified
- Anomaly detection/resolution automatically tracked
- Complete performance metrics logged

### Off-Procedure Command Validation
- Commands checked against loaded procedure
- Anomaly response commands validated (not counted as errors)
- SAFE_MODE validated (not error if 2+ cautions/warnings)
- Accurate error classification for research analysis

### Compound Anomaly System
- Realistic multi-fault scenarios
- Requires multi-step recovery procedures
- Tests operator problem-solving under pressure
- Includes telemetry stale conditions for realism

## Documentation

- `README.md` - This file
- `QUICKSTART.txt` - Quick reference guide
- `COMMAND_REFERENCE.txt` - Complete command documentation with recovery procedures
- `EXPERIMENT_GUIDE.md` - Detailed experimental procedures
- `DEVELOPER_GUIDE.md` - Technical implementation details
- `ANOMALY_RESPONSE_GUIDE.txt` - Anomaly recovery procedures

## Troubleshooting

**Problem**: Syntax errors on launch
- **Solution**: Ensure all Python files are using correct quote marks (not escaped quotes)

**Problem**: GUI doesn't launch automatically
- **Solution**: Ensure `control_gui.py` and `eid_gui.py` are in the same directory. Check Experiment Control Panel status display for errors.

**Problem**: Commands not validated correctly
- **Solution**: Ensure nominal procedure is loaded. Check that procedure commands match actual command names in `COMMAND_REFERENCE.txt`.

**Problem**: Procedure errors showing incorrectly
- **Solution**: Anomaly response commands should not be counted as errors. Verify `caution_response_map` in `experiment_manager.py` includes all valid responses.

**Problem**: COMM_TX command rejected
- **Solution**: COMM_TX now requires a timeout parameter for safety. Use `COMM_TX 60` instead of just `COMM_TX`.

**Problem**: Safe mode not working as expected
- **Solution**: Check that safe mode sequence executes all 7 steps. Verify COMM cycling (RX 60s → TX 60s → TX_RX 30s).

## Research Applications

This platform is designed for studying:
- Interface design impact on operator performance
- Command error patterns in nominal vs. off-nominal conditions
- Anomaly detection and response times
- Procedure adherence under stress
- Human factors in spacecraft operations
- Training effectiveness assessment

## Publications & Citations

If you use this system in research, please cite:
```
[Citation information to be added]
```

## Version History

- **v2.0** (2025): Complete research platform with automated experiments, compound anomalies, procedure verification
- **v1.0** (2024): Initial dual-GUI system with basic telemetry

## Contributing

This is an educational/research project. For improvements or bug reports, please contact the development team.

## License

Educational/Research use - Georgia Tech Aerospace Engineering

## Author

Created for AE6721: Human-in-the-Loop Systems
Georgia Tech Aerospace Engineering

## Acknowledgments

- Course instructors and TAs for guidance and requirements
- Test subjects for experimental participation
- Research team for testing and feedback

## Contact

For questions about experimental procedures or system usage:
- Course website: [To be added]
- Technical support: [To be added]

---

**Note**: This system is designed for research and educational purposes. All experimental data should be handled according to IRB protocols and data management guidelines.
