# Procedure Verification System

## Overview

The Experiment Control Panel now supports **procedure verification** mode, where procedures are loaded in the background for validation rather than being automatically executed. This allows you to conduct human-in-the-loop experiments where test subjects execute procedures manually while the system verifies their actions.

## How It Works

### 1. **Start Scenario**
- Select "Nominal" or "Anomaly" scenario
- The scenario starts but **does NOT** automatically execute commands
- This begins the experimental session

### 2. **Start Procedure** 
When you click "Start Procedure":
- A predefined procedure is loaded into memory
- The procedure timer starts
- The system begins monitoring commands for verification
- **No commands are executed automatically**

The test subject should now execute the procedure they have in hand (paper procedure, digital checklist, etc.)

### 3. **Command Verification**
As the test subject sends commands through the Control GUI:
- Each command is logged with timestamp
- The system checks if the command matches the expected next step
- **Correct steps** are logged as ✓ Step N correct
- **Incorrect steps** are logged as ✗ Step N ERROR with details

### 4. **End Procedure**
When you click "End Procedure":
- Procedure timing stops
- A summary shows:
  - Steps completed vs. expected
  - Number of procedure errors
  - Total time taken
- All details are saved to the experiment log file

## Default Procedure

The system currently has a **3-step default procedure**:

1. `ADCS_ON` - Turn on the Attitude Determination and Control System
2. `AVI_POWER_CYCLE` - Power cycle the Avionics system
3. `PROP_ON` - Turn on the Propulsion system

## Loading Custom Procedures

You can load a custom procedure by calling the `load_procedure()` method:

```python
# In experiment_manager
experiment_manager.load_procedure([
    "ADCS_ON",
    "AVI_POWER_CYCLE",
    "PROP_ON",
    "THERMAL_NOMINAL",
    "COMM_HIGH_GAIN"
])
```

This will be integrated into the GUI in a future update.

## Metrics Tracked

The system tracks:

### Command Metrics
- Total commands sent
- Command execution success/failure
- Command error rate

### Procedure Metrics
- Procedure duration (time to complete)
- Steps completed vs. expected
- Procedure step errors (wrong command at wrong step)
- Detailed error log with:
  - Expected command
  - Actual command received
  - Step number
  - Timestamp

### Anomaly Metrics (Anomaly Scenarios Only)
- Time when anomaly was injected
- Time when operator detected anomaly (manual marking)
- Time when anomaly was resolved (manual marking)
- Detection delay
- Response time
- Total time to resolution

## Experiment Log Files

All data is saved to JSON files with format:
```
experiment_log_{scenario_type}_{timestamp}.json
```

Example: `experiment_log_nominal_20251117_143025.json`

The log includes:
- Scenario configuration
- All events with timestamps
- All commands with timestamps
- Complete metrics
- Procedure error details

## Example Workflow

1. **Experimenter**: Click "Start Scenario" (Nominal, 5 min)
2. **Experimenter**: Hand paper procedure to test subject
3. **Experimenter**: Click "Start Procedure" 
4. **Test Subject**: Executes commands from paper procedure via Control GUI
5. **System**: Silently verifies each command, logs errors
6. **Test Subject**: Completes procedure
7. **Experimenter**: Click "End Procedure"
8. **System**: Displays summary with errors detected
9. **Experimenter**: Reviews experiment log file for detailed analysis

## Future Enhancements

Planned features:
- GUI for loading custom procedures from file
- Real-time procedure step display for experimenter
- Visual indicator when test subject makes an error
- Procedure library with multiple predefined procedures
- Export procedure error report to CSV
- Graphical timeline of procedure execution

## Notes

- The test subject should NOT be able to see the procedure verification status
- Only the experimenter sees the verification results
- This maintains experimental validity by preventing feedback during execution
- Procedure errors do NOT prevent command execution - they are only logged for analysis
