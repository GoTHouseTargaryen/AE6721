# Experiment System Guide

## Overview
The experiment system allows you to run controlled scenarios for research data collection. It logs operator performance metrics including response times, command errors, and procedure completion times.

## Starting an Experiment

1. **Launch the Experiment Control Panel**:
   - Run `launcher.py` and click "Launch Experiment Control"
   - Or run `python experiment_gui.py` directly

2. **Configure Scenario**:
   - **Nominal Scenario**: 5-minute duration with no anomalies
   - **Anomaly Scenario**: 10-minute duration with a guaranteed anomaly between minutes 2-9

3. **Define Nominal Procedure**:
   - Enter commands in the text box (one per line)
   - Example procedure:
     ```
     ATT_NADIR
     MODE_NOMINAL
     CLEAR_CAUTIONS
     CMD6
     ```
   - Comments start with `#` and are ignored

4. **Click "Start Scenario"** to begin

## During the Experiment

### Operator Actions
The experiment GUI provides buttons to record operator events:

- **✓ Anomaly Detected**: Click when you first notice the anomaly
- **✓ Anomaly Resolved**: Click after successfully resolving the anomaly
- **⏱ Start Procedure**: Click when beginning the nominal procedure
- **⏱ End Procedure**: Click when completing the nominal procedure

### Command Logging
All commands sent through the Control GUI are automatically logged with timestamps.

### EVR Log Indicators
When a command is sent, you will see three EVR entries:
1. `CMD RECV [count]: <command>` - Command received
2. `CMD EXEC [count]: Executed successfully` - Command executed
3. `CMD RADIATE [count]: Command radiated to spacecraft` - Command transmitted

The command count increments with each command.

## Anomaly Injection

The system will automatically inject one of these anomalies (random):
- **Battery Low**: Voltage drops below minimum threshold
- **Attitude Error**: Spacecraft pointing error exceeds limits
- **Thermal Warning**: Internal temperature out of range
- **Comm Degraded**: Communication signal drops
- **Power Fault**: High battery discharge rate

## Data Collection

All experiment data is logged to JSON files in the project directory:
- `experiment_log_nominal_YYYYMMDD_HHMMSS.json`
- `experiment_log_anomaly_YYYYMMDD_HHMMSS.json`

### Log File Contents:
- **scenario_type**: "nominal" or "anomaly"
- **start_time**: Scenario start timestamp
- **duration_minutes**: Planned duration
- **nominal_procedure**: List of procedure commands
- **events[]**: All logged events with timestamps
  - `timestamp`: ISO format timestamp
  - `elapsed_seconds`: Time since scenario start
  - `event_type`: Event category
  - `description`: Event details
- **commands[]**: All commands sent
  - `timestamp`: When command was sent
  - `elapsed_seconds`: Time since scenario start
  - `command_number`: Sequential command count
  - `command`: Command text
  - `success`: True/False
  - `error`: Error message if failed
- **metrics**: Performance measurements
  - `total_duration_seconds`: Actual scenario duration
  - `total_commands`: Number of commands sent
  - `command_errors`: Number of failed commands
  - `command_error_rate`: Percentage of failed commands
  - `procedures_completed`: Number of complete procedure runs
  - `average_procedure_time`: Average time to complete procedure
  - **For anomaly scenarios**:
    - `anomaly_injection_time`: When anomaly occurred (seconds)
    - `anomaly_detection_delay`: Time to detect (seconds)
    - `anomaly_response_time`: Time to resolve after detection (seconds)
    - `anomaly_total_time`: Total time from injection to resolution (seconds)

## Analysis

Use the JSON log files to analyze:
1. **Reaction Time**: How quickly operators detect anomalies
2. **Response Time**: How quickly operators resolve anomalies
3. **Command Accuracy**: Error rate and types of errors
4. **Procedure Efficiency**: Time to complete nominal procedures
5. **Workload**: Commands per minute, procedure frequency

## Example Nominal Procedure

A typical spacecraft health check procedure:
```
# Verify attitude
ATT_NADIR
CMD6

# Check power system
MODE_NOMINAL
CLEAR_CAUTIONS

# Verify communications
CMD7

# Final health check
CMD6
```

## Tips for Experiments

1. **Practice First**: Run a few scenarios without logging to familiarize operators
2. **Consistent Procedures**: Use the same nominal procedure for all test subjects
3. **Record Manually**: Note any unusual behavior or operator comments separately
4. **Randomize**: Alternate between nominal and anomaly scenarios
5. **Debrief**: Ask operators about their decision-making process after each scenario

## Stopping a Scenario

- Click "⏹ Stop Scenario" to end early
- Scenarios automatically stop after their duration
- Data is saved immediately when stopped

## Troubleshooting

**Commands not logging**: Ensure you started a scenario in the Experiment Control GUI first.

**Can't start scenario**: Check that a previous scenario finished. Stop it if needed.

**Missing data**: Check the project directory for JSON files with recent timestamps.

**EVR log full**: EVR keeps last 100 entries. Older entries are automatically removed.
