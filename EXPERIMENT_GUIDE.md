# Experiment System Guide

## Overview
The experiment system allows you to run controlled scenarios for research data collection. It logs operator performance metrics including response times, command errors, and procedure completion times.

## Starting an Experiment

1. **Launch the Experiment Control Panel**:
   - Run `launcher.py` and click "Launch Experiment Control"
   - Or run `python experiment_gui.py` directly

2. **Enter Researcher Information** (Required):
   - **Subject Number**: Enter a unique identifier for the research subject (e.g., "001", "Subject_A")
   - **Run Type**: Select from dropdown:
     * Nominal Control
     * Nominal EID
     * Off-Nominal Control
     * Off-Nominal EID

3. **Configure Scenario**:
   - **Nominal Scenario**: 5-minute duration with no anomalies
   - **Anomaly Scenario**: 10-minute duration with a guaranteed anomaly between minutes 2-9
   - **Experimental Scenario**: 10-minute duration with a guaranteed experimental fault between minutes 1-5

4. **Define Nominal Procedure**:
   - Enter commands in the text box (one per line)
   - Example procedure:
     ```
     ATT_NADIR
     MODE_NOMINAL
     CLEAR_CAUTIONS
     CMD6
     ```
   - Comments start with `#` and are ignored

5. **Click "Start Scenario"** to begin

**Note**: Subject Number is required. The system will not start without it.

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

### Standard Anomaly Scenarios
The system will automatically inject one of these anomalies (random):
- **Battery Low**: Voltage drops below minimum threshold
- **Attitude Error**: Spacecraft pointing error exceeds limits
- **Thermal Warning**: Internal temperature out of range
- **Comm Degraded**: Communication signal drops
- **Power Fault**: High battery discharge rate

### Experimental Fault Scenarios
For controlled research, the **Experimental Scenario** injects one of three guaranteed fault types:

#### CDH (Command & Data Handling) Fault
- **Effect**: ALL commanding is blocked except `CDH_POWER_CYCLE`
- **Telemetry**: All telemetry remains visible
- **Recovery**: Send `CDH_POWER_CYCLE` to clear fault
- **Research Focus**: Tests operator ability to diagnose and recover with limited control

#### TTC (Telemetry Tracking & Command) Fault
- **Effect**: 75-100% of telemetry channels go stale (marked with " S")
- **Telemetry**: TTC fault flag remains visible, other subsystems mostly stale
- **Commanding**: All commands still work
- **Metrics**: Tracks "commanding without telemetry" count when sending commands to stale subsystems
- **Recovery**: Send `TTC_POWER_CYCLE` to clear fault
- **Research Focus**: Tests operator ability to command effectively with limited visibility

#### AVI (Avionics) Fault
- **Effect**: ALL commanding is blocked except `AVI_POWER_CYCLE`
- **Telemetry**: ALL telemetry goes stale EXCEPT the AVI subsystem (including fault flag)
- **Recovery**: Send `AVI_POWER_CYCLE` to clear fault
- **Research Focus**: Tests operator ability to diagnose with minimal information

**Timing**: Experimental faults occur randomly between 1-5 minutes (earlier than standard anomalies)

## Data Collection

All experiment data is logged to both JSON and CSV files in the project directory:

### JSON Files (Detailed Logs):
- `experiment_log_nominal_YYYYMMDD_HHMMSS.json`
- `experiment_log_anomaly_YYYYMMDD_HHMMSS.json`
- `experiment_log_experimental_YYYYMMDD_HHMMSS.json`

### CSV Files (Research Summary):
- `experiment_log_nominal_YYYYMMDD_HHMMSS.csv`
- `experiment_log_anomaly_YYYYMMDD_HHMMSS.csv`
- `experiment_log_experimental_YYYYMMDD_HHMMSS.csv`

**CSV files are appended to**, allowing multiple runs to be collected in one spreadsheet for easy analysis.

### CSV Output Columns:
1. **Subject Number** - Researcher-assigned identifier
2. **Run Type** - Nominal Control, Nominal EID, Off-Nominal Control, or Off-Nominal EID
3. **Scenario Type** - nominal, anomaly, or experimental
4. **Start Time** - When experiment began
5. **End Time** - When experiment ended
6. **Total Duration (s)** - Total experiment duration in seconds
7. **Procedure Completion Time (s)** - Time to complete the nominal procedure
8. **Total Commands** - Number of commands sent
9. **Command Errors** - Number of failed commands
10. **Off-Procedure Commands** - Commands that deviated from expected procedure
11. **Total Cautions/Warnings** - Count of all cautions/warnings that occurred
12. **Cautions/Warnings List** - Detailed list of all issues
13. **Avg Caution/Warning Response Time (s)** - Average time to respond to issues
14. **Anomaly Injected** - Type of anomaly (if applicable)
15. **Anomaly Injection Time (s)** - When anomaly occurred
16. **Anomaly Resolution Time (s)** - Time to resolve anomaly
17. **Procedure Step Errors** - Number of out-of-sequence procedure steps

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
  - **For experimental scenarios**:
    - `experimental_fault_injection_time`: When fault occurred (seconds)
    - `experimental_fault_type`: Which fault was injected (CDH, TTC, or AVI)
    - `fault_detection_delay`: Time to detect (seconds)
    - `fault_response_time`: Time to resolve after detection (seconds)
    - `fault_total_time`: Total time from injection to resolution (seconds)
    - `commanding_without_telemetry_count`: Number of commands sent to stale subsystems (TTC fault only)

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
