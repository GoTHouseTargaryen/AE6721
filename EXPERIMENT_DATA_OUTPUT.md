# Experiment Data Output Guide

## Overview
The experiment system automatically outputs comprehensive research data in both JSON and CSV formats. CSV files are designed for easy analysis in spreadsheet applications and statistical software.

## Required Researcher Inputs

Before starting any experiment, researchers must provide:

### 1. Subject Number
- Unique identifier for the research subject
- Examples: "001", "S01", "Subject_Alpha"
- Used to track individual performance across multiple runs

### 2. Run Type
Select from dropdown menu:
- **Nominal Control**: Baseline performance with standard GUI
- **Nominal EID**: Performance with Enhanced Information Display
- **Off-Nominal Control**: Anomaly response with standard GUI
- **Off-Nominal EID**: Anomaly response with Enhanced Information Display

## CSV Output Format

### Automatic Export
- CSV file created automatically when experiment ends
- Same base filename as JSON log, with `.csv` extension
- **Appended to existing file** - multiple runs accumulate in one CSV
- Ready for import into Excel, SPSS, R, Python pandas, etc.

### CSV Columns

| Column Name | Description | Example Values |
|------------|-------------|----------------|
| Subject Number | Researcher-assigned ID | "001", "Subject_A" |
| Run Type | Experimental condition | "Nominal EID", "Off-Nominal Control" |
| Scenario Type | Test scenario | "nominal", "anomaly", "experimental" |
| Start Time | ISO timestamp of start | "2025-11-21T14:30:00.123456" |
| End Time | ISO timestamp of end | "2025-11-21T14:35:15.789012" |
| Total Duration (s) | Experiment duration | "315.67" |
| Procedure Completion Time (s) | Time to complete procedure | "245.12" |
| Total Commands | Number of commands sent | "23" |
| Command Errors | Failed commands | "2" |
| Off-Procedure Commands | Commands not in expected sequence | "3" |
| Total Cautions/Warnings | Count of all issues | "5" |
| Cautions/Warnings List | Detailed issue list | "CAUTION: Battery A voltage low; WARNING: Temperature high" |
| Avg Caution/Warning Response Time (s) | Mean response time | "12.45" |
| Anomaly Injected | Type of anomaly | "battery_low", "EXPERIMENTAL_TTC", "None" |
| Anomaly Injection Time (s) | When anomaly occurred | "145.23" |
| Anomaly Resolution Time (s) | Time to resolve | "67.89" |
| Procedure Step Errors | Out-of-sequence steps | "1" |

## Tracked Metrics Explained

### 1. Procedure Completion Time
- Starts when researcher clicks "⏱ Start Procedure"
- Ends when researcher clicks "⏱ End Procedure"
- Measures how long subject takes to complete nominal operations
- **Use Case**: Compare nominal procedure efficiency across subjects

### 2. Cautions and Warnings
- **Automatically tracked** throughout experiment
- Records when each caution/warning appears
- Records when each is resolved (disappears)
- Lists all issues that occurred with full details
- **Use Case**: Identify which issues occur most frequently

### 3. Caution/Warning Response Time
- Time from when issue appears to when ANY command is sent
- Averaged across all cautions/warnings in the run
- Measures operator reaction and decision-making speed
- **Use Case**: Compare response speed between control and EID conditions

### 4. Anomaly Information
For anomaly and experimental scenarios:
- **Anomaly Type**: Specific fault injected (e.g., "battery_low", "EXPERIMENTAL_CDH")
- **Injection Time**: Seconds into experiment when anomaly occurred
- **Resolution Time**: Total seconds from injection to resolution
- Automatically determined by system monitoring
- **Use Case**: Analyze recovery performance for different fault types

### 5. Command Errors
- Total commands that failed to execute
- Includes commands rejected due to:
  * Subsystem faulted
  * Invalid state transitions
  * Experimental fault conditions (CDH/AVI lockouts)
- **Use Case**: Measure operator accuracy and decision quality

### 6. Off-Procedure Commands
- Commands that don't match expected procedure sequence
- **EXCLUDES** commands addressing active cautions/warnings
- **EXCLUDES** power cycle commands for faulted subsystems
- **EXCLUDES** safe mode activations
- Only counts unnecessary deviations from procedure
- **Use Case**: Identify training needs and procedure adherence

## Example Research Analysis

### Comparing Control vs EID Performance

```
Subject | Run Type          | Completion Time | Caution Response | Command Errors
--------|-------------------|-----------------|------------------|---------------
001     | Nominal Control   | 245.12 s        | 15.23 s         | 3
001     | Nominal EID       | 198.45 s        | 8.67 s          | 1
002     | Nominal Control   | 267.89 s        | 18.91 s         | 4
002     | Nominal EID       | 201.23 s        | 9.12 s          | 1
```

**Analysis**: EID condition shows ~20% faster completion, 50% faster caution response, and fewer errors.

### Anomaly Recovery Performance

```
Subject | Anomaly Type     | Injection Time | Resolution Time | Off-Procedure
--------|------------------|----------------|-----------------|---------------
003     | battery_low      | 145.23 s       | 67.89 s        | 2
003     | thermal_warning  | 178.45 s       | 45.12 s        | 1
004     | attitude_error   | 132.67 s       | 89.34 s        | 4
```

**Analysis**: Thermal warnings resolved faster than battery issues; attitude errors correlate with more off-procedure commands.

## Data Collection Workflow

1. **Setup**:
   - Enter Subject Number
   - Select Run Type
   - Configure scenario and procedure

2. **During Experiment**:
   - System automatically tracks all metrics
   - Caution/warning monitoring runs in background
   - All commands logged with timestamps
   - Anomaly injection and resolution tracked

3. **After Experiment**:
   - Click "⏹ Stop Scenario" or wait for automatic end
   - JSON file saved with complete event log
   - CSV file automatically exported/appended
   - Files appear in project directory

4. **Analysis**:
   - Open CSV in Excel, R, Python, SPSS, etc.
   - Each row = one experimental run
   - Multiple subjects and conditions in one file
   - Ready for statistical analysis

## Best Practices

### Subject Numbering
- Use consistent format (e.g., "S01", "S02", "S03")
- Pad with zeros for proper sorting ("001" not "1")
- Can include conditions in ID if needed ("S01_Trained")

### Run Types
- **Nominal Control**: Baseline without EID, no anomalies expected
- **Nominal EID**: Baseline with EID, no anomalies expected
- **Off-Nominal Control**: With anomalies, standard GUI
- **Off-Nominal EID**: With anomalies, Enhanced Information Display

### Data Management
- CSV files accumulate all runs automatically
- Rename CSV file periodically to organize by study phase
- Keep JSON files for detailed event reconstruction
- Back up data files regularly

### Statistical Analysis
The CSV format is ready for:
- **Excel**: PivotTables, charts, descriptive statistics
- **SPSS**: ANOVA, t-tests, regression
- **R**: tidyverse, ggplot2, statistical modeling
- **Python**: pandas, scipy.stats, seaborn
- **JASP**: Free alternative to SPSS with GUI

## Troubleshooting

**Missing CSV file**: Check that experiment completed (wasn't stopped prematurely)

**CSV not appending**: File may be open in Excel - close it first

**"N/A" values**: Metric not applicable (e.g., no anomaly in nominal scenario)

**Wrong Run Type**: Edit manually in CSV or note in separate documentation

**Subject forgot to mark procedure end**: Use Total Duration as proxy for completion time

## Summary

The automated CSV output provides:
✅ All required research metrics
✅ Automatically calculated response times
✅ Comprehensive caution/warning tracking
✅ Anomaly injection and resolution data
✅ Command error analysis
✅ Ready for statistical analysis
✅ Multiple runs in one file
✅ No manual data entry required

This streamlines the research workflow from experiment execution to statistical analysis, reducing errors and saving significant time in data processing.
