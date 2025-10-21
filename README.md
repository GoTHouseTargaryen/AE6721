# Spacecraft Ground Command Display System

A comprehensive Human-in-the-Loop (HiTL) ground command display system for monitoring and controlling a spacecraft in Low Earth Orbit (LEO).

## Overview

This system consists of three main components:

1. **Telemetry Generator** (`telemetry_generator.py`) - Simulates a spacecraft in LEO at 400km altitude
2. **Control GUI** (`control_gui.py`) - Left display panel showing telemetry channels, EVR log, and caution & warning
3. **EID GUI** (`eid_gui.py`) - Right display panel showing TLM value displays with traffic light system, EVR log, and command buttons

## Features

### Spacecraft Simulator
- Realistic LEO orbital mechanics (400km altitude, ~92 minute orbit)
- Multiple subsystems:
  - Orbital position (latitude, longitude, altitude, velocity)
  - Attitude control (roll, pitch, yaw)
  - Power system (battery, solar arrays)
  - Thermal management
  - Communications
  - Environmental control (pressure)
- Automated caution and warning generation
- Command queue processing
- Event recording (EVR)

### Control GUI
- Real-time telemetry channel display (14 channels)
- EVR (Event Record) log viewer
- Caution & Warning display with color coding
- Command window for sending commands to spacecraft
- Clean table format for telemetry data

### EID GUI
- 15 TLM value displays in 3x5 grid
- Traffic light system (green/yellow/red) for quick status assessment
- EVR log viewer
- 8 command buttons for quick access
- Clickable TLM displays to open Anomaly Response Command Window
- Emergency response commands (Safe Mode, Power Reset, etc.)

## Requirements

- Python 3.6 or higher (Python comes pre-installed on most systems)
- tkinter (included with Python standard installation)

No additional packages required! This system uses only Python standard library components.

## Installation

1. Ensure Python is installed:
   ```
   python --version
   ```

2. All files should be in the same directory:
   - `telemetry_generator.py`
   - `control_gui.py`
   - `eid_gui.py`
   - `launcher.py`

## Usage

### Method 1: Using the Launcher (Recommended)

Simply run the launcher:
```
python launcher.py
```

The launcher provides options to:
- Launch both GUIs simultaneously
- Launch Control GUI only
- Launch EID GUI only
- Test the simulator in terminal mode

### Method 2: Manual Launch

Launch Control GUI:
```
python control_gui.py
```

Launch EID GUI (in a separate terminal):
```
python eid_gui.py
```

### Method 3: Test Simulator Only

To test the spacecraft simulator without GUIs:
```
python telemetry_generator.py
```

## How to Use

1. **Launch the system** using `launcher.py` and click "Launch Both GUIs"

2. **Monitor telemetry** on the Control GUI:
   - View all 14 telemetry channels in real-time
   - Check EVR log for system events
   - Watch for cautions (yellow/orange) and warnings (red)

3. **Monitor status** on the EID GUI:
   - Green displays = nominal operation
   - Yellow displays = caution condition
   - Red displays = warning condition
   - Click any TLM display to open Anomaly Response window

4. **Send commands**:
   - Use the Command Window in Control GUI to type custom commands
   - Use the Cmd buttons in EID GUI for quick commands
   - Use Anomaly Response window for emergency procedures

5. **Observe orbital behavior**:
   - Watch latitude/longitude change as spacecraft orbits
   - Notice day/night cycle affecting solar array current
   - See battery voltage decrease in eclipse, increase in sunlight
   - Monitor attitude control maintaining spacecraft orientation

## Spacecraft Telemetry Parameters

| Parameter | Description | Nominal Range |
|-----------|-------------|---------------|
| Altitude | Orbital altitude | 399.5 - 400.5 km |
| Velocity | Orbital velocity | 7.66 - 7.68 km/s |
| Latitude | Orbital latitude | -51.6° to +51.6° |
| Longitude | Orbital longitude | -180° to +180° |
| Attitude Roll/Pitch/Yaw | Spacecraft orientation | ±2° (caution at ±2°, warning at ±4°) |
| Battery Voltage | Main bus voltage | 26-28.5 V (caution <26V, warning <24.5V) |
| Battery Current | Battery load | 2-3 A |
| Solar Array Current | Solar power generation | 0-3.5 A (depends on sun exposure) |
| Temperature Internal | Cabin temperature | 18-28°C (warning outside 15-30°C) |
| Temperature External | Skin temperature | -150 to -80°C |
| Pressure | Cabin pressure | 14.6-14.8 psi |
| Communication Signal | Signal strength | >80% nominal |

## Caution & Warning System

### Warnings (Red)
- Battery voltage < 24.5V
- Attitude deviation > 4°
- Internal temperature outside 15-30°C range

### Cautions (Yellow/Orange)
- Battery voltage < 26V
- Attitude deviation > 2°
- Communication signal < 80%

## Traffic Light System (EID GUI)

- **Green**: All parameters nominal
- **Yellow**: Caution condition, monitor closely
- **Red**: Warning condition, immediate attention required

## Architecture

```
┌─────────────────────┐
│ telemetry_generator │  ← Core simulator (shared instance)
└─────────┬───────────┘
          │
    ┌─────┴─────┐
    │           │
┌───▼────┐  ┌──▼─────┐
│Control │  │  EID   │  ← GUI displays (separate processes)
│  GUI   │  │  GUI   │
└────────┘  └────────┘
```

Both GUIs connect to the same spacecraft simulator instance, ensuring synchronized data.

## Troubleshooting

**Problem**: Python not found
- **Solution**: Install Python from python.org or use Microsoft Store version

**Problem**: tkinter not available
- **Solution**: On Linux, install python3-tk package: `sudo apt-get install python3-tk`

**Problem**: GUIs not updating
- **Solution**: Close all windows and restart. Ensure only one instance is running.

**Problem**: Commands not working
- **Solution**: Check EVR log to confirm commands are being received. Type command in Command Window and press Enter or click Send.

## Development Notes

- Update rate: 10 Hz (100ms refresh)
- Orbital simulation: Simplified circular orbit model
- Thread-safe: All data access protected by locks
- Extensible: Easy to add new telemetry channels or commands

## Future Enhancements

- Add more realistic orbital propagation (SGP4)
- Implement actual command responses (thruster firing, mode changes)
- Add data logging to files
- Create historical telemetry plots
- Add ground station pass predictions
- Implement communication delays based on spacecraft position

## License

Educational/Research use

## Author

Created for AE6721 course
Georgia Tech Aerospace Engineering

## Contact

For issues or questions, please refer to the course materials or contact the instructor.
