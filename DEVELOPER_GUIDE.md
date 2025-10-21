# Developer Guide
# Spacecraft Ground Command Display System

## System Architecture

### Core Components

1. **telemetry_generator.py** - Spacecraft Simulator
   - Class: `SpacecraftSimulator`
   - Thread-safe: Uses `threading.Lock()`
   - Update rate: 10 Hz (100ms)
   - Global instance: `spacecraft`

2. **control_gui.py** - Control Display
   - Class: `ControlGUI`
   - Framework: tkinter
   - Updates: Calls `spacecraft` methods directly

3. **eid_gui.py** - EID Display  
   - Class: `EIDGUI`
   - Framework: tkinter
   - Traffic Light System: Built-in color coding

4. **launcher.py** - System Launcher
   - Class: `LauncherGUI`
   - Manages subprocess launches

## Adding New Telemetry Channels

### Step 1: Add to Simulator (telemetry_generator.py)

In `__init__` method, add to `self.telemetry`:
```python
self.telemetry = {
    # ... existing channels ...
    'new_parameter': 0.0,  # Add your new parameter
}
```

In `_update_subsystems` method, add update logic:
```python
# Update new parameter
self.telemetry['new_parameter'] = calculate_value()
```

### Step 2: Add to Control GUI (control_gui.py)

No changes needed! The Control GUI automatically displays all telemetry channels.

### Step 3: Add to EID GUI (eid_gui.py)

Add to `self.selected_tlm_keys` in `__init__`:
```python
self.selected_tlm_keys = [
    # ... existing keys ...
    'new_parameter',  # Add your new parameter
]
```

Add color rules in `get_tlm_color` method:
```python
elif key == 'new_parameter':
    if value > threshold_warning:
        return 'red'
    elif value > threshold_caution:
        return 'yellow'
    else:
        return 'lightgreen'
```

## Adding New Caution/Warning Conditions

In `telemetry_generator.py`, edit `_check_cautions_warnings` method:

```python
def _check_cautions_warnings(self):
    with self.lock:
        self.cautions.clear()
        self.warnings.clear()
        
        # Add your new check
        if self.telemetry['new_parameter'] > critical_value:
            self.warnings.append("NEW PARAMETER CRITICAL")
        elif self.telemetry['new_parameter'] > caution_value:
            self.cautions.append("New parameter elevated")
```

## Implementing Command Responses

In `telemetry_generator.py`, edit `_process_commands` method:

```python
def _process_commands(self):
    with self.lock:
        while self.command_queue:
            cmd = self.command_queue.pop(0)
            self.add_evr(f"Command received: {cmd}")
            
            # Add command handling
            if cmd == "SAFE_MODE":
                self.enter_safe_mode()
            elif cmd == "POWER_RESET":
                self.reset_power_system()
            # ... more commands ...

def enter_safe_mode(self):
    """Enter safe mode configuration"""
    self.telemetry['attitude_roll'] = 0.0
    self.telemetry['attitude_pitch'] = 0.0
    self.telemetry['attitude_yaw'] = 0.0
    self.add_evr("Entered SAFE MODE")
```

## Adding More Realistic Orbital Mechanics

Replace simple circular orbit with SGP4:

```python
# Install: pip install sgp4
from sgp4.api import Satrec, jday
from datetime import datetime

# In __init__:
self.satrec = Satrec.twoline2rv(
    tle_line1,  # TLE line 1
    tle_line2   # TLE line 2
)

# In _update_orbital_position:
def _update_orbital_position(self, elapsed_time):
    jd, fr = jday(year, month, day, hour, minute, second)
    e, r, v = self.satrec.sgp4(jd, fr)
    
    # Convert r, v to lat/lon/alt
    # ... implementation ...
```

## Adding Data Logging

Create a logger class:

```python
import json
from datetime import datetime

class TelemetryLogger:
    def __init__(self, filename):
        self.filename = filename
        self.log_file = open(filename, 'w')
        
    def log(self, telemetry):
        entry = {
            'timestamp': datetime.now().isoformat(),
            'data': telemetry
        }
        self.log_file.write(json.dumps(entry) + '\n')
        self.log_file.flush()
        
    def close(self):
        self.log_file.close()

# In SpacecraftSimulator:
self.logger = TelemetryLogger('telemetry.log')

# In _simulate loop:
self.logger.log(self.telemetry)
```

## Adding Telemetry Plots

Use matplotlib for plots:

```python
# Create new file: telemetry_plotter.py
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from telemetry_generator import spacecraft

class TelemetryPlotter:
    def __init__(self):
        self.fig, self.axes = plt.subplots(2, 2)
        self.data_history = {
            'time': [],
            'altitude': [],
            'battery_voltage': [],
            'temperature_internal': [],
            'attitude_roll': []
        }
        
    def update(self, frame):
        telemetry = spacecraft.get_telemetry()
        self.data_history['time'].append(time.time())
        self.data_history['altitude'].append(telemetry['altitude'])
        # ... add more ...
        
        # Plot on axes
        self.axes[0, 0].clear()
        self.axes[0, 0].plot(self.data_history['time'], 
                            self.data_history['altitude'])
        self.axes[0, 0].set_title('Altitude')
        # ... plot more ...
        
    def start(self):
        ani = FuncAnimation(self.fig, self.update, interval=1000)
        plt.show()
```

## Adding 3D Visualization

Use matplotlib 3D or VPython:

```python
# Install: pip install vpython
from vpython import sphere, vector, rate

class SpacecraftVisualizer:
    def __init__(self):
        self.earth = sphere(pos=vector(0,0,0), radius=6371, 
                          texture='earth.jpg')
        self.spacecraft = sphere(pos=vector(6771,0,0), radius=50,
                               color=color.cyan)
        
    def update(self):
        while True:
            rate(10)  # 10 Hz
            telemetry = spacecraft.get_telemetry()
            
            # Convert lat/lon/alt to 3D position
            r = 6371 + telemetry['altitude']
            lat = math.radians(telemetry['latitude'])
            lon = math.radians(telemetry['longitude'])
            
            x = r * math.cos(lat) * math.cos(lon)
            y = r * math.cos(lat) * math.sin(lon)
            z = r * math.sin(lat)
            
            self.spacecraft.pos = vector(x, y, z)
```

## Adding Ground Station Communications

```python
class GroundStation:
    def __init__(self, name, lat, lon):
        self.name = name
        self.latitude = lat
        self.longitude = lon
        self.min_elevation = 10.0  # degrees
        
    def is_visible(self, spacecraft_lat, spacecraft_lon, spacecraft_alt):
        # Calculate elevation angle
        # ... implementation using spherical trigonometry ...
        return elevation > self.min_elevation
        
# In SpacecraftSimulator:
self.ground_stations = [
    GroundStation("Atlanta", 33.7490, -84.3880),
    GroundStation("Hawaii", 19.8968, -155.5828),
    # ... more stations ...
]

def check_ground_station_visibility(self):
    for station in self.ground_stations:
        if station.is_visible(self.telemetry['latitude'],
                             self.telemetry['longitude'],
                             self.telemetry['altitude']):
            self.add_evr(f"In view of {station.name}")
```

## Customizing GUI Layout

### Changing Colors

In control_gui.py or eid_gui.py:
```python
# Change background colors
main_frame.configure(bg='darkblue')

# Change text colors  
label.configure(fg='white', bg='darkblue')

# Change fonts
label.configure(font=('Arial', 14, 'bold'))
```

### Adding New Buttons

In eid_gui.py:
```python
new_btn = tk.Button(cmd_grid, text="New Command",
                   command=lambda: self.execute_command("NEW_CMD"),
                   bg='lightblue', font=('Arial', 8))
new_btn.grid(row=2, column=0, padx=2, pady=2)
```

### Adding Status Indicators

```python
status_frame = tk.Frame(inner_frame, bg='white')
status_frame.pack()

self.status_label = tk.Label(status_frame, text="STATUS: NOMINAL",
                            font=('Arial', 12, 'bold'),
                            bg='green', fg='white')
self.status_label.pack()

# Update in update_data():
if warnings:
    self.status_label.configure(text="STATUS: WARNING", bg='red')
elif cautions:
    self.status_label.configure(text="STATUS: CAUTION", bg='yellow')
else:
    self.status_label.configure(text="STATUS: NOMINAL", bg='green')
```

## Testing

### Unit Tests

Create test_simulator.py:
```python
import unittest
from telemetry_generator import SpacecraftSimulator

class TestSpacecraftSimulator(unittest.TestCase):
    def setUp(self):
        self.sim = SpacecraftSimulator()
        
    def test_initialization(self):
        self.assertEqual(self.sim.telemetry['altitude'], 400.0)
        
    def test_battery_in_range(self):
        self.sim.start()
        time.sleep(1)
        telemetry = self.sim.get_telemetry()
        self.assertGreaterEqual(telemetry['battery_voltage'], 24.0)
        self.assertLessEqual(telemetry['battery_voltage'], 28.5)
        self.sim.stop()
        
if __name__ == '__main__':
    unittest.main()
```

### Integration Tests

Test GUI interactions:
```python
def test_command_sending():
    # Start simulator
    spacecraft.start()
    
    # Send command
    spacecraft.send_command("TEST_COMMAND")
    time.sleep(0.5)
    
    # Check EVR log
    evr_log = spacecraft.get_evr_log()
    assert "TEST_COMMAND" in str(evr_log)
    
    spacecraft.stop()
```

## Performance Optimization

### Reduce Update Rate
```python
# In GUIs, change:
self.root.after(100, self.update_data)  # 10 Hz
# To:
self.root.after(500, self.update_data)  # 2 Hz (less CPU)
```

### Optimize Telemetry Storage
```python
# Use numpy for efficient arrays
import numpy as np

self.telemetry_history = {
    'time': np.array([]),
    'altitude': np.array([]),
    # ... more ...
}
```

## Debugging Tips

### Enable Debug Logging
```python
import logging
logging.basicConfig(level=logging.DEBUG)

# In critical sections:
logging.debug(f"Telemetry: {self.telemetry}")
logging.debug(f"Command queue: {self.command_queue}")
```

### Print Thread State
```python
import threading
print(f"Active threads: {threading.active_count()}")
print(f"Thread list: {threading.enumerate()}")
```

### Monitor Update Rate
```python
import time
last_time = time.time()

def update_data(self):
    global last_time
    now = time.time()
    delta = now - last_time
    print(f"Update delta: {delta:.3f}s ({1/delta:.1f} Hz)")
    last_time = now
    # ... rest of update ...
```

## Common Issues and Solutions

### Issue: GUIs freeze
**Solution**: Ensure long operations are in threads, not in GUI update loop

### Issue: Data not updating
**Solution**: Check spacecraft.running is True, verify locks aren't deadlocked

### Issue: High CPU usage
**Solution**: Reduce update rate, optimize calculations in _simulate loop

### Issue: Memory leak
**Solution**: Limit EVR log size (currently limited to 100 entries)

## Best Practices

1. **Always use locks** when accessing shared data
2. **Keep GUI updates fast** - no heavy computation in update_data()
3. **Log important events** - use add_evr() liberally
4. **Test with edge cases** - battery depleted, attitude extreme
5. **Document new features** - update README.md
6. **Use descriptive names** - channel names should be clear
7. **Validate inputs** - check command formats
8. **Handle errors gracefully** - try/except in critical sections

## Version Control

Recommended .gitignore:
```
__pycache__/
*.pyc
*.log
*.txt (except documentation)
venv/
.vscode/
```

## Packaging for Distribution

### Using PyInstaller

```bash
pip install pyinstaller

# Create executable
pyinstaller --onefile --windowed launcher.py

# Include all files
pyinstaller --onefile --windowed \
  --add-data "telemetry_generator.py;." \
  --add-data "control_gui.py;." \
  --add-data "eid_gui.py;." \
  launcher.py
```

### Creating installer

Use Inno Setup (Windows) or create .app bundle (macOS)

## Contributing

When adding features:
1. Test thoroughly
2. Update documentation
3. Add comments to complex sections
4. Follow existing code style
5. Update PROJECT_SUMMARY.txt

---
Happy coding! 🚀
