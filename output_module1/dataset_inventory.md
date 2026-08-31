# IO-VNBD Dataset Inventory Report
**Dataset Name**: IO-VNBD
**Root Directory**: `d:/prototype/IO-VNBD-master`
**Inspection Status**: COMPLETED

## Sequences Discovered
### Sequence: S1
- **Driver / Category**: S (Driver A)
- **Path**: `d:/prototype/IO-VNBD-master\Synchronised V abd S datasets\Categorised IOVNB Dataset\S (Driver A)\S1`
  - **Stream**: `S-S1.csv`
    - Status: `LOADED_OK`
    - Records: 51746
    - Effective Freq: 10.0 Hz (Documented: 10.0 Hz)
    - Schema Match: True
  - **Stream**: `V-S1.csv`
    - Status: `LOADED_OK`
    - Records: 51746
    - Effective Freq: 10.0 Hz (Documented: 10.0 Hz)
    - Schema Match: True

## Data Stream Schemas & Fields
### Smartphone Stream S-S1
Total Fields: 24
```
- GPS LATITUDE (degrees)
- GPS LONGITUDE (degrees)
- GPS ALTITUDE (m)
- GPS SPEED (Kmh)
- GPS ACCURACY (m)
- GPS ORIENTATION (Â°)
- GPS SATELLITES IN RANGE
- TIME SINCE START (ms)
- DATE (YYYY-MO-DD HH-MI-SS_SSS)
- ACCELEROMETER X (m/s²)
- ACCELEROMETER Y (m/s²)
- ACCELEROMETER Z (m/s²)
- GRAVITY X (m/s²)
- GRAVITY Y (m/s²)
- GRAVITY Z (m/s²)
- GYROSCOPE Yaw (rad/s)
- GYROSCOPE Pitch (rad/s)
- GYROSCOPE Roll (rad/s)
- MAGNETIC FIELD X (Î¼T)
- MAGNETIC FIELD Y (Î¼T)
- MAGNETIC FIELD Z (Î¼T)
- ORIENTATION (Yaw) (Â°)
- ORIENTATION (Pitch) (Â°)
- ORIENTATION (Roll ) (Â°)
```

### Vehicle Stream V-S1
Total Fields: 29
```
- No of GPS Satellites Available
- Time Since Start of Day (seconds)
- Latitude (degrees)
- Longitude (degrees)
- Velocity (km/hr)
- Heading (degrees)
- Height (km)
- Vertical velocity (km/hr)
- Sample period (seconds)
- Steering Angle (degrees)
- Wheel Speed Front Left (rad/sec)
- Wheel Speed Front Right (rad/sec)
- Wheel Speed Rear Left (rad/sec)
- Wheel Speed Rear Right (rad/sec)
- Yaw Rate (deg/sec)
- Indicated Vehicle Speed (km/hr)
- Indicated Longitudinal Acceleration (g)
- Indicated Lateral Acceleration (g)
- Handbrake (0 or 1)
- Gear Requested (Number fof gear employed 1-5)
- Gear (Number fof gear employed 1-5)
- Engine Speed (rev/min)
- Coolant Temperature (degrees)
- Clutch Position (0 or 1)
- Brake Pressure (psi)
- Brake Position (0 or 1)
- Battery Voltage (volts)
- Air Temperature (degrees)
- Accelerator Pedal Position (0 or 1)
```
