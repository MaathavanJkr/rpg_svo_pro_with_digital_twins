# GPS Simulator

The GPS Simulator is a ROS 2 tool for simulating GPS data using real-world ephemeris data and Gaussian models. It incorporates the effects of multipath errors and provides trilaterated receiver positions in multiple coordinate frames.

## Overview

The simulator models GPS signals by:
- Using Gaussian Mixture Models  and Gaussian Processes  to simulate multipath errors and the number of satellites in line-of-sight as a function of the receiver's height.
- Taking ground truth poses of the receiver in the ENU (East-North-Up) frame.
- Producing the receiver's trilaterated position in multiple coordinate frames:
  - ENU (East-North-Up)
  - LLA (Latitude-Longitude-Altitude)
  - ECEF (Earth-Centered, Earth-Fixed)

## How It Works

1. **Input Models**:
   - Gaussian Mixture Model : Simulates multipath error.
   - Gaussian Process : Models the number of satellites visible in the line-of-sight as a function of receiver height.

2. **Input Data**:
   - The receiver's ground truth pose in a cartesian frame.

3. **Simulation**:
   - Uses the true GPS oribital data and trilaterates the receiver's position based on their corrupted pseudorange data.

4. **Output**:
   - Trilaterated receiver coordinates in ENU, LLA, or ECEF frames.

## Command

To start the GPS simulator, use the following command:

```bash
ros2 launch satellite_simulator gnss_simulator.launch.py
```
## Example Models

Examples of Gaussian Mixture Models and Gaussian Processes for simulating multipath error can be found here:

[Example Models](https://example.com/gps-models)
