Fly or Die
=========

Fly or Die is a sandbox for experimenting with PX4 flight logs, UAV datasets, and motor models. This repo currently contains:

- `download_uav_dataset.sh` – helper script that fetches a small sample dataset with logs, configs, and models.
- Notes on inferring rotor RPM and angular velocity from PX4 actuator outputs.

This README walks you through downloading the assets, running the helper script, and translating actuator values into physically meaningful quantities.

Dataset Download
----------------

The dataset lives in an S3 bucket and is downloaded by `download_uav_dataset.sh`. The script retrieves two sample logs along with companion configuration and model bundles, unzips them, and removes the original archives.

### Prerequisites

- Bash 4+
- `wget`
- `unzip`
- Sufficient disk space for the extracted logs (a few hundred MB)

### Usage

```bash
chmod +x download_uav_dataset.sh
./download_uav_dataset.sh
```

Assets are extracted to `./data/UAV`. Re-run the script any time you need to refresh the data—the script overwrites existing files but will not delete additional items you add manually.

The weight of the drone when the data is captured is about 1075g.

Drone Multi-IMUs
-------------------------
The drone data is collected using Modal AI M500 drone, which is available at: https://docs.modalai.com/m500.
There are four IMUs are embedded on the drone's flight module called as Flight core. Two ICM20948 IMUs are available on the VOXL (https://docs.modalai.com/voxl-datasheets-functional-description/), and the other two IMUs, ICM42688 and ICM2002, are embedded on the PX4 flight controller known as Flight Core (https://docs.modalai.com/flight-core-datasheets-functional-description/).

The frequency for each IMU is:
 - VOXL:
    - ICM20948: ~ 1000Hz
 - PX4:
    - ICM42688: ~ 100Hz
    - ICM2002: ~ 100Hz

You are free to select one IMU or combine multiple IMUs to estimate drone's position and draw the trajectory.

Rotor Speed From PX4 Logs
-------------------------

PX4 logs expose actuator setpoints in the `actuator_motors` topic. Each element in the `control` array is normalized to the range `[-1, 1]` where `0` is neutral thrust, `1` is the maximum forward thrust setpoint, and `-1` (if supported) is the maximum reverse thrust setpoint.

To map a control value to rotor speed you need the motor’s maximum RPM (`RPM_max`). For VOXL ESCs the nominal maximum is `15 000 RPM`.

1. Convert control values to RPM:

   ```
   RPM = RPM_max × control_value
   ```

2. Convert RPM to angular velocity (rad/s):

   ```
   ω = RPM × (2π / 60)
   ```

Example: With `RPM_max = 10 000` and a control value of `0.5`, the rotor spins at `5 000 RPM`, giving `ω ≈ 523.6 rad/s`.

Notes & Tips
------------

- Always verify the actual `RPM_max` for your motor and ESC combo—manufacturer specs or calibration data may differ from nominal values.
- For reversible motors, negative control values represent reverse rotation using the same relationships above.
- Ensure the actuator scaling in PX4 matches the firmware and ESC configuration; otherwise the mapping from control value to torque/RPM may be non-linear.


Next step
---------
Under the src/dataset folder, dataset reader is given in python script. You can play around the given scripts to familiarize with the drone data.