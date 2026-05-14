# Security Analysis of LiDAR-based UAV Navigation

BSc Computer Science Dissertation  
University of Bristol, 2026  
Author: Borys Babushkin Fynkelstheyn

---

## Abstract

Uncrewed Aerial Vehicles (UAVs) are increasingly deployed in GPS-denied
environments — indoor facilities, urban canyons, and contested electromagnetic
spaces — where Light Detection and Ranging (LiDAR) sensors serve as the primary
means of obstacle avoidance and navigation. While considerable research has
examined the security of GPS-based UAV systems and of LiDAR in the autonomous
vehicle domain, the intersection of LiDAR vulnerabilities and UAV navigation
remains critically under-explored. This dissertation presents a security
analysis of a LiDAR-based UAV navigation system, built around a Holybro X500 v2
airframe with a Pixhawk 6C flight controller running PX4 firmware and a Garmin
LiDAR-Lite v3 (1D) sensor.

The empirical contribution is a four-set surface-manipulation study spanning
reflective aluminium-foil, transparent, planar-mirror, and magnified-mirror
conditions, sampled across incidence angles of 0°, 15°, 30°, 45°, and 60° with
30 to 31 trials per cell, yielding 581 measurements. The results expose three
qualitatively distinct failure classes: stable biased returns, where the
transparent surface produces a +6.53 cm bias at 0° and the reflective surface
progresses from +1.4 cm at 0° to +44.4 cm at 60°; catastrophic false ranging,
where the planar mirror at 15° returns a mean of 346.3 cm against a 42.5 cm
ground truth; and complete signal loss, where the magnified-mirror condition
exhibits a 100% failure rate at 15° to 45°.

These failure modes threaten PX4's altitude-hold, terrain-relative navigation,
and landing logic in distinct ways. Stable bias evades variance-based sanity
checks and is silently absorbed into the EKF2 altitude state. Catastrophic
ranging produces innovations large enough to corrupt the estimator. Signal loss
forces the rangefinder timeout path with potentially unsafe fallback behaviour.
A layered countermeasure framework is proposed and validated through a
100-experiment SITL parameter sweep, combining signal-quality gating, temporal
rate-of-change validation, and cross-modal sensor fusion, with each layer
traced to the specific failure it mitigates.

The project was completed as a 40-credit unit over approximately 400 hours.

**Key achievements:**

- Comprehensive literature review covering LiDAR principles, UAV navigation
  architectures, the PX4/MAVLink stack, and the AV LiDAR attack taxonomy, plus
  extensive study of the full PX4 Autopilot documentation, EKF2 estimator
  source code, and MAVLink 2.0 protocol specification.
- Assembly of the Holybro X500 v2 drone platform from kit form with a
  downward-facing Garmin LiDAR-Lite v3 sensor. Physical bench testing was
  conducted over approximately four days at 4 hours each, producing 581
  measurements across four surface conditions and five incidence angles.
- Roughly 6,000 lines of original Python and shell code, comprising a LiDAR
  injection toolkit with eight attack modes and two software countermeasures,
  a telemetry monitor, automated sweep and experiment management scripts, and
  post-experiment analysis and plotting tools.
- A 100-experiment SITL parameter sweep validating the layered countermeasure
  framework against drift, oscillation, spike, and constant attacks, with full
  CSV logging of all EKF altitude traces and countermeasure actions.
- Identification of three qualitatively distinct failure classes and their
  corresponding EKF2 propagation pathways, plus the discovery that clamping
  preserves sensor fusion continuity while sample rejection causes secondary
  failure.

---

## Repository Structure
├── dissertation/ # LaTeX source and compiled PDF
├── lidar_scripts/ # Injection toolkit, monitor, sweep, and analysis
├── results/ # SITL experiment logs (CSV)
├── results_sweep_v2/ # Countermeasure parameter sweep results
├── figures/ # Plots and diagrams
├── logs/ # Monitor and injector CSV logs
├── bench_experiment_data/ # Physical sensor test measurements
└── PX4-Autopilot/ # PX4 firmware (built for SITL)

---

## Prerequisites

- **Python 3.8+**
- **PX4 Autopilot** — built for SITL
- **QGroundControl** — for vehicle configuration and telemetry
- **jMAVSim** — for visual observation during attacks
- **Python packages:** `pymavlink`, `matplotlib`, `csv`, `argparse`, `statistics`

### Install Python dependencies

```bash
pip install -r scripts/requirements.txt 
```

## SITL Testing
### Build PX4
```bash
cd PX4-Autopilot
make px4_sitl_default
```
### Start PX4 SITL
```bash
cd PX4-Autopilot
PX4_SIM_MODEL=none_iris ./build/px4_sitl_default/bin/px4 \
  -s "$PWD/build/px4_sitl_default/etc/init.d-posix/rcS" \
  -w "$PWD/build/px4_sitl_default/rootfs" \
  "$PWD/build/px4_sitl_default/etc"
```
### Start jMAVSim for visual observation (in a separate terminal)
```bash
cd PX4-Autopilot
./Tools/simulation/jmavsim/jmavsim_run.sh -l
```
### Configure PX4 for rangefinder injection (in PX4 console (pxh>))
```bash
param set EKF2_RNG_CTRL 1
param set EKF2_RNG_A_HMAX 50
param set EKF2_RNG_QLTY_T 0.1
param set EKF2_BARO_CTRL 0
param set EKF2_GPS_CTRL 0
param set COM_ARM_WO_GPS 1
commander arm -f
commander takeoff
```
### Run the LiDAR injection attack
Terminal A — Monitor:
```bash
cd lidar_scripts
python3 lidar_monitor.py --endpoint udpin:127.0.0.1:14550
```

Terminal B — Injector:
```bash
cd lidar_scripts
python3 lidar_injector.py --mode drift --duration 30 --drift-rate 0.5 --countermeasure none
```

## License
This project is academic research software produced for a BSc dissertation at
the University of Bristol. All code, data, and written content are the author's
own work except where third-party resources are explicitly credited in the
Supporting Technologies section of the dissertation.

## Citation
Babushkin Fynkelstheyn, B. (2026).
Hacking Non-GPS/GPS-Denied Drones:
Security Analysis of LiDAR-based UAV Navigation.
BSc Dissertation, Department of Computer Science, University of Bristol.
