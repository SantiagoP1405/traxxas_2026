# parking_system

Control, perception, and decision-making package for the autonomous parking system of the Traxxas vehicle. This package coordinates information from multiple sensors (LiDAR, ultrasonics, ZED stereo camera) and executes a finite state machine to perform the parking maneuver safely.

---

## System Overview

The system is based on an architecture of interconnected ROS2 nodes that divide perception logic, high-level control, and physical actuation:

**1. State Machine (FSM):** The `fsm_parking3` node dictates the current phase of the parking maneuver.
**2. LiDAR Processing:** The `lidar_processor` node converts the scanner readings into logical zones (front, rear, sides) to determine if the environment is clear or occupied.
**3. Stereo Vision (ZED + YOLO):** The `lane_detector_camera` node reads the ZED camera and, through YOLO segmentation, calculates the car's orientation angle relative to the lane.
**4. Physical Controller:** The `parking_controller_traxxas` node calculates the necessary actuation (using PID control for steering) and translates speeds and angles into PWM signals. It also communicates via serial to receive ultrasonic telemetry from the onboard microcontrollers.

### Topic Flow

```text
       /scan (LiDAR)
             │
     ┌───────▼────────────┐        /parking/perception      ┌──────────────────┐
     │  lidar_processor   │ ───────────────────────────────►│   fsm_parking3   │
     └────────────────────┘                                 └─────────┬────────┘
                                                                      │
     ┌────────────────────┐         /parking/state                    │
     │ lane_detector      │ ◄─────────────────────────────────────────┘
     └───────┬────────────┘
             │ /qcar/lane_angle_ema
             ▼
     ┌──────────────────────────────┐
     │ parking_controller_traxxas   │ ────► /parking/state_feedback (FSM Feedback)
     └───────┬───────────────▲──────┘
             │               │
  direction_servo            │ Ultrasonic & Battery Readings (Serial)
  throttle_motor             │
             ▼               │
     (ESP32 / Arduino / Traxxas ESC)
```

---

## Main Nodes

### `fsm_parking3` — Finite State Machine
**Executable:** `fsm_parking3`

Coordinates the parking maneuver flow from start to finish.
*   **Main phases:** Starts in `ORIENTANDOSE` (Orienting), moves to space searching (`BUSCAR_CAJA`, `BUSCAR`), confirms distances, and iteratively proceeds to park (`ALEJARSE`, `ENTRAR_DIAGONAL`, `ENDEREZAR`, `ESTACIONANDO`) until finishing in `STOP`.
*   **Dynamic decision:** The vehicle automatically detects if the first reference box (adjacent vehicle) is on the right or left and adjusts the parking direction accordingly.
*   **Topics:**
    *   Publishes: `/parking/state` (String with the state and assigned side).
    *   Subscribes: `/parking/perception` (To evaluate LiDAR), `/parking/state_feedback` (Confirmation signals from the motor controller).

### `lidar_processor` — Point Cloud Translator
**Executable:** `lidar_processor`

Cleans the RPLiDAR reading and summarizes it into boolean free-space flags.
*   **Geometric segmentation:** Converts radians to degrees (0 to 359) and segments data into 4 key zones for proximity evaluation: Front, Rear, Right, and Left.
*   **Safety margins:** Evaluates if measurements exceed predefined thresholds, such as 0.20m at the front (`FRONT_STOP_DIST`) and 0.30m on the sides (`RIGHT_FREE_DIST`, `LEFT_FREE_DIST`).
*   **Topics:**
    *   Publishes: `/parking/perception` (String formatted as `FC1 RF1 LF0 RC1` indicating free sides).
    *   Subscribes: `/scan` (Laser points).

### `lane_detector_camera` — Visual Perception and Orientation
**Executable:** `lane_detector_camera`

Uses the ZED 2 stereo camera and AI models to orient the vehicle within the lane.
*   **YOLO Segmentation & Processing:** Uses a TensorRT model (`best_m.engine`) to segment the lane. Filters the mask using Canny algorithms and the Hough Transform (`HoughLinesP`) applied over calibrated Regions of Interest (ROI) for each eye (left and right cameras).
*   **Multithreading Performance:** To maintain low latency, ZED frame capture occurs in a producer thread (`_zed_producer`), while left and right image processing happens in parallel using a `ThreadPoolExecutor`.
*   **Virtual Lane Injection:** Implements logic to construct a virtual lane edge leveraging the ROI polygon on the left camera if needed to improve the calculation angle.
*   **EMA Filter:** Calculates and publishes a smoothed angle using an Exponential Moving Average (EMA with alpha=0.7) to prevent sudden steering changes.
*   **Topics:**
    *   Publishes: `/qcar/lane_angle_raw` (Float32, raw angle), `/qcar/lane_angle_ema` (Float32, filtered angle).

### `parking_controller_traxxas` — PID Controller and Actuation
**Executable:** `parking_controller_traxxas`

The driving brain that translates FSM orders into movements and prevents collisions.
*   **Serial Communication:** Connects to the onboard microcontroller (default `/dev/ttyUSB1` at 115200 baud) to read the 3 ultrasonic sensors and the IR sensor at extremely high speeds.
*   **Orientation Control:** During the `ORIENTANDOSE` and `ENDEREZAR` phases, it uses a Proportional-Derivative (`KP_ORIENT`, `KD_ORIENT`) control loop to calculate the dynamic steering center in PWM signals.
*   **Survival System:** Incorporates safety locks that detect if any ultrasonic sensor reports a hazard closer than 15 cm (`LATERAL_DANGER_DIST`, `REAR_DANGER_DIST`), forcing the car to correct its trajectory by making automatic forward and backward adjustments.
*   **Topics:**
    *   Publishes: `direction_servo`, `throttle_motor`, `/led_power`, `/parking/state_feedback`.
    *   Subscribes: `/imu/euler`, `/qcar/lane_angle_ema`, `/parking/perception`, `/parking/state`.

---

## Diagnostic and Testing Scripts

The package includes standalone nodes to isolate and diagnose physical issues on the track:
*   **`test_sensors.py`:** Monitor to exclusively visualize the Serial connection, cleanly displaying ultrasonic and IR sensor responses while validating battery voltage status.
*   **`test_motors.py`:** Non-blocking console interface that allows assigning continuous open-loop speeds and instantly brakes the vehicle by pressing the "a" key.
*   **`test_freno.py`:** Specialized test to calibrate slow reverse and validate total emergency stops; permanently halts the software if the center sensor locates an obstacle closer than 8 cm.

---

## Build & Run

**Build the package in ROS2:**
```bash
cd ~/path_to_your_workspace
colcon build --packages-select parking_system
source install/setup.bash
```

**Launch the full environment:**
The `parking_system.launch.py` file is the main bringup of the system. It automatically launches the ZED camera, LiDAR, odometry, and the complete parking logic simultaneously.
```bash
ros2 launch parking_system parking_system.launch.py
```
*To modify the physical LiDAR port when launching, you can overwrite the argument:*
```bash
ros2 launch parking_system parking_system.launch.py lidar_serial_port:=/dev/ttyUSB0
```
