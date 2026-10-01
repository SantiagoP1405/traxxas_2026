# Autonomous Parking Simulation

## Note on Parking Simulation

This package contains the original autonomous parking algorithms. The virtual environment they were designed for (`parking_world.sdf`) is located in the `traxxas_robot_simulation` package. It is important to note that **these nodes were developed and validated using a previous version of the robot model (URDF/Xacro)**.

While the simulation world can be loaded using the current architecture, the control and vision algorithms in this package will require adaptations to work with the final robot model.

**Main known discrepancies to be resolved by future developers:**
*   **Topic Mapping:** Topic names have changed in the new model. For example, the original algorithm listens to the `/scan` topic, but the current robot publishes the lidar data on `/traxxas/scan`.
*   **Transforms (TF) and Dimensions:** The kinematics and sensor positions in the current URDF differ from the model used in this simulation, which can affect distance calculations and blind spots.

These scripts are kept as an architectural reference and starting point for anyone who wishes to reactivate the parking test in the virtual environment.

---

## Node Architecture

### 1. Parking Controller (`parking_controller.py`)
This node acts as the hybrid controller that executes the physical maneuvers of the vehicle and includes a rescue logic to avoid collisions.
*   **Main Function:** Processes the vehicle's orientation and rear sensor distances to calculate linear and angular velocity commands. It features evasive maneuvers if the lateral ultrasonic sensors detect obstacles closer than 0.15 meters during the straightening phase.
*   **Subscriptions:** Listens to the current maneuver state on `/parking/state`, orientation on `/imu`, and distances on `/ultrasonic/rear_left`, `/ultrasonic/rear_right`, and `/ultrasonic/rear_center`.
*   **Publications:** Sends velocities to the `/cmd_vel` and `/qcar/user_command` topics, and feeds back the progress (e.g., `ENDEREZADO`, `DIST_OK`) to the system via `/parking/state_feedback`.

### 2. LiDAR Processor (`lidar_processor.py`)
This node is responsible for perimeter environment perception using the laser sensor.
*   **Main Function:** Filters scan data into four regions (front, right, left, and rear) to determine if there is free space to park.
*   **Subscriptions:** Receives laser data on the `/scan` topic.
*   **Publications:** Emits a text message on `/parking/perception` indicating the binary state of each region, where 1 means clear and 0 means blocked by an obstacle.

### 3. Finite State Machine (`fsm_parking.py`)
This script is the high-level logical core that orchestrates the maneuver sequences.
*   **Main Function:** Manages the process through the `BUSCAR`, `ESPERAR`, `ENTRAR_DIAGONAL`, `ENDEREZAR`, `ESTACIONANDO`, and `STOP` states. It evaluates spatial perception to determine if the parking box is on the left or right, and manages the time limits for each phase.
*   **Subscriptions:** Monitors environment availability on `/parking/perception` and physical progress on `/parking/state_feedback`.
*   **Publications:** Dictates the current action and direction to the controller via `/parking/state` (e.g., `ENTRAR_DIAGONAL:RIGHT`).
