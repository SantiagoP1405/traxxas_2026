# traxxas_lane_detection

Lane-detection and perception package for the autonomous Traxxas car built for the
**Torneo Mexicano de Robótica (TMR)**. It covers the camera streaming from the
on-board Jetson and the lane-following controllers, in both an **on-board** and a
**distributed (Jetson + laptop)** configuration.

> **Scope of this package.** This is the **lane-detection** part of the project.
> The serial / IMU / odometry bringup lives in the separate `traxxas_pose_estimation`
> package.

---

## System overview

The car can run in two configurations:

**1. On-board (all-in-one).** `lane_detector_yolo` runs directly on the Jetson: it opens
the ZED, runs YOLO, computes the steering and publishes the motor commands locally.
Simplest to deploy, but YOLO inference saturates the Jetson GPU.

**2. Distributed (Jetson → laptop).** The Jetson runs only a thin streamer
(`jetson_transmisor` or `jetson_transmisor_point_cloud`) that publishes the compressed
stereo image. An off-board node on a laptop (`lane_detector_yolo_external` or
`lane_detector_pure_pursuit`) runs YOLO + control and publishes the PWM commands back.
This offloads the heavy inference from the Jetson onto a laptop GPU.

### Topic flow (distributed mode)

```
            ZED (stereo)
                │
        ┌───────▼────────────┐         /zed/stereo/compressed
        │  Jetson streamer   │ ─────────────────────────────────►  ┌──────────────────┐
        │  jetson_transmisor │                                      │  Laptop controller│
        └────────────────────┘                                      │  (YOLO + control) │
                                                                     └─────────┬────────┘
   direction_servo / throttle_motor / led_power                                │
        ◄──────────────────────────────────────────────────────────────────────┘
                │
        (serial bridge on the car → ESC + steering servo + LED)
```

The point-cloud streamer additionally publishes `/zed/depth_registered` and
`/zed/xyzrgba_image` for a future obstacle-detection node.

---

## Hardware & PWM calibration

The raw PWM values used everywhere in this package — the **steering center**, the
**max-steer mapping**, and the **minimum throttle** needed for the car to start moving —
are **motor-dependent**. They were calibrated by mounting an IMU on the wheel and
recording the PWM → steering-angle relationship. That calibration table is included in
this package as [`pwm.xlsx`](./pwm.xlsx) (PWM vs degrees, for both 16-bit and 14-bit
representations).

This calibration was done on the **Traxxas v1**. The team later switched to a
**Traxxas v2** (the v1 exceeded the size limits), and there was no time to fully re-map
the v2's PWM to 14 bits. The PWM values currently in the code are the ones the v2 moved
well with empirically, even though they are not fully characterized — **they should be
properly re-mapped for the v2.**

### Pixel ↔ meters (`ym`, `xm`)

The curvature math converts pixels to meters using two camera-dependent scales:

- **`ym`** — the longitudinal distance the camera sees, over the image height
  (`meters seen / 720`). Example: `1.0/720` in the on-board node, `1.6/720` in the
  external variant.
- **`xm`** — the lane width over the warped lane width (`0.40 m / 384 px`).

They must be re-measured for a different camera, mounting height or warp geometry.
(See the note in `lane_detector_yolo` about why this matters less than it used to.)

---

## Nodes

### `jetson_transmisor` — Jetson stereo streamer

**Node name:** `jetson_stereo_bridge` · **Runs on:** the car's Jetson (ZED attached).

Minimal camera bridge that lives on the Jetson and streams the ZED stereo pair to an
external PC. It is the entry point of the **distributed mode**: by shipping the raw image
off-board, the heavy YOLO inference can run on a laptop GPU instead of the Jetson.

**What it does:** opens the ZED at HD720/30 fps, grabs each frame, retrieves the LEFT and
RIGHT views, stacks them side by side into a single 2560×720 image, JPEG-encodes it
(quality 70) and publishes it at 30 Hz.

**Topics**
- Publishes: `/zed/stereo/compressed` (`sensor_msgs/CompressedImage`, best-effort QoS).
- Subscribes: none.

**Key parameters (in code):** camera resolution `HD720`, `camera_fps = 30`,
JPEG quality `70`, timer at 30 Hz.

**Areas of improvement**
- Allocates new `sl.Mat()` objects inside the callback on every frame → reuse persistent
  buffers (as the point-cloud variant already does) to avoid per-frame reallocation.
- Resolution and JPEG quality are hardcoded; expose them as ROS parameters so the
  bandwidth/quality trade-off can be tuned without editing code.
- Streams only RGB; no depth or point cloud (that is the role of the point-cloud variant).

---

### `jetson_transmisor_point_cloud` — Jetson streamer with depth & point cloud

**Node name:** `jetson_stereo_bridge` · **Runs on:** the car's Jetson (ZED attached).

Extended version of the streamer: it sends the same compressed stereo pair **plus**
per-pixel depth and the full XYZRGBA point cloud, so an obstacle-detection node can reason
about 3D space. It enables the ZED depth engine and reuses persistent buffers to avoid
per-frame reallocation.

**What it does:** opens the ZED at HD720/30 with `DEPTH_MODE.PERFORMANCE`, metric units and
a right-handed Y-up frame; on every grab it retrieves the stereo image, the depth measure
and the XYZRGBA measure, then publishes all three at 30 Hz. The XYZRGBA channel layout
(X/Y/Z + packed RGBA) is meant to let a `see_obstacle_node` reuse its existing
`point_cloud_np[:, :, :3]` logic.

**Topics**
- Publishes:
  - `/zed/stereo/compressed` (`CompressedImage`, JPEG)
  - `/zed/depth_registered` (`Image`, `32FC1`, meters)
  - `/zed/xyzrgba_image` (`Image`, `32FC4`)
- Subscribes: none.

**Key parameters (in code):** `HD720` / `30 fps`, `DEPTH_MODE.PERFORMANCE`, `UNIT.METER`,
`COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP`, JPEG quality `70`.

**Areas of improvement**
- **Bandwidth:** depth and XYZRGBA are published uncompressed (~110 MB/s and ~440 MB/s at
  720p/30 Hz). They will not survive a Wi-Fi link — run the consumer on the Jetson, or
  throttle/downsample/compress these topics before sending them off-board.
- **`cv_bridge` dependency:** the depth/XYZRGBA messages are built with `cv_bridge`, which
  on this Jetson setup has clashed with the installed NumPy. Verify it imports and runs
  before relying on these topics.

---

### `lane_detector_yolo` — on-board lane follower (baseline)

**Node name:** `lane_detector_node` · **Runs on:** the car's Jetson (opens the ZED directly).

The original, self-contained lane-following controller: capture, perception and steering
all run in a single process on the Jetson. It opens the ZED through `pyzed` and grabs
frames in a background producer thread.

**Pipeline (per eye):** YOLO segmentation mask → ROI mask → bird's-eye warp →
sliding-window quadratic fit → curvature. A linear *slope* is also fit on the bottom 60% of
the warped mask (cleaner zone, without the opposite lane).

**Steering:** adaptive fusion of three estimators — sliding-window curvature, warped-lane
slope, and the previous frame's angle — weighted by how tight the curve is
(`W_STRAIGHT`/`W_CURVE` interpolated by `curve_level`), then mapped to a servo PWM.

**Design intent:** the algorithm was built to run *fast*. The sliding window deliberately
looks well ahead and reconstructs the upcoming track so the car can anticipate the geometry
several moments in advance instead of reacting late.

**Parallelization:** capture and processing are decoupled to keep per-frame latency low.
A background producer thread grabs ZED frames into a 1-slot queue so the control loop never
blocks on the camera (and always works on the freshest frame), and the two eyes are
processed concurrently with a `ThreadPoolExecutor` (one worker per eye) — ROI, warp and
sliding window run in parallel for left and right instead of sequentially.

**Topics**
- Publishes: `direction_servo` (`String`, steering PWM), `throttle_motor`
  (`String`, constant `2610`).
- Input: ZED camera, read directly (no ROS image topic).

**Key parameters (top of file):** `MODEL_PATH`, `RUTA_SVO`/`USE_SVO` (offline SVO playback),
`CONF`, `SLOPE_MAX_DEG=22`, fusion weights, `delta_to_pwm` clamped to ±25°, curvature scale
`ym=1.0/720`, `xm=0.40/384`.

**Calibration & tuning**
- **`SLOPE_MAX_DEG`** — tuned empirically against the real track with the `control_traxxas`
  node: the car was driven through the course to find, roughly, the steering angle (mapped
  to PWM) needed to take each curve, and `SLOPE_MAX_DEG` was set so the slope estimator maps
  onto those required angles. Track- and car-specific.
- **Fusion weights (`W_STRAIGHT`, `W_CURVE`)** — hardcoded and found by trial-and-error;
  these are simply the values that performed best on track. They are not learned or
  auto-tuned, so they likely need re-tuning for a new track or controller.
- **Pixel ↔ meters (`ym`, `xm`)** — see the shared
  [Hardware & PWM calibration](#pixel--meters-ym-xm) section. *Note:* this is largely moot
  now — because of the new "S" section the whole curvature approach has to be redesigned
  anyway.

**Extras:** SVO playback for offline testing, optional OpenCV visualization, curve-mode
telemetry log, and a SIGINT handler that publishes a safe stop before exiting.

**Areas of improvement**
- **Look-ahead breaks on the new "S" section.** The far-ahead sliding window was an asset on
  the original track, but the course was changed late (a rule change) and now contains an
  S-shaped segment. There the window sees almost the entire "S" at once, so the quadratic fit
  can't resolve a single turn direction or a clean steering angle — that shape injects noise
  into the estimator. A shorter/adaptive look-ahead, or splitting the fit into near/far
  segments, would be needed for this track.
- All paths (model, SVO, log) are hardcoded absolute paths under `/home/traxxas/...` → move
  to ROS parameters or a config file.
- `retina_masks=True` is heavy on the Jetson and caused GPU out-of-memory (ENOMEM); this is
  what motivated the external/laptop variants.
- Throttle is a fixed constant — no speed control (e.g. slowing down in curves).
- No lateral centering (no centroid/offset term) — the car follows curvature but doesn't
  actively recenter in the lane.

---

### `lane_detector_yolo_external` — off-board lane follower + lateral centering

**Node name:** `laptop_brain_node` · **Runs on:** an external PC / laptop (subscribes to the
Jetson stream).

Same perception and 3-factor steering fusion as `lane_detector_yolo`, but moved **off the
car**: instead of opening the ZED, it subscribes to `/zed/stereo/compressed`, decodes the
JPEG and splits the stereo pair into two eyes. This offloads YOLO inference onto a laptop
GPU. On top of the base controller it adds a **lateral centering** term.

**What's different from the base node**
- **Input:** consumes `/zed/stereo/compressed` (no `pyzed`); the Jetson runs
  `jetson_transmisor` as the producer.
- **Lateral centering:** `lateral_offset()` estimates the lane center vs. the car center and
  adds a correction (`OFFSET_GAIN`, `OFFSET_BIAS`) on top of the fused steering angle, so the
  car tries to actively recenter in the lane.
- **Single-eye fallback:** if only one lane is detected, it keeps the steering magnitude from
  that eye and reuses the previous frame's sign (`prev_sliding_sign`).
- **Start delay:** holds neutral for the first 6 s before driving.

**Topics**
- Subscribes: `/zed/stereo/compressed` (`CompressedImage`).
- Publishes: `direction_servo` (`String`), `throttle_motor` (`String`, constant `2700`).

**Key parameters / calibration:** paths under `/home/edwin/...`, `SLOPE_MAX_DEG=30`,
`W_CURVE=[0.30,0.30,0.40]`, wheelbase `L=0.18`, `delta_to_pwm` clamped to ±16° with center
`2525`, `ym=1.6/720`, `xm=0.40/384`, `OFFSET_GAIN=0.05`, `OFFSET_BIAS=1.5`. The shared
pixel↔meters and PWM & motor calibration notes apply.

**Areas of improvement**
- **The centroid does not work well.** The lateral offset is computed from the detected lane
  pixels, which is noisy and unreliable. A more robust approach would derive the lane/car
  center from the camera geometry or the vehicle itself, instead of depending on the
  detected pixels.
- Shares the same far-ahead sliding window as the base node, so the **"S" section problem
  applies here too**.
- Paths are hardcoded under `/home/edwin/...` → move to ROS parameters.
- Throttle is a fixed constant (`2700`) — no speed control.
- `OFFSET_GAIN` / `OFFSET_BIAS` are hardcoded guesses and need empirical calibration.

---

### `lane_detector_pure_pursuit` — off-board Pure Pursuit controller (race build)

**Node name:** `laptop_brain_node` · **Runs on:** an external PC / laptop (subscribes to the
Jetson stream).

Like `lane_detector_yolo_external` it runs off-board and consumes `/zed/stereo/compressed`,
but it replaces the 3-factor fusion with **Pure Pursuit**, undistorts the camera with the
real ZED calibration, and integrates traffic-sign response and an LED status output.

> **⚠️ Status — prototype / ideas, not validated.** This node did **not** work reliably.
> The Pure Pursuit logic (look-ahead, etc.) could not be properly tested: the track was
> changed at the last minute and there was no time left to tune it. Treat it as a starting
> point, not a working controller.

**Pipeline (per eye):** decode stereo → split eyes → **undistort** → YOLO segmentation mask →
ROI → bird's-eye warp → sliding-window quadratic fit. The left/right fits are combined into a
BEV centerline (`center_fit_from_lane_fits`).

**Steering — Pure Pursuit:** projects a look-ahead point at `LOOKAHEAD_DIST_M = 0.60 m` along
the BEV centerline, transforms it into vehicle coordinates and computes the steering angle
geometrically (wheelbase `L = 0.18 m`), then maps it to PWM (±16°, center `2525`).

**Camera undistortion:** each eye is undistorted with the real intrinsics and distortion
coefficients of our specific ZED (per-eye camera matrices + coeffs, applied via
`cv2.undistort`) before the ROI/warp stage. These values depend on the ZED unit — **if you
want to switch to a different ZED, ask profe Daniel how they were obtained.**

**Traffic-sign response (stop / crosswalk)**

*To be documented by the teammate who owns the sign-detection part.*

**Topics**
- Subscribes: `/zed/stereo/compressed` (`CompressedImage`).
- Publishes: `direction_servo` (`String`), `throttle_motor` (`String`), `led_power`
  (`String`, drive/stop state).

**Key parameters / calibration:** `MODEL_PATH` / `SIGNAL_MODEL_PATH` / `CURVE_LOG_PATH` under
`/home/edwin/...`, `LOOKAHEAD_DIST_M=0.60`, `WHEELBASE_L=0.18`, `MAX_STEER_DEG=16`,
`ym=1.0/720`, `xm=0.40/384`. The shared pixel↔meters and PWM & motor calibration notes apply.

**Extras:** optional OpenCV/BEV visualization, and a SIGINT handler that publishes a safe stop
before exiting.

**Areas of improvement**
- Pure Pursuit needs a center fit from **both** lanes; when one lane is missing, `center_fit`
  is `None` and the node falls back to a centered PWM (no steering). A single-lane fallback
  (as in the external node) or a width-based failsafe centerline would make it more robust.
- **Lane-fusion visualization looks wrong.** When the combined two-lane centerline was
  visualized it looked strange — it's unclear whether this is only a rendering issue or an
  actual bug in the center fit. It should be verified and fixed if it really affects the
  control.
- **Not validated / needs odometry.** The look-ahead and the Pure Pursuit logic in general
  could not be tested in time (last-minute track change). For Pure Pursuit to work properly it
  would likely need geometric odometry, or use of the IMU, to track the vehicle's motion/pose
  rather than relying only on the instantaneous centerline.
- Paths and the undistortion intrinsics are hardcoded → move to ROS parameters / a
  calibration file.
- `SHOW_VIZ` is on by default — disable it for headless/race runs to save latency.

---

## Build & run

```bash
cd ~/Workspaces/traxxas_ws
colcon build --packages-select traxxas_lane_detection
source install/setup.bash
```

**On-board (Jetson):**
```bash
ros2 run traxxas_lane_detection lane_detector_yolo
```

**Distributed mode:**
```bash
# On the Jetson (pick one streamer):
ros2 run traxxas_lane_detection jetson_transmisor
ros2 run traxxas_lane_detection jetson_transmisor_point_cloud   # with depth + point cloud

# On the laptop (pick one controller):
ros2 run traxxas_lane_detection lane_detector_yolo_external
ros2 run traxxas_lane_detection lane_detector_pure_pursuit
```

---

## Assets (models & calibration)

- **`pwm.xlsx`** — PWM ↔ steering-angle calibration table (measured with an IMU on the wheel;
  16-bit and 14-bit). See [Hardware & PWM calibration](#hardware--pwm-calibration).
- **`models/`** — the YOLO engines used by the nodes (lane segmentation and
  the stop/crosswalk detector). Note that `.engine` files are TensorRT builds tied to a
  specific device/JetPack and are not portable; this folder will document what each model is
  and how it was trained.

> **Heads-up for whoever inherits this:** most file paths inside the nodes are hardcoded
> absolute paths (`/home/traxxas/...` on the Jetson, `/home/edwin/...` on the laptop). Update
> them to your environment, or better, migrate them to ROS parameters.
