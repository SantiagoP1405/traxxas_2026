# AI_models

Trained models used by the `traxxas_lane_detection` package (Torneo Mexicano de
Robótica). All of them were trained on **Google Colab with an NVIDIA A100 (40 GB) GPU**
using **Ultralytics YOLOv8** (imgsz 640, AdamW, 200 epochs with early stopping).

---

## Models

| File | Task | Base model | Size | Runs on | Used by |
|------|------|-----------|------|---------|---------|
| `best_m.pt` | Lane **segmentation** (1 class: `carril`) | `yolov8m-seg` | **medium** (~27.2 M params) | the **Jetson** (on-board) | `lane_detector_yolo` |
| `lane_detector_L.pt` | Lane **segmentation** (1 class: `carril`) | `yolov8l-seg` | **large** | the **external laptop** | `lane_detector_yolo_external`, `lane_detector_pure_pursuit` |
| `crosswalks_alto` (deployed as `crosswalks_alto.engine`) | **Detection** of `Stop` and `crosswalk` (bounding boxes) | `yolov8m` | **medium** (~25.8 M params) | the laptop, for traffic-sign response | `lane_detector_pure_pursuit` |

- **`best_m.pt`** is the lane model that ran **on the Jetson**. It is the lighter (medium)
  network so it fits the Jetson's GPU.
- **`lane_detector_L.pt`** is the **large** lane model. It started being used **once we
  moved the processing off-board** (Jetson → laptop), since the laptop GPU can run the
  bigger network for better accuracy.
- **`crosswalks_alto`** is the stop/crosswalk detector. The lane models are segmentation;
  this one is a plain detection model (bounding boxes).

> The `.engine` files are TensorRT builds tied to a specific device/JetPack and are **not
> portable** — they must be re-exported from the corresponding `.pt` on the target machine.

---

## Training

All training was done on **Google Colab (A100 40 GB)** with Ultralytics YOLOv8.

### Lane segmentation — medium (`best_m`)
- **Dataset:** `cacahuate-v2` (Roboflow), 1 class `carril` (~7.9 k train / 492 val images).
- **Setup:** `yolov8m-seg`, imgsz 640, batch 64, AdamW, 200 epochs (early-stopped at 92,
  best at epoch 72).
- **Best validation metrics:**
  - Mask mAP50: **0.916** · mAP50-95: **0.727**
  - Box mAP50: **0.903** · mAP50-95: **0.826**
- **Training logs** (`args.yaml` + `results.csv`) are in `results_lane_detector_m/`.

### Lane segmentation — large (`lane_detector_L`)
- Same dataset (`cacahuate-v2`), trained as the large variant (`yolov8l-seg`) for the
  off-board laptop. Its standalone training logs are not included in this folder.

### Stop / crosswalk detection (`crosswalks_alto`)
- **Dataset:** `Crosswalk-2` (Roboflow), 2 classes: `Stop` (0), `crosswalk` (1)
  (~36.7 k train / 2.16 k val images).
- **Setup:** `yolov8m` (detection), imgsz 640, batch 64, AdamW, 200 epochs (early-stopped
  at 44, best at epoch 24).
- **Best validation metrics:**
  - All: mAP50 **0.948** · mAP50-95 **0.696**
  - `Stop`: mAP50 **0.98** · mAP50-95 **0.824**
  - `crosswalk`: mAP50 **0.916** · mAP50-95 **0.567**

---

## Datasets (Roboflow)

The datasets live on Roboflow. You can **clone them and add new images** for future
training runs.

### 1. Lane segmentation dataset (`cacahuate-v2`)
- Class: `carril`
- **Link:** _(pegar aquí el link del workspace/proyecto de Roboflow)_

### 2. Stop / crosswalk dataset (`Crosswalk-2`)
- Classes: `Stop`, `crosswalk`
- **Link:** _(pegar aquí el link del workspace/proyecto de Roboflow)_

---

## Known limitations & future work

- **The lane detection needs improvement — both the models and the code.** See the package
  README (the "S" section problem) for the algorithmic side.
- **Overfitting to our own track.** Both lane models (medium and large) were trained on a
  dataset limited to our own track, so they generalize poorly to other layouts. The main
  next step is to **collect more data from more tracks** and retrain.
- **Use the recorded run for more data.** A video of the track was recorded and is stored in
  `traxxas_pruebas` as `video_pista_18abril_TMR`. Extract frames from it to add more (and
  more varied) images to the lane dataset, since right now we only have footage of our own
  track.
- After adding images, clone the Roboflow datasets above, merge the new frames, and retrain.

---

