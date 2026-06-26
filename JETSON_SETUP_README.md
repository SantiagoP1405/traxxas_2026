# Jetson setup (Traxxas)

Notes on the software environment and hardware quirks of the **Jetson (Orin)** that runs the
Traxxas car, for whoever inherits the project. JetPack 6.x runs only on Orin-series modules.

---

## Installed software

| Component | Version / notes |
|-----------|-----------------|
| **JetPack** | **6.2** (provides the L4T base, CUDA, cuDNN, TensorRT pinned to this release) |
| **ROS 2** | **Humble** |
| **OpenCV** | built/installed **with CUDA** support (used by the GPU filters, e.g. `cv2.cuda.*`) |
| **PyTorch** | installed from a Jetson-specific prebuilt binary (see note below) |
| **torchvision** | matching the PyTorch build |
| **Ultralytics** | YOLOv8 (inference + TensorRT engine export) |

### Note on PyTorch (the tricky one)

PyTorch on Jetson is **not** a plain `pip install torch` — the standard PyPI wheels have no
CUDA support for the Jetson's ARM/L4T platform. It had to be installed from a **special
prebuilt binary / wheel** matched to JetPack 6.2, which was finicky to get working (the exact
steps weren't recorded). If you ever need to reinstall or upgrade it, **follow NVIDIA's
official "PyTorch for Jetson" instructions for JetPack 6.2** and use the wheel that matches
this JetPack version — don't install from PyPI directly.

### Quick verification

```bash
jtop                                   # JetPack, CUDA, cuDNN, TensorRT, load
python3 -c "import torch; print(torch.__version__, torch.cuda.is_available())"
python3 -c "import cv2; print(cv2.cuda.getCudaEnabledDeviceCount())"   # >0 means CUDA OpenCV
ros2 --version
```

---

## Hardware quirk — cooling fan

The **original Jetson fan stopped working** due to a **firmware issue**. The Jetson normally
controls the fan via PWM (0–100%), ramping it up or down based on GPU/CPU load, but on this
unit that firmware control failed. The cause was investigated and **never found** — and
notably, **the same original fan works perfectly on another Jetson Orin Nano**, so the fan
itself is fine; the problem is specific to this board's fan control.

**Workaround in place:** a **generic fan** was wired in that runs **always on at a fixed
speed** (no PWM control). It keeps the board cool, but it does **not** vary with temperature
like the original one did.

> If you reflash or update the board, it's worth re-checking whether the original PWM fan
> control comes back 
