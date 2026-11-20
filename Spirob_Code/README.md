# Spirob_Code
This is the control program for Spirob Arm

## Optional experiment video

With OpenCV installed, start the webcam recorder before running an experiment.

```python
from Spirob_Code.core_nodes.vision.video_recorder import start_recording

stop_recording = start_recording("runs/experiment-01")
try:
    run_experiment()
finally:
    stop_recording()
```

The recorder uses camera index 0 by default and saves a timestamped MP4 when
a camera is available. Set `camera_index` to select a different camera.
