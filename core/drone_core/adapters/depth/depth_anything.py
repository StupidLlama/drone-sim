"""DepthEstimator using a Depth Anything V2 *metric* model via Hugging Face
transformers. Real-drone only; the sim sends ground-truth depth.

STATUS: written against the transformers depth-estimation API but NOT run here
(no access to the model weights from the build environment). Check the output
scale against a tape measure before trusting the map.

    pip install transformers
    python main.py --mode real --camera 0,1,2,3 \
        --depth-model depth-anything/Depth-Anything-V2-Metric-Outdoor-Small-hf
"""
from __future__ import annotations

import cv2
import numpy as np

from drone_core.domain.models import Frame


class DepthAnythingEstimator:
    def __init__(self, model: str = "depth-anything/Depth-Anything-V2-Metric-Outdoor-Small-hf",
                 device=None, out_size=(64, 48)):
        import torch
        from transformers import AutoImageProcessor, AutoModelForDepthEstimation
        self.torch = torch
        self.device = device if device not in (None, "") else ("cuda" if torch.cuda.is_available() else "cpu")
        if str(self.device).isdigit():
            self.device = f"cuda:{self.device}"
        self.processor = AutoImageProcessor.from_pretrained(model)
        self.model = AutoModelForDepthEstimation.from_pretrained(model).to(self.device).eval()
        self.out_size = out_size

    def estimate(self, frame: Frame) -> np.ndarray:
        rgb = cv2.cvtColor(frame.image, cv2.COLOR_BGR2RGB)
        inputs = self.processor(images=rgb, return_tensors="pt").to(self.device)
        with self.torch.no_grad():
            pred = self.model(**inputs).predicted_depth          # (1, H', W') metres for metric models
        depth = pred[0].float().cpu().numpy()
        # The map only needs a coarse grid; downsample with a min-pool so thin
        # obstacles (poles) aren't averaged away.
        w, h = self.out_size
        ys = np.linspace(0, depth.shape[0], h + 1).astype(int)
        xs = np.linspace(0, depth.shape[1], w + 1).astype(int)
        out = np.empty((h, w), np.float32)
        for j in range(h):
            for i in range(w):
                out[j, i] = depth[ys[j]:ys[j + 1], xs[i]:xs[i + 1]].min()
        # metric models return depth along the optical axis; the map wants range
        # along each pixel's ray, close enough for a 90-degree lens at this resolution
        return out
