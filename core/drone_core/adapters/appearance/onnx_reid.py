"""Re-ID network via onnxruntime: person crop -> 512-d appearance vector.

Default model: Ultralytics yolo26n-reid.onnx (2.8M params, trained on MSMT17),
downloaded on first use from github.com/ultralytics/assets. Any ONNX re-ID model
with input [batch, 3, H, W] RGB 0..1 works (e.g. an exported OSNet; pass
mean/std if it expects ImageNet normalisation).

pip install onnxruntime   (or onnxruntime-gpu)"""
from __future__ import annotations

import os
import urllib.request
from typing import Optional

import cv2
import numpy as np

from drone_core.domain.models import BoundingBox

DEFAULT_URL = "https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26n-reid.onnx"


def ensure_model(path: str = "yolo26n-reid.onnx", url: str = DEFAULT_URL) -> str:
    if not os.path.exists(path):
        print(f"downloading re-ID model to {path} ...")
        urllib.request.urlretrieve(url, path)
    return path


class OnnxReidEncoder:
    match_threshold = 0.75

    def __init__(self, model_path: str = "yolo26n-reid.onnx", imgsz: Optional[int] = None,
                 mean=None, std=None, providers=None, max_batch: int = 8):
        import onnxruntime as ort
        model_path = ensure_model(model_path) if os.path.basename(model_path) == "yolo26n-reid.onnx" else model_path
        self.session = ort.InferenceSession(model_path, providers=providers or ort.get_available_providers())
        inp = self.session.get_inputs()[0]
        self.input_name = inp.name
        meta = self.session.get_modelmeta().custom_metadata_map
        if imgsz is None:
            shp = inp.shape
            if isinstance(shp[2], int) and isinstance(shp[3], int):
                imgsz = (shp[2], shp[3])
            elif "imgsz" in meta:
                imgsz = tuple(int(x) for x in meta["imgsz"].strip("[]").split(","))
            else:
                imgsz = (256, 128)
        self.imgsz = imgsz if isinstance(imgsz, tuple) else (imgsz, imgsz)
        fixed = inp.shape[0]
        self.fixed_batch = fixed if isinstance(fixed, int) else None
        self.max_batch = max_batch
        self.mean = None if mean is None else np.array(mean, np.float32).reshape(1, 3, 1, 1)
        self.std = None if std is None else np.array(std, np.float32).reshape(1, 3, 1, 1)

    def _prep(self, crop: np.ndarray) -> np.ndarray:
        rgb = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)
        rgb = cv2.resize(rgb, (self.imgsz[1], self.imgsz[0]), interpolation=cv2.INTER_LINEAR)
        return rgb.transpose(2, 0, 1).astype(np.float32) / 255.0

    def embed(self, image: np.ndarray, boxes: list[BoundingBox]) -> list[Optional[np.ndarray]]:
        H, W = image.shape[:2]
        crops, keep = [], []
        for k, b in enumerate(boxes):
            x1, y1 = int(max(0, b.x1)), int(max(0, b.y1))
            x2, y2 = int(min(W, b.x2)), int(min(H, b.y2))
            if x2 - x1 >= 4 and y2 - y1 >= 8:
                crops.append(self._prep(image[y1:y2, x1:x2]))
                keep.append(k)
        out: list[Optional[np.ndarray]] = [None] * len(boxes)
        if not crops:
            return out
        bs = self.fixed_batch or self.max_batch
        feats = []
        for s in range(0, len(crops), bs):
            chunk = np.stack(crops[s:s + bs])
            n = len(chunk)
            if self.fixed_batch and n < bs:
                chunk = np.concatenate([chunk, np.repeat(chunk[-1:], bs - n, 0)])
            if self.mean is not None:
                chunk = (chunk - self.mean) / self.std
            feats.append(self.session.run(None, {self.input_name: chunk})[0][:n])
        feats = np.concatenate(feats).astype(np.float32)
        feats /= np.maximum(np.linalg.norm(feats, axis=1, keepdims=True), 1e-6)
        for k, f in zip(keep, feats):
            out[k] = f
        return out
