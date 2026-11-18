"""Colour-histogram 'fingerprint': no neural net, no download.

Splits the person box into an upper (shirt) and lower (trousers) half and takes
a hue/saturation histogram of each. Good enough for the sim's blocky people and
as a fallback; a real re-ID network (onnx_reid.py) is far better on real people
(clothes texture, body shape, bags...)."""
from __future__ import annotations

from typing import Optional

import cv2
import numpy as np

from drone_core.domain.models import BoundingBox


class HistogramEncoder:
    match_threshold = 0.9      # cosine similarity that counts as "same person"

    def __init__(self, h_bins: int = 18, s_bins: int = 4, parts: int = 2):
        self.h_bins, self.s_bins, self.parts = h_bins, s_bins, parts

    def embed(self, image: np.ndarray, boxes: list[BoundingBox]) -> list[Optional[np.ndarray]]:
        H, W = image.shape[:2]
        out: list[Optional[np.ndarray]] = []
        for b in boxes:
            # shrink 10% sideways: box edges pick up background
            x1 = int(max(0, b.x1 + 0.1 * b.w)); x2 = int(min(W, b.x2 - 0.1 * b.w))
            y1 = int(max(0, b.y1)); y2 = int(min(H, b.y2))
            if x2 - x1 < 3 or y2 - y1 < 6:
                out.append(None)
                continue
            hsv = cv2.cvtColor(image[y1:y2, x1:x2], cv2.COLOR_BGR2HSV)
            feats = []
            n = hsv.shape[0]
            for k in range(self.parts):
                part = hsv[k * n // self.parts:(k + 1) * n // self.parts]
                # dark / grey pixels have meaningless hue: count them in a value histogram instead
                hist = cv2.calcHist([part], [0, 1], None, [self.h_bins, self.s_bins], [0, 180, 0, 256]).ravel()
                v = cv2.calcHist([part], [2], None, [4], [0, 256]).ravel()
                f = np.concatenate([hist, 0.5 * v])
                feats.append(np.sqrt(f / max(f.sum(), 1e-6)))      # Hellinger: robust to lighting
            v = np.concatenate(feats).astype(np.float32)
            out.append(v / max(np.linalg.norm(v), 1e-6))
        return out
