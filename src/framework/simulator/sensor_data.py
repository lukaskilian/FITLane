import logging
import os
from abc import ABC
from dataclasses import dataclass, field
from typing import Optional, Dict

import cv2
import numpy as np


@dataclass
class SensorData(ABC):
    """Daten, die von Sensor bzw. vom Simulator an den LaneKeepingAgent weitergereicht werden."""
    sensor_id: str
    snapshot_id: Optional[int] = None


@dataclass
class CameraImage(SensorData):
    """Daten, die von einer (simulierten) Kamera an den LaneKeepingAgent weitergereicht werden"""
    img_array: Optional[np.ndarray] = field(default=None, repr=False)
    filename: Optional[str] = field(default=None, repr=False)

    def save_to_disk(self, directory_name: str) -> None:
        self.filename = f"{self.sensor_id}_{self.snapshot_id:06d}.png"
        filepath = os.path.join(directory_name, self.filename)
        cv2.imwrite(filepath, self.img_array)
        logging.debug(f"Bild {self.filename} wurde abgespeichert")

    def to_dict(self) -> Dict:
        return {"sensor_id": self.sensor_id,
                "filename": self.filename}
