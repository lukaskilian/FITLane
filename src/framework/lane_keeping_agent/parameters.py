from dataclasses import dataclass
from typing import Optional


@dataclass
class LaneData:
    """Fahrstreifendaten wie Breite, Krümmung etc."""
    lane_width: float
    curvature: float
    change_in_curvature: float
    lateral_deviation: float
    yaw_angle: float


@dataclass
class LateralAcceleration:
    """Querbeschleunigung"""
    value: float


@dataclass
class VehicleControl:
    """Zusammenfassung von Befehlen, die an das Fahrzeug in der Simulation weitergegeben werden sollen, wie
    Gaspedalstellung, Lenkbefehl und Bremsbefehl."""
    steer: float = 0.0
    throttle: Optional[float] = None
    brake: Optional[float] = None
