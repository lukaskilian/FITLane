from __future__ import annotations

import logging
import math
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Optional, Dict, TYPE_CHECKING

if TYPE_CHECKING:
    from framework.lane_keeping_test.test_protocol import Snapshot


@dataclass
class Location:
    """Hilfsklasse, die einen Ort mit x,y und z Koordinaten speichert"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    @classmethod
    def from_dict(cls, data: Dict):
        return cls(**data)


@dataclass
class Rotation:
    """Hilfsklasse, die die Rotation eines Objekts speichert"""
    pitch: float = 0.0
    yaw: float = 0.0
    roll: float = 0.0

    @classmethod
    def from_dict(cls, data: Dict):
        return cls(**data)


@dataclass
class Transform:
    """Kombination von Location und Rotation"""
    location: Location
    rotation: Rotation

    @classmethod
    def from_dict(cls, data: Dict):
        data["location"] = Location.from_dict(data.get("location", {}))
        data["rotation"] = Rotation.from_dict(data.get("rotation", {}))
        return cls(**data)


class AbstractMap(ABC):
    """Legt die Simulationsumgebung fest."""

    @classmethod
    def from_dict(cls, data: Dict):
        del data["type"]
        return cls(**data)


@dataclass
class SimulatorImportedMap(AbstractMap):
    """Name der zu verwendenden Karte, die im Simulator bereits importiert ist"""
    map_name: str = ""

    def to_dict(self):
        return {"type": "simulator_imported_map",
                "map_name": self.map_name}


@dataclass
class OpenDriveMap(AbstractMap):
    """Der Dateipfad zu einer OpenDrive-Datei."""
    xodr_filepath: str = ""

    def to_dict(self):
        return {"type": "open_drive_map",
                "xodr_filepath": self.xodr_filepath}


@dataclass
class WeatherParameters:
    """Wetter-Parameter in der Simulationsumgebung"""
    cloudiness: float = 0.0
    precipitation: float = 0.0
    precipitation_deposits: float = 0.0
    wind_intensity: float = 0.0
    sun_azimuth_angle: float = 0.0
    sun_altitude_angle: float = 0.0
    fog_density: float = 0.0
    fog_distance: float = 0.0
    wetness: float = 0.0
    fog_falloff: float = 0.2

    @classmethod
    def from_dict(cls, data: Dict):
        return cls(**data)


class TerminationCondition(ABC):
    """Abbruchbedingungen zum vorzeitigen Abbrechen eines SimulationRuns"""

    @abstractmethod
    def initialize(self) -> None:
        """Methode, die zu Beginn eines SimulationRuns aufgerufen wird"""

    @abstractmethod
    def is_condition_met(self, snapshot: Snapshot) -> bool:
        """Methode zum Abfragen, ob die Abbruchbedingung erfüllt ist"""

    @classmethod
    def from_dict(cls, data: Dict):
        del data["type"]
        return cls(**data)


@dataclass
class CrossingLaneMarking(TerminationCondition):
    """Abbruchbedingung zum Abbruch beim Überschreiten der Linie"""

    def initialize(self) -> None:
        """muss nicht initialisiert werden"""

    def is_condition_met(self, snapshot: Snapshot) -> bool:
        if snapshot.driving_data.distance_to_lane_marking_front_left is None \
                or snapshot.driving_data.distance_to_lane_marking_front_right is None:
            return False
        else:
            return snapshot.driving_data.distance_to_lane_marking_front_left < 0.0 \
                   or snapshot.driving_data.distance_to_lane_marking_front_right < 0.0

    def to_dict(self):
        return {"type": "crossing_lane_marking"}


@dataclass
class StepLimit(TerminationCondition):
    """Abbruchbedingung zum Abbruch beim Erreichen eines Zeitlimits"""
    limit: int
    __current_number_of_steps: int = -1

    def initialize(self) -> None:
        self.__current_number_of_steps = -1

    def is_condition_met(self, snapshot: Snapshot) -> bool:
        self.__current_number_of_steps += 1
        return self.__current_number_of_steps >= self.limit

    def to_dict(self):
        return {"type": "step_limit",
                "limit": self.limit}


@dataclass
class TargetPointReached(TerminationCondition):
    """Abbruchbedingung zum Abbruch beim Erreichen eines Zielpunkts.
       tolerance beschreibt den Puffer in alle Richtungen"""
    target_location: Location
    tolerance: float = 5.0

    def initialize(self) -> None:
        """muss nicht initialisiert werden"""

    def is_condition_met(self, snapshot: Snapshot) -> bool:
        current_location = snapshot.driving_data.vehicle_transform.location
        return math.isclose(self.target_location.x, current_location.x, abs_tol=self.tolerance) and \
               math.isclose(self.target_location.y, current_location.y, abs_tol=self.tolerance) and \
               math.isclose(self.target_location.z, current_location.z, abs_tol=self.tolerance)

    @classmethod
    def from_dict(cls, data: Dict):
        del data["type"]
        data["target_location"] = Location.from_dict(data.get("target_location", {}))
        return cls(**data)

    def to_dict(self):
        return {"type": "target_point_reached",
                "target_location": self.target_location,
                "tolerance": self.tolerance}


@dataclass
class SensorInformation:
    """Daten über einen verwendeten Sensor"""
    sensor_id: str
    name: str
    mounting_point: Transform

    @classmethod
    def from_dict(cls, data: Dict):
        data["mounting_point"] = Transform.from_dict(data.get("mounting_point", {}))
        del data["type"]
        return cls(**data)


@dataclass
class CameraInformation(SensorInformation):
    """Daten über eine verwendete Kamera"""
    image_size_x: int = 800
    image_size_y: int = 600
    fov: int = 90


@dataclass
class VehicleInformation:
    """Daten über das verwendete Fahrzeug"""
    name: str
    sensors: List[SensorInformation]
    wheelbase: Optional[float] = None
    track: Optional[float] = None
    length: Optional[float] = None
    width: Optional[float] = None

    @classmethod
    def from_dict(cls, data: Dict):
        return cls(**data)


@dataclass
class TestSpecification:
    """Spezifikation für eine Test-Ausführung (TestExecution); enthält die Anzahl der SimulationRuns,
        die vom Simulator zu verwendende Simulationsumgebung (Map),
        des Start und Endpunkts auf der gewählten Karte, die Abbruchbedingungen (TerminationCondition), sowie
        Informationen zum Fahrzeug (VehicleInformation) und zu den Sensoren (SensorInformation), einschließlich
        Kameras (ImageInformation), und zu Wetter/Lichteinstellungen des Simulators."""
    map: AbstractMap
    starting_point: Transform
    weather: WeatherParameters
    termination_conditions: List[TerminationCondition]
    ego_vehicle: VehicleInformation

    simulation_runs: int = 1
    start_speed: float = 0.0
    fixed_throttle: float = 0.0
    use_autopilot: bool = False

    @classmethod
    def from_dict(cls, data: Dict):
        map_dict = data.get("map", {})
        if map_dict.get("type", "") == "simulator_imported_map":
            data["map"] = SimulatorImportedMap.from_dict(map_dict)
        elif map_dict.get("type", "") == "open_drive_map":
            data["map"] = OpenDriveMap.from_dict(map_dict)

        starting_point = data.get("starting_point", None)
        if starting_point:
            data["starting_point"] = Transform.from_dict(starting_point)

        data["weather"] = WeatherParameters.from_dict(data.get("weather", {}))

        termination_conditions = []
        for condition_dict in data.get("termination_conditions", []):
            if condition_dict.get("type", "") == "step_limit":
                termination_conditions.append(StepLimit.from_dict(condition_dict))
            elif condition_dict.get("type", "") == "crossing_lane_marking":
                termination_conditions.append(CrossingLaneMarking.from_dict(condition_dict))
            elif condition_dict.get("type", "") == "target_point_reached":
                termination_conditions.append(TargetPointReached.from_dict(condition_dict))
            else:
                logging.warning("Unbekannter Typ für TerminationCondition")
        data["termination_conditions"] = termination_conditions

        sensors = []
        for sensor_dict in data.get("ego_vehicle", {}).get("sensors", []):
            if sensor_dict.get("type", "") == "camera":
                sensors.append(CameraInformation.from_dict(sensor_dict))
            else:
                logging.warning("Unbekannter Typ für Sensor")

        data["ego_vehicle"].update({"sensors": sensors})
        data["ego_vehicle"] = VehicleInformation.from_dict(data.get("ego_vehicle", {}))

        return cls(**data)
