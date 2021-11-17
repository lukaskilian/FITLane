from __future__ import annotations

from abc import ABC
from dataclasses import dataclass
from typing import Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from framework.lane_keeping_test.test_specification import VehicleInformation
    from framework.lane_keeping_test.test_protocol import Timestamp


class LaneKeepingSubtask(ABC):
    """Oberklasse für Teilaufgaben eines LaneKeepingAgent"""
    vehicle_information: Optional[VehicleInformation]
    log_input_parameters: bool = False
    log_output_parameters: bool = False

    def __init__(self, log_input_parameters: bool, log_output_parameters: bool) -> None:
        self.log_input_parameters = log_input_parameters
        self.log_output_parameters = log_output_parameters
        self.vehicle_information = None

    def to_dict(self):
        return {"name": type(self).__name__,
                "log_input_parameters": self.log_input_parameters,
                "log_output_parameters": self.log_output_parameters}


@dataclass
class LaneKeepingParameter(ABC):
    """Oberklasse für LaneKeepingInput und LaneKeepingOutput"""
    timestamp: Timestamp


class LaneKeepingInput(LaneKeepingParameter):
    """Oberklasse für die Eingaben einer LaneKeeping-Teilaufgabe"""


@dataclass
class LaneKeepingOutput(LaneKeepingParameter):
    """Oberklasse für die Ausgaben einer LaneKeeping-Teilaufgabe"""
    success: bool
