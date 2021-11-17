from __future__ import annotations

from abc import abstractmethod, ABC
from dataclasses import dataclass, field
from typing import Tuple, List, TYPE_CHECKING, Optional

from framework.lane_keeping_agent.abstract_subtask import LaneKeepingOutput, LaneKeepingInput, LaneKeepingSubtask
from framework.lane_keeping_test.test_protocol import LaneKeepingTaskParameters
from framework.lane_keeping_test.test_specification import VehicleInformation

if TYPE_CHECKING:
    from framework.lane_keeping_agent.parameters import VehicleControl
    from framework.simulator.sensor_data import CameraImage, SensorData


@dataclass
class LaneKeepingAgentInput(LaneKeepingInput):
    """Zusammenfassung der Eingaben für einen LaneKeepingAgent"""
    current_speed: float
    main_camera_image: CameraImage
    additional_sensor_data: List[SensorData]


@dataclass
class LaneKeepingAgentOutput(LaneKeepingOutput):
    """Zusammenfassung der Eingaben für einen LaneKeepingAgent"""
    vehicle_control_command: Optional[VehicleControl]


@dataclass
class LaneKeepingAgent(ABC):
    """Das in einer TestExecution zu verwendende Lane-Keeping-System. Hat Zugriff auf Informationen zum Fahrzeug und
    zur Sensorik."""
    name: str
    subtasks: List[LaneKeepingSubtask] = field(default_factory=lambda: [])
    __vehicle_information: Optional[VehicleInformation] = None

    @property
    def vehicle_information(self):
        return self.__vehicle_information

    @vehicle_information.setter
    def vehicle_information(self, vehicle_information: VehicleInformation):
        self.__vehicle_information = vehicle_information
        for subtask in self.subtasks:
            subtask.vehicle_information = vehicle_information

    @abstractmethod
    def calc_steering(self, lane_keeping_agent_input: LaneKeepingAgentInput) \
            -> Tuple[LaneKeepingAgentOutput, List[LaneKeepingTaskParameters]]:
        """Erhält Sensordaten als Eingabe und liefert den Lenkwinkel. Kann und soll in mehrere Teilaufgaben aufgeteilt
        werden"""

    def log_parameters(self, subtask: LaneKeepingSubtask, input: LaneKeepingInput, output: LaneKeepingOutput) -> \
            Optional[LaneKeepingTaskParameters]:
        if subtask.log_input_parameters or subtask.log_output_parameters:
            subtask_parameters = LaneKeepingTaskParameters(type(subtask).__name__)
            if subtask.log_input_parameters:
                subtask_parameters.input = input
            if subtask.log_output_parameters:
                subtask_parameters.output = output
            return subtask_parameters
        return None

    def to_dict(self):
        return {"name": self.name,
                "subtasks": self.subtasks}
