from __future__ import annotations

import time
from dataclasses import dataclass
from dataclasses import field
from typing import List, Optional, TYPE_CHECKING, ClassVar

from framework.evaluation.file_saver import FileSaver
from framework.lane_keeping_agent.abstract_subtask import LaneKeepingInput, LaneKeepingOutput
from framework.lane_keeping_test.test_specification import TestSpecification, Transform

if TYPE_CHECKING:
    from framework.lane_keeping_agent.abstract_agent import LaneKeepingAgent


@dataclass
class Vector:
    """Hilfsklasse für einen 3D-Vektor"""
    x: float
    y: float
    z: float


@dataclass
class Timestamp:
    """Hilfsklasse, um einen exakten Zeitpunkt zu speichern"""
    unix_time: float

    @classmethod
    def create_current_timestamp(cls):
        return cls(unix_time=time.time())


@dataclass
class DrivingDataEntry:
    """Vom Simulator erhaltene Daten über den Zustand des Fahrzeugs in der Simulation, z. B. Ort, Geschwindigkeit und
    Abweichung im Fahrstreifen. Je Snapshot existiert ein Eintrag."""
    timestamp: Timestamp
    vehicle_speed: float
    vehicle_velocity: Vector
    vehicle_acceleration: Vector
    vehicle_angular_velocity: Vector
    lateral_deviation_of_lane_center: Optional[float]
    orientation_angle_in_lane: Optional[float]
    distance_to_lane_marking_front_left: Optional[float]
    distance_to_lane_marking_front_right: Optional[float]
    vehicle_transform: Transform
    lane_center_transform: Optional[Transform]
    lane_width: Optional[float]


@dataclass
class LaneKeepingTaskParameters:
    """Beinhaltet für eine Teilaufgabe (LaneKeepingSubtask) oder für den vollständigen LaneKeepingAgent die erhaltenen
    Eingaben (LaneKeepingInput) und die jeweils passenden berechneten Ausgaben (LaneKeepingOutput)."""
    subtask_name: str
    input: Optional[LaneKeepingInput] = None
    output: Optional[LaneKeepingOutput] = None


class Snapshot:
    """Eine Momentaufnahme in einem SimulationRun. Jeder Snapshot hat eine für den spezifischen SimulationRun eindeutige
     ID. Die Anzahl der Snapshots entspricht der Anzahl der vom Simulator gelieferten Kamerabilder. Der Snapshot
     beinhaltet einen DrivingDataEntry und für jede zu protokollierende Teilaufgabe LaneKeepingTaskParameters.
     Außerdem wird ein Zeitstempel protokolliert mit der Zeit des Systems, welches das Framework ausführt."""
    __snapshot_id: int
    driving_data: DrivingDataEntry
    lane_keeping_task_parameters: List[LaneKeepingTaskParameters]
    __next_id: ClassVar[int] = 0

    def __init__(self, driving_data: DrivingDataEntry):
        self.__snapshot_id = Snapshot.__next_id
        Snapshot.__next_id += 1
        self.driving_data = driving_data
        self.lane_keeping_task_parameters: List[LaneKeepingTaskParameters] = []

    @property
    def snapshot_id(self) -> int:
        return self.__snapshot_id

    def __str__(self) -> str:
        return f"Snapshot(" \
               f"snapshot_id={self.snapshot_id}," \
               f"driving_data={self.driving_data}," \
               f"lane_keeping_task_parameters={self.lane_keeping_task_parameters}" \
               f")"

    def __repr__(self) -> str:
        return str(self)


@dataclass
class SimulationRunProtocol:
    """Enthält Informationen über ein durchgeführten SimulationRun: Sowohl DrivingData, als auch LaneKeepingData.
    Kann in einer Datei abgespeichert werdens."""
    simulation_run_number: int
    snapshots: List[Snapshot] = field(default_factory=lambda: [])


@dataclass
class TestExecutionProtocol:
    """Enthält Informationen über eine durchgeführte TestExecution: neben der SimulationConfiguration auch für jeden
    SimulationRun ein SimulationRunProtocol."""
    __test_execution_id: str
    specification: TestSpecification
    lane_keeping_agent: Optional[LaneKeepingAgent]
    simulation_run_protocols: List[SimulationRunProtocol] = field(default_factory=lambda: [])

    @property
    def test_execution_id(self) -> str:
        return self.__test_execution_id

    def save_to_disk(self, file_saver: FileSaver) -> None:
        """speichert die Daten in einer Datei"""
        to_save = {"lane_keeping_agent": self.lane_keeping_agent,
                   "test_specification": self.specification}
        file_saver.write_to_file(to_save, f"{self.__test_execution_id}_specification")
        file_saver.write_to_file(self.simulation_run_protocols, f"{self.__test_execution_id}_protocols")
