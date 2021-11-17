from abc import ABC, abstractmethod
from typing import Optional, Tuple, List

from framework.lane_keeping_agent.parameters import VehicleControl
from framework.lane_keeping_test.test_protocol import DrivingDataEntry
from framework.lane_keeping_test.test_specification import TestSpecification
from framework.simulator.sensor_data import SensorData


class SimulatorInterface(ABC):
    """Schnittstelle zwischen dem Framework und dem ausführenden Simulator."""

    @abstractmethod
    def initialize(self) -> None:
        """Methode, die vor allen TestExecutions aufgerufen wird, um den Simulator zu initialisieren und sich mit diesem
         zu verbinden"""

    @abstractmethod
    def start_test_execution(self, test_specification: TestSpecification) -> None:
        """Bereitet die Simulationsumgebung für eine TestExecution vor"""

    @abstractmethod
    def start_simulation_run(self) -> None:
        """Bereitet die Simulationsumgebung für einen SimulationRun vor und startet diesen"""

    @abstractmethod
    def run_step(self, vehicle_control: Optional[VehicleControl] = None) -> Tuple[List[SensorData], DrivingDataEntry]:
        """
        Ausführen eines Befehls des LaneKeepingAgent im Simulator und Rückgabe der Sensor- und Fahrdaten.
        Kann solange blockieren, bis alle SensorDaten erhalten wurden.
        :param vehicle_control: Vom LaneKeepingAgent berechneter Befehl
        :return: Liste mit SensorData: aktuelle Sensordaten aus dem Simulator, DrivingDataEntry: aktuelle Umgebungsdaten
         des Simulators
        """

    @abstractmethod
    def stop_simulation_run(self) -> None:
        """Stoppen eines SimulationRuns und Reset der Simulation"""

    @abstractmethod
    def stop_test_execution(self) -> None:
        """Reset der Simulation nach einer TestExecution"""

    @abstractmethod
    def terminate(self) -> None:
        """Methode, die nach allen TestExecutions ausgeführt wird"""
