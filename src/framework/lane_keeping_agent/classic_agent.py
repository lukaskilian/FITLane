import logging
from abc import abstractmethod
from dataclasses import dataclass
from typing import List, Tuple

from framework.lane_keeping_agent.abstract_agent import LaneKeepingAgent, LaneKeepingAgentInput, LaneKeepingAgentOutput
from framework.lane_keeping_agent.abstract_subtask import LaneKeepingSubtask, LaneKeepingOutput, LaneKeepingInput
from framework.lane_keeping_agent.parameters import LaneData
from framework.lane_keeping_agent.parameters import LateralAcceleration
from framework.lane_keeping_agent.parameters import VehicleControl
from framework.lane_keeping_test.test_protocol import LaneKeepingTaskParameters, Timestamp
from framework.simulator.sensor_data import CameraImage, SensorData


@dataclass
class LaneDetectionInput(LaneKeepingInput):
    """Klasse für die Eingaben des LaneDetectors: Kamerabilder, Sensordaten, ..."""
    current_speed: float
    main_camera_image: CameraImage
    additional_sensor_data: List[SensorData]


@dataclass
class LaneDetectionOutput(LaneKeepingOutput):
    """Klasse für die Ausgaben des LaneDetectors: Fahrstreifendaten, Ist-Querbeschleunigung"""
    lane_data: LaneData
    current_lat_acc: LateralAcceleration


class LaneDetector(LaneKeepingSubtask):
    """LaneKeeping-Teilaufgabe zur Fahrstreifenerkennung"""

    @abstractmethod
    def detect_lanes(self, input: LaneDetectionInput) -> LaneDetectionOutput:
        """Liefert Fahrstreifendaten auf Basis Kamerabildern und weiteren Sensordaten"""


@dataclass
class TrajectoryPlanningInput(LaneKeepingInput):
    """Klasse für die Eingaben des TrajectoryPlanners: Fahrstreifendaten, Ist-Querbeschleunigung"""
    lane_data: LaneData
    current_lat_acc: LateralAcceleration


@dataclass
class TrajectoryPlanningOutput(LaneKeepingOutput):
    """Klasse für die Ausgaben des TrajectoryPlanners: Soll-Querbeschleunigung"""
    target_lat_acc: LateralAcceleration


class TrajectoryPlanner(LaneKeepingSubtask):
    """LaneKeeping-Teilaufgabe zur Planung der Trajektorie"""

    @abstractmethod
    def plan_trajectory(self, input: TrajectoryPlanningInput) -> TrajectoryPlanningOutput:
        """Liefert Soll-Querbschleunigung auf Basis von Fahrstreifendaten"""


@dataclass
class LateralControlInput(LaneKeepingInput):
    """Klasse für die Eingaben des LateralControllers: Ist-Querbeschleunigung, Soll-Querbeschleunigung"""
    current_lat_acc: LateralAcceleration
    target_lat_acc: LateralAcceleration


@dataclass
class LateralControlOutput(LaneKeepingOutput):
    """Klasse für die Ausgaben des TrajectoryPlanners: VehicleControl (Lenkbefehle, ...)"""
    vehicle_control_command: VehicleControl


class LateralController(LaneKeepingSubtask):
    """LaneKeeping-Teilaufgabe zur Regelung"""

    @abstractmethod
    def calc_steering_command(self, input: LateralControlInput) -> LateralControlOutput:
        """Liefert Lenkbefehl auf Basis von Ist- und Soll-Querbeschleunigung"""


class ClassicLaneKeepingAgent(LaneKeepingAgent):
    """LaneKeepingAgent, der mehrere Teilaufgaben (LaneKeepingSubtask) zur Berechnung des Lenkwinkels nutzt:
    Fahrstreifenerkennung (LaneDetector), Trajektorienplanung (TrajectoryPlanner) und Querregler (LateralController)."""
    __lane_detector: LaneDetector
    __trajectory_planner: TrajectoryPlanner
    __lateral_controller: LateralController

    def __init__(self, name: str, lane_detector: LaneDetector,
                 trajectory_planner: TrajectoryPlanner, lateral_controller: LateralController):
        super().__init__(name)
        self.__lane_detector = lane_detector
        self.__trajectory_planner = trajectory_planner
        self.__lateral_controller = lateral_controller

        self.subtasks.append(self.__lane_detector)
        self.subtasks.append(self.__trajectory_planner)
        self.subtasks.append(self.__lateral_controller)

    def calc_steering(self, lane_keeping_agent_input: LaneKeepingAgentInput) \
            -> Tuple[LaneKeepingAgentOutput, List[LaneKeepingTaskParameters]]:
        """Aufteilung der Aufgaben auf die Teilaufgaben"""
        task_parameters_list: List[LaneKeepingTaskParameters] = []

        # Lane Detection
        logging.debug("Fahrstreifendaten werden ermittelt...")
        lane_detector_input = LaneDetectionInput(Timestamp.create_current_timestamp(),
                                                 lane_keeping_agent_input.current_speed,
                                                 lane_keeping_agent_input.main_camera_image,
                                                 lane_keeping_agent_input.additional_sensor_data)
        lane_detector_output = self.__lane_detector.detect_lanes(lane_detector_input)
        subtask_parameters = self.log_parameters(self.__lane_detector, lane_detector_input, lane_detector_output)
        if subtask_parameters is not None:
            task_parameters_list.append(subtask_parameters)

        if lane_detector_output.success:
            logging.debug("Fahrstreifendaten ermittelt")
        else:
            logging.debug("Fahrstreifenerkennung fehlgeschlagen.")
            return LaneKeepingAgentOutput(Timestamp.create_current_timestamp(), False, None), task_parameters_list

        # Trajectory Planning
        logging.debug("Trajektorie wird geplant...")
        trajectory_planner_input = TrajectoryPlanningInput(Timestamp.create_current_timestamp(),
                                                           lane_detector_output.lane_data,
                                                           lane_detector_output.current_lat_acc)
        trajectory_planner_output = self.__trajectory_planner.plan_trajectory(trajectory_planner_input)
        subtask_parameters = self.log_parameters(self.__trajectory_planner, trajectory_planner_input,
                                                 trajectory_planner_output)
        if subtask_parameters is not None:
            task_parameters_list.append(subtask_parameters)

        if trajectory_planner_output.success:
            logging.debug("Trajektorie geplant")
        else:
            logging.debug("Trajektorienplanung fehlgeschlagen.")
            return LaneKeepingAgentOutput(Timestamp.create_current_timestamp(), False, None), task_parameters_list

        # Lateral Controller
        logging.debug("Lenkbefehl wird berechnet...")
        controller_input = LateralControlInput(Timestamp.create_current_timestamp(),
                                               lane_detector_output.current_lat_acc,
                                               trajectory_planner_output.target_lat_acc)
        controller_output = self.__lateral_controller.calc_steering_command(controller_input)
        subtask_parameters = self.log_parameters(self.__lateral_controller, controller_input, controller_output)
        if subtask_parameters is not None:
            task_parameters_list.append(subtask_parameters)

        if controller_output.success:
            logging.debug("Lenkbefehl berechnet")
        else:
            logging.debug("Querregelung fehlgeschlagen.")
            return LaneKeepingAgentOutput(Timestamp.create_current_timestamp(), False, None), task_parameters_list

        # Return output and Protocol
        lane_keeping_agent_output = LaneKeepingAgentOutput(Timestamp.create_current_timestamp(), True,
                                                           controller_output.vehicle_control_command)
        return lane_keeping_agent_output, task_parameters_list
