from __future__ import annotations

import copy
import logging
import threading
from contextlib import AbstractContextManager
from queue import Queue, Empty
from typing import Optional, Tuple, List

from framework.evaluation.evaluation_manager import EvaluationManager
from framework.evaluation.graph_plotter import GraphPlotter
from framework.lane_keeping_agent.abstract_agent import LaneKeepingAgentInput, LaneKeepingAgent
from framework.lane_keeping_agent.parameters import VehicleControl
from framework.lane_keeping_test.test_protocol import TestExecutionProtocol, SimulationRunProtocol, Snapshot, \
    LaneKeepingTaskParameters, Timestamp
from framework.lane_keeping_test.test_specification import TestSpecification
from framework.simulator.sensor_data import CameraImage
from framework.simulator.simulator_interface import SimulatorInterface
from framework.user_interface.view import AbstractView
from framework.evaluation.file_saver import FileSaver


class SimulationRun(AbstractContextManager):
    """
    Ein SimulationRun ist ein Durchlauf der Simulation von einem definierten Startpunkt zu einem
    definierten Zielpunkt. Dieser kann vorzeitig beendet werden, falls eine Abbruchbedingung
    (TerminationCondition) erfüllt ist. Vor jedem SimulationRun wird die Simulationsumgebung zurückgesetzt.
    """
    LOG_AUTOPILOT_INPUT = False

    def __init__(self, run_number: int, simulator: SimulatorInterface, test_specification: TestSpecification,
                 lane_keeping_agent: LaneKeepingAgent, view: AbstractView):
        self.run_number = run_number
        self.simulator: SimulatorInterface = simulator
        self.test_specification: TestSpecification = test_specification
        self.lane_keeping_agent: Optional[LaneKeepingAgent] = lane_keeping_agent
        self.__interrupted = False
        self.view = view

    def __enter__(self):
        logging.info(f"SimulationRun {self.run_number} wird durchgeführt...")

        logging.info("SimulationRun wird in Simulationsumgebung gestartet...")
        self.simulator.start_simulation_run()
        logging.info("SimulationRun in Simulationsumgebung gestartet")

        logging.info("Abbruchbedingungen werden initialisiert...")
        self.__initialize_termination_conditions()
        logging.info("Abbruchbedingungen initialisiert")

        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        logging.info("SimulationRun wird in Simulationsumgebung beendet...")
        self.simulator.stop_simulation_run()
        logging.info("SimulationRun in Simulationsumgebung beendet.")

        if not exc_type:
            logging.info(f"SimulationRun {self.run_number} erfolgreich durchgeführt.")
        else:
            if exc_type is InterruptedError:
                logging.info(f"SimulationRun {self.run_number} manuell abgebrochen.")
            else:
                logging.info(f"SimulationRun {self.run_number} wegen eines Fehlers abgebrochen.")
        return False

    def __initialize_termination_conditions(self) -> None:
        for condition in self.test_specification.termination_conditions:
            condition.initialize()

    def __any_termination_condition_met(self, current_snapshot: Snapshot) -> bool:
        for condition in self.test_specification.termination_conditions:
            if condition.is_condition_met(current_snapshot):
                logging.info(f"{type(condition).__name__} wurde zu True ausgewertet. SimulationRun wird beendet.")
                return True
        return False

    def execute(self) -> SimulationRunProtocol:
        """Führt einen SimulationRun durch"""

        # create (empty) SimulationRunProtocol
        simulation_run_protocol = SimulationRunProtocol(self.run_number)

        current_vehicle_control = None
        any_termination_condition_met = False
        # while not any of the termination conditions is met: run steps
        while not any_termination_condition_met:
            # if simulation cancelled: return
            if self.__interrupted:
                raise InterruptedError()

            # run step, save current driving data and vehicle control
            current_snapshot, current_vehicle_control = self.__run_step(current_vehicle_control)

            # log snapshot
            simulation_run_protocol.snapshots.append(current_snapshot)
            logging.debug(f"Snapshot protokolliert: {current_snapshot.snapshot_id}")

            # show snapshot on view
            self.view.show_snapshot(current_snapshot)

            # test if any termination condition ist met
            any_termination_condition_met = self.__any_termination_condition_met(current_snapshot)

        # return simulation run protocol
        logging.info("SimulationRunProtocol erstellt")
        return simulation_run_protocol

    def __run_step(self, current_vehicle_control: Optional[VehicleControl]) \
            -> Tuple[Snapshot, Optional[VehicleControl]]:
        """Leitet Sensordaten an LaneKeepingAgent weiter und zurück und protokolliert Daten"""
        logging.debug("Run Step")

        # run step in simulator: process vehicle control and get sensor data and driving data
        sensor_data_list, driving_data = self.simulator.run_step(current_vehicle_control)
        logging.debug(f"Fahrdaten und Sensordaten vom Simulator erhalten:")

        # create snapshot with driving data
        snapshot = Snapshot(driving_data)
        logging.debug(f"Snapshot mit Fahrdaten erzeugt: snapshot_id={snapshot.snapshot_id}")

        # add snapshot id to sensor data
        for sensor_data in sensor_data_list:
            sensor_data.snapshot_id = snapshot.snapshot_id

        # split sensor data in camera image and additional sensor data
        camera_image: Optional[CameraImage] = None
        additional_sensor_data = []
        for sensor_data in sensor_data_list:
            if isinstance(sensor_data, CameraImage):
                camera_image = sensor_data
            else:
                additional_sensor_data.append(sensor_data)

        if camera_image is None:
            logging.warning("Kein Kamerabild vom Simulator erhalten.")
            return snapshot, None

        # pass sensor data to lane keeping agent and get vehicle control for next step
        agent_input = LaneKeepingAgentInput(Timestamp.create_current_timestamp(), driving_data.vehicle_speed, camera_image,
                                            additional_sensor_data)
        if self.lane_keeping_agent:
            logging.debug("nächster Lenkbefehl wird vom LaneKeepingAgent berechnet...")
            # calculate steering command by lane keeping agent
            agent_output, snapshot.lane_keeping_task_parameters = self.lane_keeping_agent.calc_steering(agent_input)
            # define next vehicle control as copy of agent output vehicle control
            next_vehicle_control = copy.copy(agent_output.vehicle_control_command)
            # replace throttle with fixed throttle if throttle is None
            if next_vehicle_control and next_vehicle_control.throttle is None:
                next_vehicle_control.throttle = self.test_specification.fixed_throttle
            if next_vehicle_control and next_vehicle_control.brake is None:
                next_vehicle_control.brake = 0.0
            logging.debug("nächsten Lenkbefehl vom LaneKeepingAgent berechnet.")
        else:
            # handle autopilot mode
            if SimulationRun.LOG_AUTOPILOT_INPUT:
                snapshot.lane_keeping_task_parameters = \
                    [LaneKeepingTaskParameters(subtask_name="autopilot", input=agent_input)]
            next_vehicle_control = None
        logging.debug(f"LaneKeepingTaskParameters im Snapshot mit snapshot_id={snapshot.snapshot_id} protokolliert.")
        return snapshot, next_vehicle_control

    def interrupt(self):
        logging.info(f"Laufender SimulationRun {self.run_number} wird abgebrochen...")
        self.__interrupted = True


class TestExecution(AbstractContextManager):
    """ Die Ausführung eines Tests mit einer definierten Anzahl an SimulationRuns mit der gleichen Konfiguration
    (TestSpecification)."""

    def __init__(self, test_execution_id: str, test_specification: TestSpecification,
                 lane_keeping_agent: Optional[LaneKeepingAgent], simulator_interface: SimulatorInterface,
                 evaluation_manager: EvaluationManager, file_saver: FileSaver,
                 graph_plotters: List[GraphPlotter], view: AbstractView):
        self.__test_execution_id: str = test_execution_id
        self.test_specification: TestSpecification = test_specification
        self.lane_keeping_agent: Optional[LaneKeepingAgent] = lane_keeping_agent
        if not lane_keeping_agent:
            self.test_specification.use_autopilot = True
        self.test_execution_protocol: TestExecutionProtocol = TestExecutionProtocol(test_execution_id,
                                                                                    test_specification,
                                                                                    lane_keeping_agent)
        self.simulator: SimulatorInterface = simulator_interface
        self.evaluation_manager: EvaluationManager = evaluation_manager
        self.file_saver: FileSaver = file_saver
        self.graph_plotters: List[GraphPlotter] = graph_plotters
        self.view: AbstractView = view
        self.__current_simulation_run: Optional[SimulationRun] = None
        self.__lock = threading.Lock()
        self.__interrupted = False

    @property
    def test_execution_id(self) -> str:
        """Gibt die id der TestExecution zurück"""
        return self.__test_execution_id

    def __enter__(self):
        logging.info(f"TestExecution {self.test_execution_id} wird durchgeführt...")

        logging.info("Simulationsumgebung wird für TestExecution vorbereitet...")
        self.simulator.start_test_execution(self.test_specification)
        logging.info("Simulationsumgebung für TestExecution vorbereitet")
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        logging.info("TestExecution wird in Simulationsumgebung beendet...")
        self.simulator.stop_test_execution()
        logging.info("TestExecution in Simulationsumgebung beendet.")

        if not exc_type:
            logging.info(f"TestExecution {self.test_execution_id} erfolgreich durchgeführt.")
        else:
            if exc_type is InterruptedError:
                logging.info(f"TestExecution {self.test_execution_id} manuell abgebrochen.")
            else:
                logging.info(f"TestExecution {self.test_execution_id} wegen eines Fehlers abgebrochen.")
        return False

    def execute_test(self) -> None:
        """startet die Ausführung des Tests"""
        if self.lane_keeping_agent:
            self.lane_keeping_agent.vehicle_information = self.test_specification.ego_vehicle

        self.view.show_test_specification(self.test_specification, self.__test_execution_id)

        for run_number in range(self.test_specification.simulation_runs):
            if self.__interrupted:
                raise InterruptedError()

            self.__current_simulation_run = SimulationRun(run_number, self.simulator, self.test_specification,
                                                          self.lane_keeping_agent, self.view)
            with self.__current_simulation_run:
                run_protocol = self.__current_simulation_run.execute()
                self.test_execution_protocol.simulation_run_protocols.append(run_protocol)

        with self.__lock:
            self.__current_simulation_run = None

        # send protocol to evaluation manager
        self.evaluation_manager.add_test_execution_protocol(self.test_execution_protocol)
        logging.info("TestExecutionProtocol an EvaluationManager weitergeleitet")

        # plot graphs
        logging.info("Graphen zu TestExecutionProtocol werden geplottet...")
        for plotter in self.graph_plotters:
            plotter.plot(test_execution_protocol=self.test_execution_protocol)
        logging.info("Graphen zu TestExecutionProtocol geplottet")

        # save protocol to disk
        self.test_execution_protocol.save_to_disk(self.file_saver)
        logging.info("TestExecutionProtocol abgespeichert")

    def interrupt(self) -> None:
        """Unterbricht den laufenden SimulationRun und die TestExecution"""
        logging.info(f"Laufende TestExecution {self.test_execution_id} wird abgebrochen...")
        with self.__lock:
            if self.__current_simulation_run:
                self.__current_simulation_run.interrupt()
        self.__interrupted = True


class TestExecutionManager(AbstractContextManager):
    """Startet und Stoppt TestExecutions"""

    def __init__(self, simulator: SimulatorInterface):
        self.simulator: SimulatorInterface = simulator
        self.__test_execution_queue: "Queue[TestExecution]" = Queue()
        self.__current_test_execution: Optional[TestExecution] = None
        self.__interrupted = False
        self.__lock = threading.Lock()

    def __enter__(self):
        logging.info(f"TestExecutions werden durchgeführt...")

        logging.info("Simulationsumgebung wird initialisiert...")
        self.simulator.initialize()
        logging.info("Simulationsumgebung initialisiert")

        return self

    def __exit__(self, exc_type, exc_value, traceback):
        logging.info("Simulationsumgebung wird nach Ausführung der TestExecutions zurückgesetzt...")
        self.simulator.terminate()
        logging.info("Simulationsumgebung zurückgesetzt.")

        if not exc_type:
            logging.info(f"Alle TestExecutions wurden erfolgreich durchgeführt.")
        else:
            if exc_type is InterruptedError:
                logging.info(f"TestExecutions manuell abgebrochen.")
            else:
                logging.info(f"TestExecutions wegen eines Fehlers abgebrochen.")
        return False

    def add_test_execution(self, test: TestExecution) -> None:
        """fügt eine TestExecution zur Queue hinzu"""
        self.__test_execution_queue.put(test)

    def __execute_next_test(self):
        if self.__interrupted:
            raise InterruptedError()
        try:
            self.__current_test_execution = self.__test_execution_queue.get(timeout=1)
            with self.__current_test_execution:
                self.__current_test_execution.execute_test()
        except Empty:
            with self.__lock:
                self.__current_test_execution = None
            logging.info(f"Keine TestExecution in der Queue.")

    def execute_until_empty(self):
        """führt die Tests in der Queue aus, bis die Queue leer ist oder bis interrupt aufgerufen wird (dann wird
        InterruptedError geworfen)"""
        while not self.__test_execution_queue.empty():
            self.__execute_next_test()

        with self.__lock:
            self.__current_test_execution = None

    def execute_loop(self) -> None:
        """führt die Tests in der Queue aus, bis interrupt aufgerufen wird (dann wird InterruptedError geworfen)"""
        while True:
            self.__execute_next_test()

    def interrupt(self) -> None:
        """bricht das Ausführen der TestExecutions ab"""
        with self.__lock:
            if self.__current_test_execution:
                logging.info("Die Ausführung der TestExecutions wird beendet, "
                             "laufende Simulationen werden abgebrochen...")
                self.__current_test_execution.interrupt()
        self.__interrupted = True
