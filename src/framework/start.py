import json
import logging
import sys
import time
from dataclasses import dataclass
from importlib import reload
from json.decoder import JSONDecodeError
from threading import Thread
from typing import List, Optional

from framework.carla_simulator.carla_simulator_interface import CarlaSimulatorInterface
from framework.evaluation.abstract_evaluation_criteria import EvaluationCriteria
from framework.evaluation.evaluation_manager import EvaluationManager
from framework.evaluation.file_saver import JSONFileSaver
from framework.evaluation.graph_plotter import TrajectoryPlotter, DeviationDlcLpePlotter
from framework.lane_keeping_agent.abstract_agent import LaneKeepingAgent
from framework.lane_keeping_test.test_execution import TestExecutionManager, TestExecution
from framework.lane_keeping_test.test_specification import TestSpecification
from framework.user_interface.view import ConsoleView
from framework.util import files


@dataclass
class TestExecutionTask:
    """Hilfsklasse zum Speichern der Informationen, die zum Erzeugen einer TestExecution benötigt werden"""
    name: str
    specification_filename: str
    agent: Optional[LaneKeepingAgent]


class ExecutionThread(Thread):
    """Thread-Klasse, um TestExecutions und Evaluations nacheinander auszuführen"""

    def __init__(self, name: str, test_execution_manager: TestExecutionManager, evaluation_manager: EvaluationManager):
        super().__init__(name=name)
        self.test_execution_manager = test_execution_manager
        self.evaluation_manager = evaluation_manager
        self.finished = False

    def run(self) -> None:
        try:
            with self.test_execution_manager:
                self.test_execution_manager.execute_until_empty()
            with self.evaluation_manager:
                self.evaluation_manager.execute_until_empty()
        except InterruptedError:
            logging.info("Alles erfolgreich manuell abgebrochen.")
        except ConnectionError:
            logging.critical("Konnte keine Verbindung zum Simulator aufbauen")
        except Exception:
            logging.critical("Folgender Fehler ist aufgetreten:", exc_info=True)
        self.finished = True


class Start:
    """Klasse zum Laden der TestSpecifications und zum Erzeugen der Objektstruktur"""

    def __configure_logging(self):
        """Konfigurieren des Loggings"""
        reload(logging)
        logging.basicConfig(level=logging.DEBUG, format="%(levelname)s:%(name)s:%(threadName)s:%(message)s")
        file_handler = logging.FileHandler(filename="debug.log", mode="w", encoding="utf-8")
        file_handler.setLevel(logging.DEBUG)
        file_handler.setFormatter(logging.Formatter("%(levelname)s:%(name)s:%(threadName)s:%(message)s"))
        logging.getLogger().addHandler(file_handler)
        logging.info("Logging konfiguriert")

    def __load_test_specification(self, specification_filename: str) -> TestSpecification:
        try:
            json_string = files.read(specification_filename)
            specification_dict = json.loads(json_string)
            test_spec = TestSpecification.from_dict(specification_dict)
            return test_spec
        except IOError:
            logging.critical("Test-Spezifikation: Datei konnte nicht geladen werden", exc_info=True)
            sys.exit()
        except JSONDecodeError:
            logging.critical("Test-Spezifikation: Datei liegt nicht im JSON-Format vor")
            sys.exit()
        except (TypeError, KeyError):
            logging.critical("Test-Spezifikation: inkorrektes Format (Attribut fehlt oder ist zuviel)", exc_info=True)
            sys.exit()

    def __create_objects(self, evaluation_criteria: List[EvaluationCriteria], host: str, port: int,
                         connection_timeout: float, fps: int, carla_spectator_follow: bool, calc_deviation: bool,
                         calc_dlc: bool):
        """Methode zum Erzeugen der Objekte"""
        self.carla_sim = CarlaSimulatorInterface(host=host, port=port, connection_timeout=connection_timeout, fps=fps,
                                                 spectator_follow=carla_spectator_follow,
                                                 calc_deviation_flag=calc_deviation, calc_dlc_flag=calc_dlc)
        output_directory = f'../out/output_{time.strftime("%Y%m%d_%H%M%S")}'
        self.file_saver = JSONFileSaver(output_directory)
        self.test_execution_graph_plotters = [TrajectoryPlotter(output_directory)]
        self.evaluation_graph_plotters = [DeviationDlcLpePlotter(output_directory)]
        self.view = ConsoleView()
        self.test_ex_man = TestExecutionManager(self.carla_sim)
        self.evaluation_man = EvaluationManager(evaluation_criteria, self.file_saver, self.evaluation_graph_plotters,
                                                self.view)

    def __add_test_executions(self, test_tasks: List[TestExecutionTask]):
        for task in test_tasks:
            test_spec = self.__load_test_specification(task.specification_filename)
            logging.info(f"Testspezifikation '{task.specification_filename}' geladen")
            test_ex = TestExecution(test_execution_id=task.name,
                                    test_specification=test_spec,
                                    lane_keeping_agent=task.agent,
                                    simulator_interface=self.carla_sim,
                                    evaluation_manager=self.evaluation_man,
                                    file_saver=self.file_saver,
                                    graph_plotters=self.test_execution_graph_plotters,
                                    view=self.view)
            self.test_ex_man.add_test_execution(test_ex)
            logging.info(f"TestExecution '{task.name}' erstellt und hinzugefügt")

    def __start_execution(self):
        self.ex_thread = ExecutionThread("ExecutionThread", self.test_ex_man, self.evaluation_man)
        self.ex_thread.start()

    def __wait_until_finished(self):
        """Warten, bis der ExecutionThread fertig ist oder Strg+C gedrückt wurde"""
        while not self.ex_thread.finished:
            try:
                time.sleep(0.5)
            except KeyboardInterrupt:
                self.test_ex_man.interrupt()
                self.evaluation_man.interrupt()
        logging.info("Skript beendet")

    def startup(self, test_tasks: List[TestExecutionTask], evaluation_criteria: List[EvaluationCriteria], host: str,
                port: int, connection_timeout: float, fps: int, carla_spectator_follow: bool, calc_deviation: bool,
                calc_dlc: bool):
        """Methode zum Starten"""
        self.__configure_logging()
        self.__create_objects(evaluation_criteria, host, port, connection_timeout, fps, carla_spectator_follow,
                              calc_deviation, calc_dlc)
        self.__add_test_executions(test_tasks)
        self.__start_execution()
        self.__wait_until_finished()
