import itertools
import logging
import threading
from contextlib import AbstractContextManager
from queue import Queue, Empty
from typing import List, Optional

from framework.evaluation.abstract_evaluation_criteria import EvaluationCriteria
from framework.evaluation.evaluation_protocol import EvaluationProtocol, EvaluationRunProtocol, TotalResult, \
    EvaluationDataEntry
from framework.evaluation.file_saver import FileSaver
from framework.evaluation.graph_plotter import GraphPlotter
from framework.lane_keeping_test.test_protocol import SimulationRunProtocol, TestExecutionProtocol
from framework.user_interface.view import AbstractView


class Evaluation(AbstractContextManager):
    """Eine Evaluation nutzt ein TestExecutionProtocol um mithilfe eines EvaluationCriteria ein
    EvaluationProtocol zu erstellen, welches in einer Datei abgespeichert werden kann"""
    test_protocol: TestExecutionProtocol
    criteria: EvaluationCriteria
    evaluation_protocol: Optional[EvaluationProtocol] = None

    def __init__(self, test_protocol: TestExecutionProtocol, criteria: EvaluationCriteria, file_saver: FileSaver,
                 graph_plotters: List[GraphPlotter], view: AbstractView) -> None:
        self.test_protocol = test_protocol
        self.criteria = criteria
        self.file_saver: FileSaver = file_saver
        self.graph_plotters: List[GraphPlotter] = graph_plotters
        self.view = view
        self.evaluation_protocol = EvaluationProtocol(self.test_protocol.test_execution_id,
                                                      criteria_name=self.criteria.name)
        self.__interrupted = False

    def __enter__(self):
        logging.info(f"Evaluation zu {self.test_protocol.test_execution_id} mit {self.criteria.name} wird "
                     f"durchgeführt...")

    def __exit__(self, exc_type, exc_value, traceback):
        if not exc_type:
            logging.info(f"Evaluation zu {self.test_protocol.test_execution_id} mit {self.criteria.name} "
                         f"erfolgreich durchgeführt.")
        else:
            if exc_type is InterruptedError:
                logging.info(f"Evaluation zu {self.test_protocol.test_execution_id} mit {self.criteria.name} "
                             f"manuell abgebrochen.")
            else:
                logging.info(f"Evaluation zu {self.test_protocol.test_execution_id} mit {self.criteria.name} "
                             f"wegen eines Fehlers abgebrochen.")
        return False

    def execute(self) -> None:
        """führt eine Evaluation durch"""
        for run in self.test_protocol.simulation_run_protocols:
            if self.__interrupted:
                raise InterruptedError()

            if run.snapshots:
                self.evaluation_protocol.evaluation_run_protocols.append(self.__evaluate_simulation_run(run))

        if not self.evaluation_protocol.evaluation_run_protocols:
            logging.debug("Kein EvaluationProtocol vorhanden")
            return None

        # calculate total result
        logging.debug(f"Berechne Gesamtergebnis für TestExecution {self.test_protocol.test_execution_id}")
        total_result_value = self.criteria.calc_total_result(
            list(
                itertools.chain(*[protocol.entries for protocol in self.evaluation_protocol.evaluation_run_protocols])))
        self.evaluation_protocol.total_result = TotalResult(custom_value=total_result_value)
        self.evaluation_protocol.total_result.calc_for_test_execution(
            [run_protocol.total_result for run_protocol in self.evaluation_protocol.evaluation_run_protocols])

        # show total result on view
        self.view.show_total_result(self.evaluation_protocol.total_result,
                                    test_execution_id=self.test_protocol.test_execution_id)

        # plot graphs
        for plotter in self.graph_plotters:
            plotter.plot(test_execution_protocol=self.test_protocol, evaluation_protocol=self.evaluation_protocol)
        logging.info("Graphen zu EvaluationProtocol geplottet")

        # save protocol to disk
        self.evaluation_protocol.save_to_disk(self.file_saver)
        logging.debug("EvaluationProtocol abgespeichert")

    def interrupt(self):
        logging.info(f"Laufende Evaluation zu {self.test_protocol.test_execution_id} mit {self.criteria.name}"
                     f" wird abgebrochen...")
        self.__interrupted = True

    def __evaluate_simulation_run(self, simulation_run_protocol: SimulationRunProtocol) -> EvaluationRunProtocol:
        logging.debug(f"Berechne Evaluationswerte für SimulationRun {simulation_run_protocol.simulation_run_number}")
        evaluation_data_entries: List[EvaluationDataEntry] = []
        for snapshot in simulation_run_protocol.snapshots:
            evaluation_data_entry = self.criteria.evaluate_snapshot(snapshot)
            evaluation_data_entries.append(evaluation_data_entry)
            self.view.show_evaluation_data_entry(evaluation_data_entry)

        logging.debug(f"Berechne Gesamtergebnis für SimulationRun {simulation_run_protocol.simulation_run_number}")
        total_result_value = self.criteria.calc_total_result(evaluation_data_entries)
        total_result = TotalResult(custom_value=total_result_value)
        total_result.calc_for_simulation_run([entry.value for entry in evaluation_data_entries])

        self.view.show_total_result(total_result, simulation_run_number=simulation_run_protocol.simulation_run_number)
        return EvaluationRunProtocol(simulation_run_protocol.simulation_run_number, evaluation_data_entries,
                                     total_result)


class EvaluationManager(AbstractContextManager):
    """Führt Evaluationen durch"""

    def __init__(self, criteria: List[EvaluationCriteria], file_saver: FileSaver,
                 graph_plotters: List[GraphPlotter], view: AbstractView):
        self.__criteria: List[EvaluationCriteria] = criteria
        self.file_saver: FileSaver = file_saver
        self.graph_plotters: List[GraphPlotter] = graph_plotters
        self.view = view
        self.__evaluation_protocols: List[EvaluationProtocol] = []
        self.__evaluation_queue: "Queue[Evaluation]" = Queue()
        self.__current_evaluation: Optional[Evaluation] = None
        self.__lock = threading.Lock()
        self.__interrupted = False

    def __enter__(self):
        logging.info(f"Evaluationen werden durchgeführt...")
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if not exc_type:
            logging.info(f"Alle Evaluationen wurden erfolgreich durchgeführt.")
        else:
            if exc_type is InterruptedError:
                logging.info(f"Evaluationen manuell abgebrochen.")
            else:
                logging.info(f"Evaluationen wegen eines Fehlers abgebrochen.")
        return False

    def add_test_execution_protocol(self, test_protocol: TestExecutionProtocol) -> None:
        """fügt eine neue Evaluation zur Queue hinzu (für jedes Kriterium eine Evaluation)"""
        for criterion in self.__criteria:
            evaluation = Evaluation(test_protocol, criterion, self.file_saver, self.graph_plotters, self.view)
            self.__evaluation_queue.put(evaluation)

    def __execute_next_evaluation(self):
        if self.__interrupted:
            raise InterruptedError()
        try:
            self.__current_evaluation = self.__evaluation_queue.get(timeout=1)
            with self.__current_evaluation:
                self.__current_evaluation.execute()
        except Empty:
            with self.__lock:
                self.__current_evaluation = None
            logging.info(f"Keine Evaluation in der Queue.")

    def execute_until_empty(self):
        """führt die Evaluationen in der Queue aus, bis die Queue leer ist oder bis interrupt aufgerufen wird (dann
        wird InterruptedError geworfen)"""
        while not self.__evaluation_queue.empty():
            self.__execute_next_evaluation()

        with self.__lock:
            self.__current_evaluation = None

    def execute_loop(self) -> None:
        """führt die Tests in der Queue aus, bis interrupt aufgerufen wird (dann wird InterruptedError geworfen)"""
        while True:
            self.__execute_next_evaluation()

    def interrupt(self) -> None:
        """bricht das Ausführen der Evaluationen ab"""
        with self.__lock:
            if self.__current_evaluation:
                logging.info("Die Ausführung der Evaluationen wird beendet, "
                             "laufende Evaluationen werden abgebrochen...")
                self.__current_evaluation.interrupt()
        self.__interrupted = True
