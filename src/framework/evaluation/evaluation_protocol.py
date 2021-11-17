from __future__ import annotations

import statistics
from dataclasses import dataclass, field
from typing import List, Optional

from framework.evaluation.file_saver import FileSaver


@dataclass
class EvaluationDataEntry:
    """Für jeden Snapshot kann mithilfe eines EvaluationCriteria ein Wert bestimmt werden, der die Performance eines
    LaneKeepingAgent oder einer LaneKeepingSubtask in diesem Snapshot bestimmt."""
    snapshot_id: int
    value: float


@dataclass
class TotalResult:
    """Gesamtergebnis, dass die Performance des LaneKeepingAgents oder eines LaneKeepingSubtasks in einem SimulationRun
    oder einer TestExecution beschreibt; fasst somit die Werte mehrerer EvaluationDataEntries zusammen.	"""
    custom_value: Optional[float] = None
    average_evaluation_value: Optional[float] = None
    max_evaluation_value: Optional[float] = None
    min_evaluation_value: Optional[float] = None

    def calc_for_simulation_run(self, evaluation_entry_values: List[float]):
        self.max_evaluation_value = max(evaluation_entry_values)
        self.min_evaluation_value = min(evaluation_entry_values)
        self.average_evaluation_value = statistics.mean(evaluation_entry_values)

    def calc_for_test_execution(self, total_results: List[TotalResult]):
        self.max_evaluation_value = max([result.max_evaluation_value for result in total_results])
        self.min_evaluation_value = min([result.min_evaluation_value for result in total_results])
        self.average_evaluation_value = statistics.mean(
            [result.average_evaluation_value for result in total_results])


@dataclass
class EvaluationRunProtocol:
    """Für jeden SimulationRun kann ein EvaluationRunProtocol errechnet werden. Enthält neben den einzelnen
    EvaluationDataEntries (je eins für jeden Snapshot) auch ein Ergebnis (TotalResult) des gesamten SimulationRun"""
    simulation_run_number: int
    entries: List[EvaluationDataEntry]
    total_result: TotalResult


@dataclass
class EvaluationProtocol:
    """Für jede TestExecution kann für jedes Kriterium ein EvaluationProtocol errechnet werden. Enthält neben einem
    EvaluationRunProtocol für jeden SimulationRun auch ein Ergebnis (TotalResult) der gesamten TestExecution."""
    __test_execution_id: str
    criteria_name: str
    evaluation_run_protocols: List[EvaluationRunProtocol] = field(default_factory=lambda: [])
    total_result: Optional[TotalResult] = None

    @property
    def test_execution_id(self) -> str:
        return self.__test_execution_id

    def save_to_disk(self, file_saver: FileSaver) -> None:
        """speichert das EvaluationProtocol in einer Datei ab"""
        to_save_dict = {"criteria_name": self.criteria_name,
                        "evaluation_run_protocols": self.evaluation_run_protocols,
                        "total_result": self.total_result}
        file_saver.write_to_file(to_save_dict, f"{self.__test_execution_id}_evaluation_{self.criteria_name}")
