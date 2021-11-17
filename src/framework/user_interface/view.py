from abc import ABC, abstractmethod
from typing import Optional

from framework.evaluation.evaluation_protocol import TotalResult, EvaluationDataEntry
from framework.lane_keeping_test.test_protocol import Snapshot
from framework.lane_keeping_test.test_specification import TestSpecification


class AbstractView(ABC):

    @abstractmethod
    def show_test_specification(self, test_specification: TestSpecification, test_execution_id: str):
        """Zeigt die TestSpezifikation auf einer Benutzeroberfläche an"""

    @abstractmethod
    def show_snapshot(self, snapshot: Snapshot) -> None:
        """Zeigt einen Snapshot auf einer Benutzeroberfläche an"""

    @abstractmethod
    def show_evaluation_data_entry(self, evaluation_data_entry: EvaluationDataEntry):
        """Zeigt einen EvaluationDataEntry auf einer Benutzeroberfläche an"""

    @abstractmethod
    def show_total_result(self, total_result: TotalResult, test_execution_id: Optional[str] = None,
                          simulation_run_number: Optional[int] = None):
        """Zeigt das Ergebnis einer Evaluation (TotalResult) auf einer Benutzeroberfläche an"""


class ConsoleView(AbstractView):
    def show_test_specification(self, test_specification: TestSpecification, test_execution_id: str):
        print(f"TestSpecification for TestExecution {test_execution_id}:")
        print(f"test_specification={test_specification}")

    def show_snapshot(self, snapshot: Snapshot) -> None:
        print(f"snapshot={snapshot}")

    def show_evaluation_data_entry(self, evaluation_data_entry: EvaluationDataEntry):
        print(f"evaluation_data_entry={evaluation_data_entry}")

    def show_total_result(self, total_result: TotalResult, test_execution_id: Optional[str] = None,
                          simulation_run_number: Optional[int] = None):
        if simulation_run_number:
            print(f"TotalResult for simulation_run_number={simulation_run_number}:")
        elif test_execution_id:
            print(f"TotalResult for test_execution_id={test_execution_id}:")
        print(f"total_result={total_result}")
