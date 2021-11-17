from abc import abstractmethod, ABC
from typing import List, Optional

from framework.evaluation.evaluation_protocol import EvaluationDataEntry
from framework.lane_keeping_test.test_protocol import Snapshot


class EvaluationCriteria(ABC):
    """Kriterium zur Evaluation"""

    def __init__(self, criteria_name: Optional[str] = None) -> None:
        if criteria_name is None:
            self.name = type(self).__name__
        else:
            self.name = criteria_name

    @abstractmethod
    def evaluate_snapshot(self, snapshot: Snapshot) -> EvaluationDataEntry:
        """Berechnet anhand von DrivingData und oder LaneKeepingData einen EvaluationDataEntry."""

    def calc_total_result(self, evaluation_entries: List[EvaluationDataEntry]) -> Optional[float]:
        """Berechnet einen Wert, der die EvaluationDataEntries in einer sinnvollen Weise zusammenfasst"""
        return None
