from typing import List, Optional

from framework.evaluation.abstract_evaluation_criteria import EvaluationCriteria
from framework.evaluation.evaluation_protocol import EvaluationDataEntry
from framework.lane_keeping_test.test_protocol import Snapshot


class LanePositioningError(EvaluationCriteria):
    """Berechnet den LanePositioningError (beschrieben in Innocenti et al.: Imitation Learning for Vision-based
        Lane Keeping Assistance (2017)"""

    def __init__(self, penalty_region_width: float = 0.4, beta_value: float = 0.1) -> None:
        super().__init__()
        self.penalty_region_width = penalty_region_width
        self.beta_value = beta_value

    def calc_lpe(self, d: float) -> float:
        b = self.beta_value
        w = self.penalty_region_width
        if d < 0:
            return 1.0
        elif d <= w:
            return (b * w) ** (d / w) - (b * d)
        else:
            return 0.0

    def evaluate_snapshot(self, snapshot: Snapshot) -> EvaluationDataEntry:
        left_dlc = snapshot.driving_data.distance_to_lane_marking_front_left
        right_dlc = snapshot.driving_data.distance_to_lane_marking_front_right

        if left_dlc is not None and right_dlc is not None:
            lane_positioning_error = self.calc_lpe(min(left_dlc, right_dlc))
        else:
            lane_positioning_error = -1
        return EvaluationDataEntry(snapshot_id=snapshot.snapshot_id, value=lane_positioning_error)

    def calc_total_result(self, evaluation_entries: List[EvaluationDataEntry]) -> Optional[float]:
        """berechnet hier den Anteil der Snapshots, an denen der LanePositioningError > 0 ist"""
        return len([entry for entry in evaluation_entries if entry.value > 0]) / len(evaluation_entries)
