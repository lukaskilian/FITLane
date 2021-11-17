from __future__ import annotations

import os
from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Optional

import matplotlib
matplotlib.use('pdf')
import matplotlib.pyplot as plt

from framework.util import files

if TYPE_CHECKING:
    from framework.evaluation.evaluation_protocol import EvaluationProtocol
    from framework.lane_keeping_test.test_protocol import TestExecutionProtocol


class GraphPlotter(ABC):
    def __init__(self, directory_name: str):
        self._directory_name = directory_name

    def save_plot_to_disk(self, filename: str, figure):
        files.create_dir_if_not_exists(self._directory_name)
        filepath = os.path.join(self._directory_name, filename)
        figure.savefig(filepath)

    @abstractmethod
    def plot(self, test_execution_protocol: Optional[TestExecutionProtocol] = None,
             evaluation_protocol: Optional[EvaluationProtocol] = None) -> None:
        """Methode zum Plotten einiger Daten des TestExecutionProtocols und/oder des EvaluationProtocols"""


class TrajectoryPlotter(GraphPlotter):

    def plot(self, test_execution_protocol: Optional[TestExecutionProtocol] = None,
             evaluation_protocol: Optional[EvaluationProtocol] = None) -> None:
        if not test_execution_protocol:
            return None

        test_execution_id = test_execution_protocol.test_execution_id
        run_protocols = test_execution_protocol.simulation_run_protocols
        figure = plt.figure(tight_layout=True)

        # draw lane center locations
        if run_protocols and run_protocols[0].snapshots and \
                run_protocols[0].snapshots[0].driving_data.lane_center_transform:
            xs_center = [snapshot.driving_data.lane_center_transform.location.x
                         for snapshot in run_protocols[0].snapshots]
            # -1 * : change left handed to right handed coordinates
            ys_center = [-1 * snapshot.driving_data.lane_center_transform.location.y
                         for snapshot in run_protocols[0].snapshots]
            plt.plot(xs_center, ys_center, 'o', ls='-', label="Fahrstreifenmitte", markevery=[0, -1])

            # annotate start and end
            plt.annotate("start", xy=(run_protocols[0].snapshots[0].driving_data.lane_center_transform.location.x,
                                      -1 * run_protocols[0].snapshots[0].driving_data.lane_center_transform.location.y),
                         textcoords="offset pixels", xytext=(2, 2))
            plt.annotate("end", xy=(run_protocols[0].snapshots[-1].driving_data.lane_center_transform.location.x,
                                    -1 * run_protocols[0].snapshots[-1].driving_data.lane_center_transform.location.y),
                         textcoords="offset pixels", xytext=(2, 2))

        # draw vehicle locations for each simulation run
        for run_protocol in run_protocols:
            run_number = run_protocol.simulation_run_number
            snapshots = run_protocol.snapshots
            xs_vehicle = [snapshot.driving_data.vehicle_transform.location.x for snapshot in snapshots]
            # -1 * : change left handed to right handed coordinates
            ys_vehicle = [-1 * snapshot.driving_data.vehicle_transform.location.y for snapshot in snapshots]
            plt.plot(xs_vehicle, ys_vehicle, 'o', ls='-', label=f"SimulationRun {run_number}", markevery=[0, -1])

        plt.legend()
        plt.axis('equal')
        plt.suptitle(f"TestExecution {test_execution_id}")
        plt.title("gefahrene Trajektorien im Vergleich zur Fahrstreifenmitte")

        self.save_plot_to_disk(f"{test_execution_id}_graph_Trajektorie.pdf", figure)


class DeviationDlcLpePlotter(GraphPlotter):
    FIGURE_LENGTH = 6.4
    FIGURE_HEIGHT = 8.5

    def plot(self, test_execution_protocol: Optional[TestExecutionProtocol] = None,
             evaluation_protocol: Optional[EvaluationProtocol] = None) -> None:
        if test_execution_protocol and evaluation_protocol:
            self.__plot_as_one_figure(test_execution_protocol, evaluation_protocol)
            # self.__plot_as_multiple_figures(test_execution_protocol, evaluation_protocol)

    def __plot_as_one_figure(self, test_execution_protocol: TestExecutionProtocol,
                             evaluation_protocol: EvaluationProtocol):
        test_execution_id = test_execution_protocol.test_execution_id
        figure = plt.figure(figsize=(self.FIGURE_LENGTH, self.FIGURE_HEIGHT), tight_layout=True)
        figure.suptitle(f"TestExecution {test_execution_id}")

        ax1, ax2, ax3 = figure.subplots(3)
        self.plot_deviation(ax1, test_execution_protocol)
        self.plot_dlc(ax2, test_execution_protocol)
        self.plot_evaluation_protocol(ax3, evaluation_protocol)

        self.save_plot_to_disk(f"{test_execution_id}_graph_Abweichung_DLC_{evaluation_protocol.criteria_name}.pdf",
                               figure)

    def __plot_as_multiple_figures(self, test_execution_protocol: TestExecutionProtocol,
                                   evaluation_protocol: EvaluationProtocol):
        test_execution_id = test_execution_protocol.test_execution_id
        # deviation
        figure1, ax1 = plt.subplots()
        plt.suptitle(f"TestExecution {test_execution_id}")
        self.plot_deviation(ax1, test_execution_protocol)
        self.save_plot_to_disk(f"{test_execution_id}_graph_Abweichung.pdf", figure1)

        # dlc
        figure2, ax2 = plt.subplots()
        plt.suptitle(f"TestExecution {test_execution_id}")
        self.plot_dlc(ax2, test_execution_protocol)
        self.save_plot_to_disk(f"{test_execution_id}_graph_DLC.pdf", figure2)

        # evaluation values
        figure3, ax3 = plt.subplots()
        plt.suptitle(f"TestExecution {test_execution_id}")
        self.plot_evaluation_protocol(ax3, evaluation_protocol)
        self.save_plot_to_disk(f"{test_execution_id}_graph_{evaluation_protocol.criteria_name}.pdf", figure3)

    def plot_deviation(self, ax, test_execution_protocol: TestExecutionProtocol) -> None:
        ax.axhline(label="Fahrstreifenmitte", color=next(ax._get_lines.prop_cycler)['color'])

        for run_protocol in test_execution_protocol.simulation_run_protocols:
            run_number = run_protocol.simulation_run_number
            snapshots = run_protocol.snapshots
            if snapshots and snapshots[0].driving_data.lateral_deviation_of_lane_center:
                xs = list(range(len(snapshots)))
                # -1 * : change left handed to right handed coordinates
                ys = [-1 * snapshot.driving_data.lateral_deviation_of_lane_center for snapshot in snapshots]
                ax.plot(xs, ys, label=f"SimulationRun {run_number}")

        # y=0 in the center
        xmin, xmax, ymin, ymax = ax.axis()
        ymin = min(ymin, -0.25)
        if abs(ymin) < abs(ymax):
            ymin = -ymax
        else:
            ymax = -ymin
        ax.axis([xmin, xmax, ymin, ymax])

        ax.legend()
        ax.grid(True)
        ax.set_title("Laterale Abweichung im zeitlichen Verlauf")
        ax.set_xlabel("Simulationsschritt")
        ax.set_ylabel("Abweichung von der Fahrstreifenmitte \n"
                      "(in Metern)")

    def plot_dlc(self, ax, test_execution_protocol: TestExecutionProtocol) -> None:
        ax.axhline(label="Fahrstreifenmarkierung", color=next(ax._get_lines.prop_cycler)['color'])

        for run_protocol in test_execution_protocol.simulation_run_protocols:
            run_number = run_protocol.simulation_run_number
            snapshots = run_protocol.snapshots
            if snapshots and snapshots[0].driving_data.distance_to_lane_marking_front_left and \
                    snapshots[0].driving_data.distance_to_lane_marking_front_right:
                xs = list(range(len(snapshots)))
                ys = [min(snapshot.driving_data.distance_to_lane_marking_front_left,
                          snapshot.driving_data.distance_to_lane_marking_front_right) for snapshot in snapshots]
                ax.plot(xs, ys, label=f"SimulationRun {run_number}")

        ax.grid(True)
        ax.legend()
        ax.set_title("Distance-to-Lane-Crossing im zeitlichen Verlauf")
        ax.set_xlabel("Simulationsschritt")
        ax.set_ylabel("Distance to Lane Crossing \n"
                      "(vorne, in Metern)")

    def plot_evaluation_protocol(self, ax, evaluation_protocol: EvaluationProtocol) -> None:
        criteria_name = evaluation_protocol.criteria_name
        next(ax._get_lines.prop_cycler)['color']
        for run_protocol in evaluation_protocol.evaluation_run_protocols:
            run_number = run_protocol.simulation_run_number
            xs = list(range(len(run_protocol.entries)))
            ys = [entry.value for entry in run_protocol.entries]
            ax.plot(xs, ys, label=f"SimulationRun {run_number}")

        if criteria_name == "LanePositioningError":
            ax.axis(ymin=-0.05, ymax=1.05)

        ax.grid(True)
        ax.legend()
        ax.set_title(f"{criteria_name} im zeitlichen Verlauf")
        ax.set_xlabel("Simulationsschritt")
        ax.set_ylabel(criteria_name)
