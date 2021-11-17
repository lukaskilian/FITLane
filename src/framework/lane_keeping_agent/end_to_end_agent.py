import logging
from abc import abstractmethod
from typing import List, Tuple

from framework.lane_keeping_agent.abstract_agent import LaneKeepingAgent
from framework.lane_keeping_agent.abstract_agent import LaneKeepingAgentInput, LaneKeepingAgentOutput
from framework.lane_keeping_agent.abstract_subtask import LaneKeepingSubtask
from framework.lane_keeping_test.test_protocol import LaneKeepingTaskParameters


class EndToEndController(LaneKeepingSubtask):
    """Teilaufgabe, die alle Teilaufgaben übernimmt"""

    @abstractmethod
    def calc_steering_command(self, input: LaneKeepingAgentInput) -> LaneKeepingAgentOutput:
        """Berechnet Lenkbefehl auf Basis von Kamerabildern und ggf. weiteren Sensordaten"""


class EndToEndLaneKeepingAgent(LaneKeepingAgent):
    """LaneKeepingAgent, der lediglich eine einzige Teilaufgabe (EndToEndController) zur Berechnung des Lenkwinkels
     nutzt."""

    def __init__(self, name: str, e2e_control: EndToEndController):
        super().__init__(name)
        self.__e2e_control = e2e_control
        self.subtasks.append(self.__e2e_control)

    def calc_steering(self, lane_keeping_agent_input: LaneKeepingAgentInput) \
            -> Tuple[LaneKeepingAgentOutput, List[LaneKeepingTaskParameters]]:
        """Weiterleitung an den EndToEndController"""
        task_parameters_list: List[LaneKeepingTaskParameters] = []

        logging.debug("Lenkbefehl wird berechnet...")
        lane_keeping_agent_output: LaneKeepingAgentOutput = self.__e2e_control.calc_steering_command(
            lane_keeping_agent_input)
        subtask_parameters = self.log_parameters(self.__e2e_control, lane_keeping_agent_input,
                                                 lane_keeping_agent_output)
        if subtask_parameters is not None:
            task_parameters_list.append(subtask_parameters)

        if lane_keeping_agent_output.success:
            logging.debug("Lenkbefehl berechnet.")
        else:
            logging.debug("Berechnung des Lenkbefehls fehlgeschlagen.")

        return lane_keeping_agent_output, task_parameters_list
