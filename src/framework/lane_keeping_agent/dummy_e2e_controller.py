from framework.lane_keeping_agent.abstract_agent import LaneKeepingAgentInput, LaneKeepingAgentOutput
from framework.lane_keeping_agent.end_to_end_agent import EndToEndController
from framework.lane_keeping_agent.parameters import VehicleControl
from framework.lane_keeping_test.test_protocol import Timestamp


class FixedSteerController(EndToEndController):

    def __init__(self, log_input_parameters: bool, log_output_parameters: bool, fixed_steer: float = 0.0) -> None:
        super().__init__(log_input_parameters, log_output_parameters)
        self.fixed_steer = fixed_steer

    def calc_steering_command(self, input: LaneKeepingAgentInput) -> LaneKeepingAgentOutput:
        return LaneKeepingAgentOutput(success=True, timestamp=Timestamp.create_current_timestamp(),
                                      vehicle_control_command=VehicleControl(steer=self.fixed_steer))


class AlternatingSteerController(EndToEndController):

    def __init__(self, log_input_parameters: bool, log_output_parameters: bool, abs_steer: float = 0.1) -> None:
        super().__init__(log_input_parameters, log_output_parameters)
        self.abs_steer = abs_steer
        self.RATE = 2
        self.current = -1

    def calc_steering_command(self, input: LaneKeepingAgentInput) -> LaneKeepingAgentOutput:
        # alternate left and right steering
        if self.current < 0:
            # steer left
            steer = -self.abs_steer
            self.current += 1
        else:
            # steer right
            steer = self.abs_steer
            self.current += 1
            if self.current >= self.RATE:
                self.current = -self.RATE

        return LaneKeepingAgentOutput(success=True, timestamp=Timestamp.create_current_timestamp(),
                                      vehicle_control_command=VehicleControl(steer=steer))
