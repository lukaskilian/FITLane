import logging

import cv2 as cv
import numpy as np
import tensorflow as tf

from framework.lane_keeping_agent.abstract_agent import LaneKeepingAgentInput, LaneKeepingAgentOutput
from framework.lane_keeping_agent.end_to_end_agent import EndToEndController
from framework.lane_keeping_agent.parameters import VehicleControl
from framework.lane_keeping_test.test_protocol import Timestamp


class E2ELearningAgent(EndToEndController):

    def __init__(self, log_input_parameters: bool, log_output_parameters: bool, model_filename: str) -> None:
        super().__init__(log_input_parameters, log_output_parameters)
        self.model = self.load_model(model_filename)

    def load_model(self, model_filename: str):
        return tf.keras.models.load_model(model_filename)

    def calc_steering_command(self, input: LaneKeepingAgentInput) -> LaneKeepingAgentOutput:
        # get image from input
        image = input.main_camera_image.img_array

        # convert to grayscale, resize and normalize
        image = cv.cvtColor(image, cv.COLOR_RGB2GRAY)
        image = cv.resize(image, (200, 100))
        image = np.expand_dims(image, axis=2)
        image = image / 255

        # make prediction with neural network
        result = self.model.predict(np.array([image]))
        predicted_steer = round(float(result[0][0]), 6)
        control = VehicleControl(steer=predicted_steer)

        logging.debug(f"E2ELearningAgent hat einen Lenkbefehl berechnet: {control}")

        # return predicted steering command
        return LaneKeepingAgentOutput(success=True, timestamp=Timestamp.create_current_timestamp(),
                                      vehicle_control_command=control)
