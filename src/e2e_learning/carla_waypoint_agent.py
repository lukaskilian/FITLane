# find carla module
import glob
import os
import sys

try:
    sys.path.append(glob.glob('./framework/carla_simulator/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import logging
import random
from typing import Optional

import carla

from carlasim.carla_python_api.carla.agents.navigation.local_planner import LocalPlanner, RoadOption, _retrieve_options
from framework.lane_keeping_agent.abstract_agent import LaneKeepingAgentInput, LaneKeepingAgentOutput
from framework.lane_keeping_agent.end_to_end_agent import EndToEndController
from framework.lane_keeping_agent.parameters import VehicleControl
from framework.lane_keeping_test.test_protocol import Timestamp


class LaneKeepingLocalPlanner(LocalPlanner):
    """Überschreibt den LocalPlanner, sodass dieser an Kreuzungen die Spur hält und nicht abbiegt."""

    def _compute_next_waypoints(self, k=1):
        """ Modified compute next waypoint method copied from https://github.com/carla-simulator/carla/blob/0.9.9.4/PythonAPI/carla/agents/navigation/local_planner.py (Copyright (c) 2018-2020 CVC, MIT license)
                Add new waypoints to the trajectory queue.

                :param k: how many waypoints to compute
                :return:
        """
        # check we do not overflow the queue
        available_entries = self._waypoints_queue.maxlen - len(self._waypoints_queue)
        k = min(available_entries, k)

        for _ in range(k):
            last_waypoint = self._waypoints_queue[-1][0]
            next_waypoints = list(last_waypoint.next(self._sampling_radius))

            if len(next_waypoints) == 0:
                break
            elif len(next_waypoints) == 1:
                # only one option available ==> lanefollowing
                next_waypoint = next_waypoints[0]
                road_option = RoadOption.LANEFOLLOW
            else:
                # random choice between the possible options
                road_options_list = _retrieve_options(
                    next_waypoints, last_waypoint)
                # hier wird statt zufällig zu wählen, immer die zweite Variante gewählt
                road_option = RoadOption.STRAIGHT
                next_waypoint = next_waypoints[1]
            self._waypoints_queue.append((next_waypoint, road_option))

    def set_target_speed_to_limit(self):
        speed_limit = self._vehicle.get_speed_limit()
        self.set_speed(speed_limit)

    def __del__(self):
        pass


class CarlaWaypointAgent(EndToEndController):

    def __init__(self, log_input_parameters: bool, log_output_parameters: bool, host: str, port: int,
                 connection_timeout: float, random_shift_p: float = 0.0, return_only_steer: bool = False) -> None:
        super().__init__(log_input_parameters, log_output_parameters)
        self.__host = host
        self.__port = port
        self.__connection_timeout = connection_timeout
        self.random_shift_p = random_shift_p
        self.return_only_steer = return_only_steer
        self.local_planner: Optional[LaneKeepingLocalPlanner] = None
        self.__world = None
        self.__vehicle = None
        random.seed()  # Zahl angeben für Reproduzierbarkeit

    def connect_to_carla(self):
        client = carla.Client(self.__host, self.__port)
        client.set_timeout(self.__connection_timeout)
        self.__world = client.get_world()
        logging.info(f"CarlaWaypointAgent mit CARLA-Simulator verbunden")

    def initialize(self):
        # Hinweis: Wenn das Fahrzeug in der Simulation gewechselt wird, muss der Local Planner neu initialisiert werden
        actor_list = self.__world.get_actors().filter(self.vehicle_information.name)
        if actor_list:
            carla_vehicle = actor_list[0]
            self.local_planner = LaneKeepingLocalPlanner(carla_vehicle)
            self.__vehicle = carla_vehicle
            logging.info(f"CarlaWaypointAgent initialisiert mit Fahrzeug {carla_vehicle.id}")
        else:
            logging.warning("Initialisierung fehlgeschlagen")

    def calc_steering_command(self, input: LaneKeepingAgentInput) -> LaneKeepingAgentOutput:
        # erster Simulationsschritt: Verbinde, liefere kein Ergebnis
        if not self.__world:
            self.connect_to_carla()
            return LaneKeepingAgentOutput(success=False, timestamp=Timestamp.create_current_timestamp(),
                                          vehicle_control_command=None)

        # prüfen, ob sich das Fahrzeug geändert hat
        if self.__vehicle:
            vehicle_is_alive = self.__world.get_actors().find(self.__vehicle.id)
        else:
            vehicle_is_alive = False

        # zweiter Simulationsschritt: initialisiere (erst möglich, nachdem ein ein tick ausgeführt wurde)
        # erneutes Initialisieren bei Wechsel des Fahrzeugs
        if not self.local_planner or not vehicle_is_alive:
            self.initialize()
            if not self.local_planner:
                return LaneKeepingAgentOutput(success=False, timestamp=Timestamp.create_current_timestamp(),
                                              vehicle_control_command=None)

        self.local_planner.set_target_speed_to_limit()
        carla_control = self.local_planner.run_step()

        if self.return_only_steer:
            control = VehicleControl(steer=round(carla_control.steer, 6))
        else:
            control = VehicleControl(throttle=carla_control.throttle, steer=round(carla_control.steer, 6),
                                     brake=carla_control.brake)
        success = True
        logging.debug(f"CarlaWaypointAgent hat einen Lenkbefehl berechnet: {control}")

        # Mit einer bestimmten Wahrscheinlichkeit Fahrzeug verschieben
        if self.random_shift_p > 0 and random.random() < self.random_shift_p:
            deviation = random.uniform(0.3, 1.3) * random.choice([-1, 1])
            old_transform = self.__vehicle.get_transform()
            new_transform = carla.Transform(old_transform.transform(carla.Location(y=deviation)),
                                            old_transform.rotation)
            self.__vehicle.set_transform(new_transform)
            control = None
            success = False

        return LaneKeepingAgentOutput(success=success, timestamp=Timestamp.create_current_timestamp(),
                                      vehicle_control_command=control)
