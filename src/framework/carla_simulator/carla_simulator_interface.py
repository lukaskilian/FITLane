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

from typing import Optional, List, Tuple
import logging
from queue import Queue, Empty

import transforms3d
import numpy as np
import math
import time

import carla

from framework.lane_keeping_agent.parameters import VehicleControl
from framework.lane_keeping_test.test_protocol import DrivingDataEntry, Vector, Timestamp
from framework.lane_keeping_test.test_specification import AbstractMap, WeatherParameters, \
    Transform, Location, Rotation, SimulatorImportedMap, OpenDriveMap, VehicleInformation, CameraInformation, \
    TestSpecification, SensorInformation
from framework.simulator.sensor_data import SensorData, CameraImage
from framework.simulator.simulator_interface import SimulatorInterface
from framework.util import files


class CarlaSimulatorInterface(SimulatorInterface):
    """Schnittstelle für den CARLA-Simulator Version 0.9.9.4"""

    def __init__(self, host: str = "localhost", port: int = 2000, connection_timeout: float = 2.0, fps: int = 20,
                 queue_timeout: float = 2.0, spectator_follow: bool = True, calc_deviation_flag: bool = True,
                 calc_dlc_flag: bool = True):
        """CarlaSimulatorInterface-Konstruktor
        :param host: Adresse des CARLA-Servers
        :param port: Port des CARLA-Servers
        :param connection_timeout: Verbindungs-Timeout
        :param fps: Anzahl der Frames, die der Simulator pro Sekunde erzeugen soll
        :param queue_timeout: Timeout für das Warten auf die Sensor-Daten
        """
        # carla options
        self.__host = host
        self.__port = port
        self.__connection_timeout = connection_timeout
        self.__fps = fps
        self.__spectator_follow = spectator_follow
        self.__queue_timeout = queue_timeout
        self.__calc_deviation_flag = calc_deviation_flag
        self.__calc_dlc_flag = calc_dlc_flag
        # carla client
        self.__client: Optional[carla.Client] = None
        self.__world: Optional[carla.World] = None
        self.__original_world_settings: Optional[carla.WorldSettings] = None
        # carla actors: vehicles and sensors
        self.__actor_list: List[carla.Actor] = []
        self.__ego_vehicle: Optional[carla.Vehicle] = None
        # sensor data
        self.__sensor_data_queue: "Queue[Tuple[SensorData, int]]" = Queue()
        # test specification
        self.test_specification: Optional[TestSpecification] = None

    def initialize(self) -> None:
        """
        Verbindet sich mit dem CARLA-Server. muss vor allen TestExecutions aufgerufen werden
        :raise ConnectionError, falls die Verbindung fehlschlägt
        """
        # connect to CARLA
        try:
            self.__client = carla.Client(self.__host, self.__port)
            self.__client.set_timeout(self.__connection_timeout)
            self.__world = self.__client.get_world()
        except RuntimeError:
            raise ConnectionError()
        # activate synchronous mode
        self.__activate_synchronous_mode()

    def terminate(self) -> None:
        """Wird nach allen TestExecutions ausgeführt
        """
        # restore synchrony settings
        self.__world.apply_settings(self.__original_world_settings)

    def __activate_synchronous_mode(self):
        self.__original_world_settings = self.__world.get_settings()
        settings = self.__world.get_settings()
        settings.fixed_delta_seconds = 1.0 / self.__fps
        settings.synchronous_mode = True
        self.__world.apply_settings(settings)

    def __load_map(self, simulation_map: AbstractMap):
        if isinstance(simulation_map, SimulatorImportedMap):
            if self.__world.get_map().name != simulation_map.map_name:
                try:
                    self.__world = self.__client.load_world(simulation_map.map_name)
                except RuntimeError:
                    logging.debug("RuntimeError abgefangen. Erneuter Versuch in 5 Sekunden...")
                    time.sleep(5)
                    self.__world = self.__client.get_world()
                self.__activate_synchronous_mode()
                logging.info(f"Map {simulation_map.map_name} erfolgreich geladen.")
            else:
                logging.info(f"Map neu laden nicht erforderlich. Map {simulation_map.map_name} ist bereits geladen.")
        elif isinstance(simulation_map, OpenDriveMap):
            opendrive = files.read(simulation_map.xodr_filepath)
            self.__world = self.__client.generate_opendrive_world(opendrive)
            self.__activate_synchronous_mode()
            logging.info("OpenDrive-Map geladen")
        else:
            logging.warning("Map konnte nicht geladen werden. Verwende Standard-Map")

    def __load_weather_parameters(self, weather: WeatherParameters):
        weather_params = carla.WeatherParameters(cloudiness=weather.cloudiness, precipitation=weather.precipitation,
                                                 precipitation_deposits=weather.precipitation_deposits,
                                                 wind_intensity=weather.wind_intensity,
                                                 sun_azimuth_angle=weather.sun_azimuth_angle,
                                                 sun_altitude_angle=weather.sun_altitude_angle,
                                                 fog_density=weather.fog_density, fog_distance=weather.fog_distance,
                                                 wetness=weather.wetness, fog_falloff=weather.fog_falloff)
        self.__world.set_weather(weather_params)
        logging.info("Wetter-Parameter in CARLA angewendet")

    def start_test_execution(self, test_specification: TestSpecification):
        self.test_specification = test_specification
        self.__load_map(test_specification.map)
        self.__load_weather_parameters(test_specification.weather)

    def __spawn_ego_vehicle(self, ego_vehicle_info: VehicleInformation, starting_point: Transform) -> carla.Transform:
        blueprint_library = self.__world.get_blueprint_library()
        vehicle_bp = blueprint_library.find(ego_vehicle_info.name)
        spawn_point = carla.Transform(
            carla.Location(starting_point.location.x, starting_point.location.y, starting_point.location.z),
            carla.Rotation(starting_point.rotation.pitch, starting_point.rotation.yaw, starting_point.rotation.roll))
        self.__ego_vehicle = self.__world.spawn_actor(vehicle_bp, spawn_point)
        self.__actor_list.append(self.__ego_vehicle)
        return spawn_point

    def __handle_sensor_data(self, sensor_info: SensorInformation, carla_sensor_id: int, data: carla.SensorData):
        if isinstance(sensor_info, CameraInformation):
            # convert raw data to numpy array, reshape and eliminate 4th value (alpha value of rgba)
            image_array = np.array(data.raw_data).reshape((data.height, data.width, 4))[:, :, :3]
            sensor_data = CameraImage(sensor_id=sensor_info.sensor_id, img_array=image_array)
        else:
            logging.error("Konnte SensorData nicht zuordnen. Verwerfe SensorData.")
            return None

        self.__sensor_data_queue.put((sensor_data, data.frame))
        logging.debug(
            f"SensorData zur Queue hinzugefügt. Anzahl Einträge: {self.__sensor_data_queue.qsize()}. "
            f"Frame:{data.frame}, CARLA-Sensor-ID: {carla_sensor_id}")

    def __spawn_sensors_and_listen(self, ego_vehicle_info: VehicleInformation) -> None:
        blueprint_library = self.__world.get_blueprint_library()
        for sensor_info in ego_vehicle_info.sensors:
            sensor_bp = blueprint_library.find(sensor_info.name)
            if isinstance(sensor_info, CameraInformation):
                sensor_bp.set_attribute("image_size_x", str(sensor_info.image_size_x))
                sensor_bp.set_attribute("image_size_y", str(sensor_info.image_size_y))
                sensor_bp.set_attribute("fov", str(sensor_info.fov))
            mounting_point = sensor_info.mounting_point
            relative_transform = carla.Transform(
                carla.Location(mounting_point.location.x, mounting_point.location.y, mounting_point.location.z),
                carla.Rotation(mounting_point.rotation.pitch, mounting_point.rotation.yaw,
                               mounting_point.rotation.roll))
            carla_sensor = self.__world.spawn_actor(sensor_bp, relative_transform, attach_to=self.__ego_vehicle)
            self.__actor_list.append(carla_sensor)
            logging.info(f"Sensor zur CARLA-Simulation hinzugefügt. carla-id:{carla_sensor.id} (ab dem nächsten Tick)")
            carla_sensor.listen(lambda data, carla_id=carla_sensor.id, sensor_information=sensor_info:
                                self.__handle_sensor_data(sensor_information, carla_id, data))
            logging.info(f"Sensor-Listening aktiv für Sensor mit carla-id:{carla_sensor.id} (ab dem nächsten Tick)")

    def __set_spectator(self, transform: carla.Transform):
        spectator_location = transform.transform(carla.Location(x=-20, z=12))
        spectator_rotation = carla.Rotation(pitch=-30, yaw=transform.rotation.yaw)
        self.__world.get_spectator().set_transform(carla.Transform(spectator_location, spectator_rotation))

    def start_simulation_run(self) -> None:
        ego_vehicle_transform = self.__spawn_ego_vehicle(self.test_specification.ego_vehicle,
                                                         self.test_specification.starting_point)
        self.__set_spectator(ego_vehicle_transform)
        self.__simulator_tick()
        logging.info(f"Fahrzeug zur CARLA-Simulation hinzugefügt. carla-id:{self.__ego_vehicle.id}")

        # tick a few times for starting at ground
        for _ in range(5):
            self.__simulator_tick()

        # set start speed
        velocity_vector = self.__calc_velocity_vector(ego_vehicle_transform, self.test_specification.start_speed)
        self.__ego_vehicle.set_velocity(velocity_vector)

        # set autopilot setting; for performance reasons: don't call set_autopilot method if use_autopilot is not true
        if self.test_specification.use_autopilot:
            logging.info("Autopilot-Modus wird aktiviert...")
            self.__ego_vehicle.set_autopilot(True)

        self.__simulator_tick()
        logging.info("Speed und Autopilot-Setting gesetzt")

        self.__spawn_sensors_and_listen(self.test_specification.ego_vehicle)
        logging.info("Alle Sensoren hinzugefügt und Listening aktiviert (ab dem nächsten Tick)")

    def __apply_control(self, control: VehicleControl) -> None:
        vehicle_control = carla.VehicleControl(throttle=control.throttle, steer=control.steer, brake=control.brake)
        self.__ego_vehicle.apply_control(vehicle_control)
        logging.debug(f"VehicleControl in CARLA angewandt")

    def __calc_waypoint_and_relative_location(self, vehicle_transform: carla.Transform) \
            -> Tuple[carla.Waypoint, carla.Location]:
        """Berechnet den nächsten Waypoint und die relative Position des Fahrzeugs zu diesem. Der Waypoint liegt dabei
        nicht notwendigerweise auf derselben Spur, auf der die Simulation begonnen wurde"""
        waypoint = self.__world.get_map().get_waypoint(vehicle_transform.location)
        if not waypoint:
            return None, None
        relative_vehicle_location = self.global_to_local_location(waypoint.transform, vehicle_transform.location)
        return waypoint, relative_vehicle_location

    def __calc_lane_center_deviation(self, vehicle_transform: carla.Transform) \
            -> Tuple[Optional[carla.Transform], Optional[float], Optional[float], Optional[float]]:
        """Berechnet die Mitte der Fahrspur, die Breite der Fahrspur, die laterale Abweichung des Fahrzeugs von dieser
        und den Orientierungswinkel innerhalb der Fahrspur"""
        waypoint, relative_location = self.__calc_waypoint_and_relative_location(vehicle_transform)
        if not waypoint:
            return None, None, None, None

        # calc lateral deviation
        deviation_of_center = relative_location.y

        # calc yaw angle in Lane
        relative_rotation = self.global_to_local_rotation(waypoint.transform.rotation, vehicle_transform.rotation)
        orientation_angle = relative_rotation.yaw
        return waypoint.transform, waypoint.lane_width, deviation_of_center, orientation_angle

    def __calc_dlc(self, vehicle_transform: carla.Transform, vehicle_width: float) \
            -> Tuple[Optional[float], Optional[float]]:
        """Berechnet die Distanz zwischen Fahrzeugkante und Markierungslinie"""
        waypoint, relative_location = self.__calc_waypoint_and_relative_location(vehicle_transform)
        if not waypoint:
            return None, None

        relative_left_lane_marking_y = -(waypoint.lane_width / 2)
        relative_right_lane_marking_y = (waypoint.lane_width / 2)
        relative_left_vehicle_y = relative_location.y - (vehicle_width / 2)
        relative_right_vehicle_y = relative_location.y + (vehicle_width / 2)

        left_dlc = relative_left_vehicle_y - relative_left_lane_marking_y
        right_dlc = relative_right_lane_marking_y - relative_right_vehicle_y

        return left_dlc, right_dlc

    def __get_driving_data(self) -> Tuple[DrivingDataEntry, int]:
        """ Berechnet Fahrdaten.
        Hinweis: lateral_deviation, orientation_angle, left_dlc und right_dlc orientieren sich immer am aktuellen
        Fahrstreifen, nicht an dem, in dem das Fahrzeug gestartet ist. So können also Spurwechsel auftreten, sodass
        sich die Werte nahezu umdrehen.
        :return: Tupel aus DrivingDataEntry und Carla-Frame (zum Prüfen der Synchronisation)
        """
        world_snapshot: carla.WorldSnapshot = self.__world.get_snapshot()
        ego_vehicle_snapshot: carla.ActorSnapshot = world_snapshot.find(self.__ego_vehicle.id)

        logging.debug(f"Driving Data für Frame {world_snapshot.frame} wird berechnet...")

        # velocity, speed, acceleration, angular velocity
        carla_velocity = ego_vehicle_snapshot.get_velocity()
        velocity = Vector(carla_velocity.x, carla_velocity.y, carla_velocity.z)
        speed = self.__get_speed(carla_velocity)
        carla_acc = ego_vehicle_snapshot.get_acceleration()
        acceleration = Vector(carla_acc.x, carla_acc.y, carla_acc.z)
        carla_angular_velocity = ego_vehicle_snapshot.get_angular_velocity()
        angular_velocity = Vector(carla_angular_velocity.x, carla_angular_velocity.y, carla_angular_velocity.z)

        # position (transform) in map
        carla_vehicle_transform = ego_vehicle_snapshot.get_transform()
        vehicle_transform = Transform(
            Location(carla_vehicle_transform.location.x, carla_vehicle_transform.location.y,
                     carla_vehicle_transform.location.z),
            Rotation(carla_vehicle_transform.rotation.pitch, carla_vehicle_transform.rotation.yaw,
                     carla_vehicle_transform.rotation.roll))

        # lane center, lane width, deviation and yaw angle
        lane_center_transform, lane_width, lateral_deviation, orientation_angle = None, None, None, None
        if self.__calc_deviation_flag:
            logging.debug("Berechne Lane Center und Abweichung...")
            carla_lane_center_transform, lane_width, lateral_deviation, orientation_angle = \
                self.__calc_lane_center_deviation(carla_vehicle_transform)
            if carla_lane_center_transform:
                lane_center_transform = Transform(
                    Location(carla_lane_center_transform.location.x, carla_lane_center_transform.location.y,
                             carla_lane_center_transform.location.z),
                    Rotation(carla_lane_center_transform.rotation.pitch, carla_lane_center_transform.rotation.yaw,
                             carla_lane_center_transform.rotation.roll))

        # distance to lane crossing in front of vehicle
        dlc_front_left, dlc_front_right = None, None
        if self.__calc_dlc_flag and self.test_specification.ego_vehicle.length and self.test_specification.ego_vehicle.width:
            logging.debug("Berechne DLC...")
            carla_vehicle_front_relative_location = carla.Location(x=self.test_specification.ego_vehicle.length / 2)
            carla_vehicle_front_location = carla_vehicle_transform.transform(carla_vehicle_front_relative_location)
            carla_vehicle_front_transform = carla.Transform(carla_vehicle_front_location,
                                                            carla_vehicle_transform.rotation)
            dlc_front_left, dlc_front_right = self.__calc_dlc(carla_vehicle_front_transform,
                                                              self.test_specification.ego_vehicle.width)

        logging.debug(f"Driving Data für Frame {world_snapshot.frame} fertig berechnet.")

        # set spectator
        if self.__spectator_follow:
            self.__set_spectator(carla_vehicle_transform)

        return (DrivingDataEntry(timestamp=Timestamp.create_current_timestamp(),
                                 vehicle_speed=speed,
                                 vehicle_velocity=velocity,
                                 vehicle_acceleration=acceleration,
                                 vehicle_angular_velocity=angular_velocity,
                                 lateral_deviation_of_lane_center=lateral_deviation,
                                 orientation_angle_in_lane=orientation_angle,
                                 distance_to_lane_marking_front_left=dlc_front_left,
                                 distance_to_lane_marking_front_right=dlc_front_right,
                                 vehicle_transform=vehicle_transform,
                                 lane_center_transform=lane_center_transform,
                                 lane_width=lane_width),
                world_snapshot.frame)

    def __get_sensor_data(self, world_frame: int) -> List[SensorData]:
        sensor_data_list: List[SensorData] = []
        for _ in range(len(self.test_specification.ego_vehicle.sensors)):
            logging.debug(f"Versuche SensorData aus Queue zu entnehmen.")
            try:
                sensor_data, sensor_frame = self.__sensor_data_queue.get(timeout=self.__queue_timeout)
                logging.debug(f"SensorData (Typ: {type(sensor_data)}) aus Queue entnommen (Frame: {sensor_frame}. "
                              f"Anzahl Einträge: {self.__sensor_data_queue.qsize()}")
                if sensor_frame == world_frame:
                    sensor_data_list.append(sensor_data)
                else:
                    logging.error("Sensor- und Welt-Daten nicht synchronisiert. (SensorData verworfen)")
            except Empty:
                logging.error("Timeout: Queue war leer, es fehlt SensorData (nicht da oder zu spät).")
        return sensor_data_list

    def run_step(self, vehicle_control: Optional[VehicleControl] = None) -> Tuple[List[SensorData], DrivingDataEntry]:
        # apply vehicle control
        if vehicle_control is not None:
            self.__apply_control(vehicle_control)

        # tick simulator for new frame
        self.__simulator_tick()

        # get driving data
        driving_data_entry, world_frame = self.__get_driving_data()

        # get sensor data
        sensor_data_list = self.__get_sensor_data(world_frame)

        # return sensor data and driving data
        return sensor_data_list, driving_data_entry

    def stop_simulation_run(self) -> None:
        self.__client.apply_batch_sync([carla.command.DestroyActor(x) for x in self.__actor_list[::-1]])
        logging.info("Simulation in CARLA gestoppt. Akteure entfernt (beim nächsten Tick)")
        self.__simulator_tick()
        self.__actor_list = []
        self.__ego_vehicle = None
        self.__sensor_data_queue = Queue()

    def stop_test_execution(self):
        self.test_specification = None

    def __simulator_tick(self) -> int:
        frame = self.__world.tick()
        logging.debug(f"CARLA tick ausgeführt. Neuer Frame: {frame}")
        return frame

    @staticmethod
    def __calc_velocity_vector(vehicle_transform: carla.Transform, km_h: float) -> carla.Vector3D:
        return vehicle_transform.transform(carla.Vector3D(x=(km_h / 3.6))) - vehicle_transform.location

    @staticmethod
    def __get_speed(velocity: carla.Vector3D) -> float:
        m_s = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
        km_h = 3.6 * m_s
        return km_h

    @staticmethod
    def local_to_global_location(reference: carla.Transform, relative_location: carla.Location) -> carla.Location:
        return reference.transform(relative_location)

    @staticmethod
    def local_to_global_rotation(reference: carla.Rotation, relative_rotation: carla.Rotation) -> carla.Rotation:
        abs_pitch = (relative_rotation.pitch + reference.pitch) % 360
        abs_yaw = (relative_rotation.yaw + reference.yaw) % 360
        abs_roll = (relative_rotation.roll + reference.roll) % 360

        return carla.Rotation(abs_pitch, abs_yaw, abs_roll)

    @staticmethod
    def global_to_local_location(reference: carla.Transform, absolute_location: carla.Location) -> carla.Location:
        # angelehnt an https://github.com/carla-simulator/carla/issues/1120
        sub_loc = absolute_location - reference.location
        sub_loc_array = np.array([sub_loc.x, sub_loc.y, sub_loc.z])

        roll = math.radians(reference.rotation.roll)
        pitch = math.radians(reference.rotation.pitch)
        yaw = math.radians(reference.rotation.yaw)

        R = transforms3d.euler.euler2mat(roll, pitch, yaw).T
        rel_loc_array = np.dot(R, sub_loc_array)
        return carla.Location(x=rel_loc_array[0], y=rel_loc_array[1], z=rel_loc_array[2])

    @staticmethod
    def global_to_local_rotation(reference: carla.Rotation, absolute_rotation: carla.Rotation) -> carla.Rotation:
        rel_pitch = (absolute_rotation.pitch - reference.pitch) % 360
        rel_yaw = (absolute_rotation.yaw - reference.yaw) % 360
        rel_roll = (absolute_rotation.roll - reference.roll) % 360

        if rel_pitch > 180:
            rel_pitch -= 360
        if rel_yaw > 180:
            rel_yaw -= 360
        if rel_roll > 180:
            rel_roll -= 360

        return carla.Rotation(rel_pitch, rel_yaw, rel_roll)
