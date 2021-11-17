"""
Main-Script, zum Starten der Tests und Evaluationen. In der Main-Methode werden Parameter des Simulators und die
zu verwendenden Evaluationskriterien, TestSpecifications und Agenten angegeben.
"""
from typing import List

from e2e_learning.carla_waypoint_agent import CarlaWaypointAgent
from framework.evaluation.abstract_evaluation_criteria import EvaluationCriteria
from framework.lane_keeping_agent.end_to_end_agent import EndToEndLaneKeepingAgent
from framework.start import TestExecutionTask, Start


def main() -> None:
    """Hauptmethode, wird beim Ausführen des Skripts aufgerufen"""
    # CARLA-Konfiguration
    # carla_host = "192.168.0.80"
    carla_host = "localhost"
    carla_port = 2000
    carla_timeout = 10.0
    carla_fps = 30  # min. 10 fps, je mehr desto öfter werden Kamerabilder und damit auch Lenkwinkel berechnet
    carla_spectator_follows = False  # wenn True, folgt die Ansicht im CARLA dem Fahrzeug
    calc_lane_center_deviation = False  # kann die Laufzeit erhöhen, ggf. notwendig für Evaluation
    calc_distance_to_lane_crossing = False  # kann die Laufzeit erhöhen, ggf. notwendig für Evaluation und für CrossingLaneMarking-TerminationCondition

    # Liste mit Tests
    test_tasks: List[TestExecutionTask] = []

    # Evaluationskriterien (für alle Tests gleich)
    evaluation_criteria: List[EvaluationCriteria] = []

    # Agenten
    carla_waypoint_agent = EndToEndLaneKeepingAgent(name="carla-waypoint-agent",
                                                    e2e_control=CarlaWaypointAgent(log_input_parameters=True,
                                                                                   log_output_parameters=True,
                                                                                   host=carla_host, port=carla_port,
                                                                                   connection_timeout=carla_timeout,
                                                                                   random_shift_p=1 / 500,
                                                                                   return_only_steer=False))

    # Test-Spezifikationen
    test_specs = [f"test_specifications/data_generation{i}.json" for i in range(6)]

    # Tests: Trainingsdaten generieren
    # Richtung A, Spur mitte links
    test_tasks.append(TestExecutionTask(name="generate-training-data-0",
                                        specification_filename=test_specs[0],
                                        agent=carla_waypoint_agent))
    # Richtung A, Spur mitte rechts
    test_tasks.append(TestExecutionTask(name="generate-training-data-1",
                                        specification_filename=test_specs[1],
                                        agent=carla_waypoint_agent))
    # Richtung B, Spur mitte links
    test_tasks.append(TestExecutionTask(name="generate-training-data-2",
                                        specification_filename=test_specs[2],
                                        agent=carla_waypoint_agent))
    # Richtung B, Spur mitte rechts
    test_tasks.append(TestExecutionTask(name="generate-training-data-3",
                                        specification_filename=test_specs[3],
                                        agent=carla_waypoint_agent))

    # äußerer Ring
    test_tasks.append(TestExecutionTask(name="generate-training-data-4",
                                        specification_filename=test_specs[4],
                                        agent=carla_waypoint_agent))
    # innerer Ring
    test_tasks.append(TestExecutionTask(name="generate-training-data-5",
                                        specification_filename=test_specs[5],
                                        agent=carla_waypoint_agent))

    # Starte Tests
    Start().startup(test_tasks, evaluation_criteria, carla_host, carla_port, carla_timeout, carla_fps,
                    carla_spectator_follows, calc_lane_center_deviation, calc_distance_to_lane_crossing)


if __name__ == '__main__':
    main()
