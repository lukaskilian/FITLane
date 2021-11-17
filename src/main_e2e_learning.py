"""
Main-Script, zum Starten der Tests und Evaluationen. In der Main-Methode werden Parameter des Simulators und die
zu verwendenden Evaluationskriterien, TestSpecifications und Agenten angegeben.
"""
from typing import List

from e2e_learning.carla_waypoint_agent import CarlaWaypointAgent
from e2e_learning.e2e_learning_agent import E2ELearningAgent
from framework.evaluation.abstract_evaluation_criteria import EvaluationCriteria
from framework.evaluation.example_evaluation_criteria import LanePositioningError
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
    carla_spectator_follows = True  # wenn True, folgt die Ansicht im CARLA dem Fahrzeug
    calc_lane_center_deviation = True  # kann die Laufzeit erhöhen, ggf. notwendig für Evaluation
    calc_distance_to_lane_crossing = True  # kann die Laufzeit erhöhen, ggf. notwendig für Evaluation und für CrossingLaneMarking-TerminationCondition

    # Liste mit Tests
    test_tasks: List[TestExecutionTask] = []

    # Evaluationskriterien (für alle Tests gleich)
    evaluation_criteria: List[EvaluationCriteria] = [LanePositioningError(penalty_region_width=0.5)]

    # Agenten
    carla_waypoint_agent = EndToEndLaneKeepingAgent(name="carla-waypoint-agent",
                                                    e2e_control=CarlaWaypointAgent(log_input_parameters=False,
                                                                                   log_output_parameters=False,
                                                                                   host=carla_host, port=carla_port,
                                                                                   connection_timeout=carla_timeout,
                                                                                   return_only_steer=True))
    e2e_agent = EndToEndLaneKeepingAgent(name="e2e-learning-agent",
                                         e2e_control=E2ELearningAgent(log_input_parameters=False,
                                                                      log_output_parameters=False,
                                                                      model_filename="e2e_learning/model.h5"))

    # Test-Spezifikationen
    straight_on_specification = "test_specifications/e2e-test_straight_on_scenario.json"
    curve_specification = "test_specifications/e2e-test_curve_scenario.json"
    recover_from_error_specification = "test_specifications/e2e-test_recover_from_error.json"
    full_track_specification = "test_specifications/e2e-test_full_track.json"
    unknown_env_specification = "test_specifications/e2e-test_unknown_environment.json"

    # Test 1: Szenario: geradeaus, Agent: CARLA-Waypoint
    test_tasks.append(TestExecutionTask(name="Test1",
                                        specification_filename=straight_on_specification,
                                        agent=carla_waypoint_agent))

    # Test 2: Szenario: geradeaus, Agent: E2E-Learning
    test_tasks.append(TestExecutionTask(name="Test2",
                                        specification_filename=straight_on_specification,
                                        agent=e2e_agent))

    # Test 3: Szenario: Kurve, Agent: CARLA-Waypoint
    test_tasks.append(TestExecutionTask(name="Test3",
                                        specification_filename=curve_specification,
                                        agent=carla_waypoint_agent))

    # Test 4: Szenario: Kurve, Agent: E2E-Learning
    test_tasks.append(TestExecutionTask(name="Test4",
                                        specification_filename=curve_specification,
                                        agent=e2e_agent))

    # Test 5: Szenario: ganz am Fahrstreifenrand (Fehlerzustand), Agent: CARLA-Waypoint
    test_tasks.append(TestExecutionTask(name="Test5",
                                        specification_filename=recover_from_error_specification,
                                        agent=carla_waypoint_agent))

    # Test 6: Szenario: ganz am Fahrstreifenrand (Fehlerzustand), Agent: E2E-Learning
    test_tasks.append(TestExecutionTask(name="Test6",
                                        specification_filename=recover_from_error_specification,
                                        agent=e2e_agent))

    # Test 7: Szenario: gesamte Runde (längerer Test), Agent: E2E-Learning
    test_tasks.append(TestExecutionTask(name="Test7",
                                        specification_filename=full_track_specification,
                                        agent=e2e_agent))

    # Test 8: Szenario: unbekannte Umgebung, Agent: E2E-Learning
    test_tasks.append(TestExecutionTask(name="Test8",
                                        specification_filename=unknown_env_specification,
                                        agent=e2e_agent))

    # Starte Tests
    Start().startup(test_tasks, evaluation_criteria, carla_host, carla_port, carla_timeout, carla_fps,
                    carla_spectator_follows, calc_lane_center_deviation, calc_distance_to_lane_crossing)


if __name__ == '__main__':
    main()
