"""
Main-Script, zum Starten der Tests und Evaluationen. In der Main-Methode werden Parameter des Simulators und die
zu verwendenden Evaluationskriterien, TestSpecifications und Agenten angegeben.
"""
from typing import List

from framework.evaluation.abstract_evaluation_criteria import EvaluationCriteria
from framework.evaluation.example_evaluation_criteria import LanePositioningError
from framework.lane_keeping_agent.dummy_e2e_controller import FixedSteerController, AlternatingSteerController
from framework.lane_keeping_agent.end_to_end_agent import EndToEndLaneKeepingAgent
from framework.start import TestExecutionTask, Start


def main() -> None:
    """Hauptmethode, wird beim Ausführen des Skripts aufgerufen"""
    # CARLA-Konfiguration
    # carla_host = "192.168.0.80"
    carla_host = "localhost"
    carla_port = 2000
    carla_timeout = 15.0
    carla_fps = 10  # min. 10 fps, je mehr desto öfter werden Kamerabilder und damit auch Lenkwinkel berechnet
    carla_spectator_follows = True  # wenn True, folgt die Ansicht im CARLA dem Fahrzeug
    calc_lane_center_deviation = True  # kann die Laufzeit erhöhen, ggf. notwendig für Evaluation
    calc_distance_to_lane_crossing = True  # kann die Laufzeit erhöhen, ggf. notwendig für Evaluation und für CrossingLaneMarking-TerminationCondition

    # Liste mit Tests
    test_tasks: List[TestExecutionTask] = []

    # Evaluationskriterien (für alle Tests gleich)
    evaluation_criteria: List[EvaluationCriteria] = [LanePositioningError()]

    # Agents, die getestet werden sollen
    straight_on_agent = EndToEndLaneKeepingAgent(name="dummy-agent",
                                                 e2e_control=FixedSteerController(log_input_parameters=False,
                                                                                  log_output_parameters=True,
                                                                                  fixed_steer=0.0))
    slight_left_agent = EndToEndLaneKeepingAgent(name="dummy-slight-left-agent",
                                                 e2e_control=FixedSteerController(log_input_parameters=False,
                                                                                  log_output_parameters=True,
                                                                                  fixed_steer=-0.01))
    moderate_left_agent = EndToEndLaneKeepingAgent(name="dummy-moderate-left-agent",
                                                   e2e_control=FixedSteerController(log_input_parameters=False,
                                                                                    log_output_parameters=True,
                                                                                    fixed_steer=-0.1))
    alternating_agent = EndToEndLaneKeepingAgent(name="dummy-alternate-agent",
                                                 e2e_control=AlternatingSteerController(log_input_parameters=True,
                                                                                        log_output_parameters=True,
                                                                                        abs_steer=0.1))
    # Test-Spezifikationen
    straight_on_specification = "test_specifications/fitlane-test_straight_on_scenario.json"
    curve_specification = "test_specifications/fitlane-test_curve_scenario.json"
    curve_specification_5_runs = "test_specifications/fitlane-test_curve_scenario_5_runs.json"
    opendrive_map_specification = "test_specifications/fitlane-test_opendrive_map_scenario.json"

    # Test 1: Szenario: geradeaus, Agent: geradeaus
    test_tasks.append(TestExecutionTask(name="test1_straight_straight",
                                        specification_filename=straight_on_specification,
                                        agent=straight_on_agent))

    # Test 2: Szenario: geradeaus, Agent: Autopilot
    test_tasks.append(TestExecutionTask(name="test2_straight_autopilot",
                                        specification_filename=straight_on_specification,
                                        agent=None))

    # Test 3: Szenario: geradeaus, Agent: lenkt leicht nach links
    test_tasks.append(TestExecutionTask(name="test3_straight_left",
                                        specification_filename=straight_on_specification,
                                        agent=slight_left_agent))

    # Test 4: Szenario: geradeaus, Agent: lenkt abwechselnd links und rechts
    test_tasks.append(TestExecutionTask(name="test4_straight_alternate",
                                        specification_filename=straight_on_specification,
                                        agent=alternating_agent))

    # Test 5: Szenario: Kurve, Agent: geradeaus
    test_tasks.append(TestExecutionTask(name="test5_curve_straight",
                                        specification_filename=curve_specification,
                                        agent=straight_on_agent))

    # Test 6: Szenario: Kurve, Agent: Autopilot
    test_tasks.append(TestExecutionTask(name="test6_curve_autopilot",
                                        specification_filename=curve_specification,
                                        agent=None))

    # Test 7: 5 SimulationRuns, Szenario: Kurve, Agent: lenkt leicht nach links
    test_tasks.append(TestExecutionTask(name="test7_curve_slight_left",
                                        specification_filename=curve_specification_5_runs,
                                        agent=slight_left_agent))

    # Test 8: Szenario: Kurve, Agent: lenkt mäßig nach links
    test_tasks.append(TestExecutionTask(name="test8_curve_moderate_left",
                                        specification_filename=curve_specification,
                                        agent=moderate_left_agent))

    # Test 9: OpenDrive-Map-Szenario, Autopilot
    test_tasks.append(TestExecutionTask(name="test9_opendrive_map_autopilot",
                                        specification_filename=opendrive_map_specification,
                                        agent=None))

    # Starte Tests
    Start().startup(test_tasks, evaluation_criteria, carla_host, carla_port, carla_timeout, carla_fps,
                    carla_spectator_follows, calc_lane_center_deviation, calc_distance_to_lane_crossing)


if __name__ == '__main__':
    main()
