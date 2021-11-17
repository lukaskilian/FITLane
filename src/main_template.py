"""
Main-Script, zum Starten der Tests und Evaluationen. In der Main-Methode werden Parameter des Simulators und die
zu verwendenden Evaluationskriterien, TestSpecifications und Agenten angegeben.
"""
from typing import List

from framework.evaluation.abstract_evaluation_criteria import EvaluationCriteria
from framework.evaluation.example_evaluation_criteria import LanePositioningError
from framework.lane_keeping_agent.dummy_e2e_controller import FixedSteerController
from framework.lane_keeping_agent.end_to_end_agent import EndToEndLaneKeepingAgent
from framework.start import TestExecutionTask, Start


def main() -> None:
    """Hauptmethode, wird beim Ausführen des Skripts aufgerufen"""
    # CARLA-Konfiguration
    carla_host = "localhost"
    carla_port = 2000
    carla_timeout = 10.0
    carla_fps = 10  # min. 10 fps, je mehr desto öfter werden Kamerabilder und damit auch Lenkwinkel berechnet
    carla_spectator_follows = True  # wenn True, folgt die Ansicht im CARLA dem Fahrzeug
    calc_lane_center_deviation = True  # kann die Laufzeit erhöhen, ggf. notwendig für Evaluation
    calc_distance_to_lane_crossing = True  # kann die Laufzeit erhöhen, ggf. notwendig für Evaluation und für CrossingLaneMarking-TerminationCondition

    # Liste mit Tests
    test_tasks: List[TestExecutionTask] = []

    # Evaluationskriterien (für alle Tests gleich)
    evaluation_criteria: List[EvaluationCriteria] = [LanePositioningError()]  # Hier die Evaluationskriterien eintragen

    # Agents, die getestet werden sollen
    agent = EndToEndLaneKeepingAgent(name="dummy-agent",  # Hier die Agenten mit seinen Teilaufgaben anlegen
                                     e2e_control=FixedSteerController(log_input_parameters=False,
                                                                      log_output_parameters=True,
                                                                      fixed_steer=0.0))

    # Test-Spezifikationen
    test_specification = "test_specifications/template_specification_straight_on.json"  # Hier den Pfad der Testspezifikation angeben

    # Tests
    test_tasks.append(TestExecutionTask(name="test",
                                        specification_filename=test_specification,
                                        agent=agent))
    # Hier Test-Tasks hinzufügen, die ausgeführt werden sollen

    # Starte Tests
    Start().startup(test_tasks, evaluation_criteria, carla_host, carla_port, carla_timeout, carla_fps,
                    carla_spectator_follows, calc_lane_center_deviation, calc_distance_to_lane_crossing)


if __name__ == '__main__':
    main()
