"""Post-run strict Phase 3.3 validation, including raw decisions and reuse evidence."""
import argparse
import json
from pathlib import Path
from adapters.robot_mock import MockRobot
from brain.split_brain import parse_flat
from brain.split_schemas import PlannerDecision
from evaluate_split_scenario import evaluate
from run_video import write_report


def evaluate_phase33(report, scenario):
    result = evaluate(report, scenario)
    errors = result['integrity_errors']
    models, processors = set(), set()
    policy_checks = []
    for frame in report.get('frames', []):
        stages = [frame['planner']]
        if report.get('mode') != 'saved-observation text-only planner':
            stages.append(frame['perception'])
        for stage in stages:
            if not stage.get('bare_json') or stage.get('fallback'):
                errors.append(f"Invalid/non-bare structured JSON at {frame['timestamp']}")
            metrics = stage.get('metrics', {})
            models.add(metrics.get('model_instance_id')); processors.add(metrics.get('processor_instance_id'))
            if metrics.get('model_load_count') != 1:
                errors.append('Model was not loaded exactly once')
        try:
            decision = parse_flat(frame['planner']['raw_response'], PlannerDecision)
            robot = MockRobot(); robot.execute(decision)
            if robot.last_output != frame.get('mock_robot_output'):
                errors.append(f"Mock output mismatch at {frame['timestamp']}")
            obs = frame['perception']['observation']; state = frame['state_after']
            if obs['person_visible']:
                direction = obs['person_direction']
                actions = ['LOOK']
                if direction == 'CENTER' and obs['person_distance'] in ['MID','FAR']:
                    actions = ['APPROACH']
                if state['transition'] == 'APPEARED': actions.append('FOUND')
                ok = (decision.action in actions and decision.direction == direction and
                      decision.target == 'PERSON' and decision.distance == obs['person_distance'])
            elif state['last_seen_person_direction'] != 'UNKNOWN' and (state['transition']=='DISAPPEARED' or 1<=state['frames_since_person_seen']<=3):
                ok = (decision.action=='SEARCH' and decision.direction==state['last_seen_person_direction']
                      and decision.target=='PERSON' and decision.distance=='UNKNOWN')
            else:
                obstacle = obs['blocking_obstacle']
                action = {'LEFT':'AVOID_RIGHT','RIGHT':'AVOID_LEFT'}.get(obstacle,'WAIT')
                direction = {'LEFT':'RIGHT','RIGHT':'LEFT'}.get(obstacle,'UNKNOWN')
                ok = (decision.action==action and decision.direction==direction and
                      decision.target==('NONE' if action=='WAIT' else 'OBSTACLE'))
            policy_checks.append({'timestamp':frame['timestamp'], 'pass':ok})
        except (ValueError, TypeError):
            errors.append(f"Invalid raw Planner at {frame['timestamp']}")
            policy_checks.append({'timestamp':frame['timestamp'], 'pass':False})
    result['planner_policy_checks'] = policy_checks
    result['planner_policy_pass'] = bool(policy_checks) and all(c['pass'] for c in policy_checks)
    result['model_reuse'] = (len(models) == len(processors) == 1 and None not in models and None not in processors)
    if not result['model_reuse']:
        errors.append('Model/processor instance reuse not proven')
    result['pass'] = not errors and result['planner_policy_pass'] and all(c['pass'] for c in result['checks'])
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('report', type=Path); parser.add_argument('scenario', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = evaluate_phase33(*(json.loads(p.read_text(encoding='utf-8')) for p in (args.report, args.scenario)))
    write_report(args.output, result); print(json.dumps(result, indent=2))
    return 0 if result['pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
