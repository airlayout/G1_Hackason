"""Run the text-only GPU Planner on strictly validated saved Observations.

No scenario labels or images are read by this runner.
"""
import argparse
import json
import hashlib
from dataclasses import asdict
from pathlib import Path
from time import perf_counter
from adapters.robot_mock import MockRobot
from adapters.video_source import validate_timestamps
from brain.split_brain import Planner, parse_flat
from brain.split_prompt import PLANNER_SYSTEM
from brain.split_schemas import CurrentObservation
from brain.state import State
from config import Config
from run_video import write_report


def replay_planner(source, timestamps, config, report_path, engine=None):
    report = {'mode': 'saved-observation text-only planner', 'timestamps': validate_timestamps(timestamps),
              'config': {**asdict(config), 'cache_dir': str(config.cache_dir)},
              'planner_system_prompt': PLANNER_SYSTEM, 'status': 'running', 'frames': []}
    report['source_sha256'] = hashlib.sha256(json.dumps(source, sort_keys=True).encode()).hexdigest()
    report['note'] = 'Saved Perception only; perception latency zero means no new inference.'
    write_report(report_path, report)
    try:
        if engine is None:
            from brain.split_vlm import SplitQwenVLM
            engine = SplitQwenVLM(config)
        if hasattr(engine, 'load'):
            engine.load()
            report['model_load'] = {'status': 'pass', **engine.last_metrics}
        planner, state, robot = Planner(engine), State(), MockRobot()
        for timestamp in report['timestamps']:
            found = [f for f in source['frames'] if f['timestamp'] == timestamp]
            if len(found) != 1:
                raise ValueError('Missing/duplicate source Observation')
            p = found[0]
            observation = parse_flat(p['raw_response'], CurrentObservation)
            if p['fallback'] or p['observation'] != observation.model_dump(mode='json'):
                raise ValueError('Source raw/Observation mismatch')
            start = perf_counter()
            before = state.snapshot
            sstart = perf_counter(); after = state.update(observation); state_s = perf_counter()-sstart
            pstart = perf_counter(); result = planner.decide(observation, after); planner_s = perf_counter()-pstart
            total_s = perf_counter()-start
            robot.execute(result.decision)
            entry = {'timestamp': timestamp,
                     'perception': {key: p[key] for key in ('raw_response','observation','fallback','error','bare_json') if key in p},
                     'state_before': before.model_dump(mode='json'), 'state_after': after.model_dump(mode='json'),
                     'transition': after.transition.value,
                     'planner': result.to_dict(), 'mock_robot_output': robot.last_output,
                     'timing': {'perception_wall_s': 0.0, 'state_update_s': state_s,
                                'planner_wall_s': planner_s, 'total_brain_s': total_s}}
            entry['perception'].update(metrics={}, inference_executed=False)
            report['frames'].append(entry); write_report(report_path, report)
            print(json.dumps(entry), flush=True)
        report['status'] = 'pass' if all(not f['planner']['fallback'] for f in report['frames']) else 'failed'
    except Exception as exc:
        report.update(status='failed', error=f'{type(exc).__name__}: {exc}')
    finally:
        write_report(report_path, report)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--timestamps', default='8,10,12')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = replay_planner(json.loads(args.source.read_text(encoding='utf-8')), args.timestamps.split(','),
                            Config(max_image_side=384, max_pixels=384**2), args.output)
    return 0 if result['status']=='pass' else 1


if __name__ == '__main__':
    raise SystemExit(main())
