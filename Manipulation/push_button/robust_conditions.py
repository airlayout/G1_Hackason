"""Standard-library-only contract for the four verified button conditions."""
import math

BUTTON_CONDITION_NAMES = ('nominal','far_3mm','left_3mm','right_3mm')
BUTTON_CONDITION_COUNT = len(BUTTON_CONDITION_NAMES)
BUTTON_POSITION_SPAN_M = .003


def button_condition_positions(normal):
    """Panel-horizontal left is world-up cross the measured inward normal."""
    if len(normal)!=3 or not all(math.isfinite(v) for v in normal):
        raise ValueError('Invalid button normal')
    x,y,z = normal
    horizontal = math.hypot(x,y)
    if horizontal < 1e-9 or abs(math.sqrt(x*x+y*y+z*z)-1.) > .001:
        raise ValueError('Invalid button normal')
    left = (-y/horizontal,x/horizontal,0.)
    return ((0.,0.,0.),tuple(BUTTON_POSITION_SPAN_M*v for v in normal),
        tuple(BUTTON_POSITION_SPAN_M*v for v in left),tuple(-BUTTON_POSITION_SPAN_M*v for v in left))


def validate_button_scenario_settings(count,span,policy='standard'):
    if policy == 'collection':
        if count != 1:
            raise ValueError('収集モードは基準位置の1条件で検証します')
        return
    if policy != 'standard':
        raise ValueError('Unknown button policy')
    if count!=BUTTON_CONDITION_COUNT or not math.isclose(span,BUTTON_POSITION_SPAN_M,rel_tol=0.,abs_tol=1e-9):
        raise ValueError('ボタンは基準・奥3mm・左3mm・右3mmの4条件が必要です')


def validate_button_plan(plan, *, method='physical'):
    """Reject stale three-condition, incomplete, renamed or displaced plans before motion."""
    if method == 'kinematic':
        validate_kinematic_button_plan(plan)
        return
    if method != 'physical' or plan.get('validation_method', 'physical') != 'physical':
        raise ValueError('物理検証済みの経路ではありません')
    policy = plan.get('settings',{}).get('button_policy','standard')
    if policy not in ('standard','collection'):
        raise ValueError('Unknown button plan policy')
    if policy == 'collection' and plan.get('scene',{}).get('button_policy') != 'collection':
        raise ValueError('Button scene/plan policy mismatch')
    count = 1 if policy == 'collection' else BUTTON_CONDITION_COUNT
    names = ('nominal',) if policy == 'collection' else BUTTON_CONDITION_NAMES
    message = ('ボタン収集用の基準位置1条件の完走・合格が必要です' if policy == 'collection' else
        'ボタン用の基準・奥3mm・左3mm・右3mmの4条件合格が必要です')
    if not plan.get('accepted') or plan.get('accepted_conditions')!=count:
        raise ValueError(message)
    scene = plan.get('scene',{})
    if scene.get('task')!='button':
        raise ValueError(message)
    try:
        positions = button_condition_positions(scene.get('normal',()))[:count]
    except (TypeError,ValueError) as exc:
        raise ValueError(message) from exc
    conditions = plan.get('conditions',[])
    if len(conditions)!=count:
        raise ValueError(message)
    selected = [row for row in plan.get('candidates',[]) if row.get('index')==plan.get('trajectory',{}).get('candidate')]
    if len(selected)!=1 or selected[0].get('accepted') is not True:
        raise ValueError(message)
    if plan.get('settings',{}).get('button_feedback') == 'joint':
        metrics = plan.get('trajectory',{}).get('motion_metrics',{})
        if metrics.get('button_feedback',{}).get('version') != 1 or metrics.get('button_tracking_settings') != plan['settings']:
            raise ValueError('Button feedback settings were not verified')
        if any(t.get('button_feedback') != 'joint' for t in selected[0].get('trials',[])):
            raise ValueError('Button joint feedback rollout is required')
    trials = selected[0].get('trials',[])
    if len(trials)!=count:
        raise ValueError(message)
    if plan.get('trajectory',{}).get('motion_metrics',{}).get('lamp_return_plan'):
        checks = trials[0].get('lamp_return_trials',[])
        if len(checks) != 2 or any(c.get('completed') is not True or c.get('success') is not True
                                   or c.get('lamp_return_triggered') is not True for c in checks):
            raise ValueError('点灯後の帰還経路の物理検証が必要です')
        trajectory = plan['trajectory']
        if [c.get('trigger_index') for c in checks] != [trajectory.get('press_start'),trajectory.get('press_end')]:
            raise ValueError('点灯後の帰還経路の検証位置が一致しません')
    for name,position,condition,trial in zip(names,positions,conditions,trials):
        if (condition.get('name')!=name or trial.get('condition_name')!=name
                or trial.get('completed') is not True or trial.get('success') is not True):
            raise ValueError(message)
        for actual in (condition.get('position',()),trial.get('condition_position_m',())):
            if len(actual)!=3 or not all(math.isclose(a,b,rel_tol=0.,abs_tol=1e-9) for a,b in zip(actual,position)):
                raise ValueError(message)


def validate_kinematic_button_plan(plan):
    """Validate the distinct opt-in prototype contract; never claim physical validation."""
    message = '運動学の試作経路が未完了または不正です'
    settings, scene, trajectory = plan.get('settings', {}), plan.get('scene', {}), plan.get('trajectory', {})
    if (plan.get('schema') != 'g1-kinematic-button-plan-v1'
            or plan.get('validation_method') != 'kinematic'
            or plan.get('physical_simulation_performed') is not False
            or plan.get('kinematic_checked') is not True or plan.get('accepted') is not True
            or plan.get('completed_rollouts') != 0 or plan.get('accepted_conditions') != 0
            or settings.get('button_policy') != 'collection' or settings.get('button_feedback') != 'joint'
            or settings.get('scenarios') != 1 or scene.get('task') != 'button'
            or scene.get('button_policy') != 'collection'):
        raise ValueError(message)
    rows = [v for v in plan.get('candidates', []) if v.get('index') == trajectory.get('candidate')]
    if (len(rows) != 1 or rows[0].get('accepted') is not True
            or not isinstance(rows[0].get('min_clearance_m'), (int, float))
            or not isinstance(rows[0].get('min_joint_margin_rad'), (int, float))
            or not math.isfinite(rows[0]['min_clearance_m'])
            or not math.isfinite(rows[0]['min_joint_margin_rad'])
            or rows[0].get('min_clearance_m', -1) < .003
            or rows[0].get('min_joint_margin_rad', -1) < .025):
        raise ValueError(message)
    metrics = trajectory.get('motion_metrics', {})
    if (metrics.get('button_feedback', {}).get('version') != 1
            or metrics.get('button_tracking_settings') != settings):
        raise ValueError('Button feedback settings were not verified')
    q, tcp, phases = trajectory.get('q', []), trajectory.get('tcp', []), trajectory.get('phases', [])
    if (not q or len(q) != len(tcp) or len(q) != len(phases)
            or any(len(v) != 7 or not all(math.isfinite(x) for x in v) for v in q)
            or any(len(v) != 3 or not all(math.isfinite(x) for x in v) for v in tcp)
            or not 0 <= trajectory.get('press_start', -1) <= trajectory.get('press_end', -1) < len(q)
            or not trajectory.get('press_end', -1) <= trajectory.get('retract_end', -1) < len(q)):
        raise ValueError(message)
