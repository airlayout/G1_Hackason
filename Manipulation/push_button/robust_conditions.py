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


def validate_button_scenario_settings(count,span):
    if count!=BUTTON_CONDITION_COUNT or not math.isclose(span,BUTTON_POSITION_SPAN_M,rel_tol=0.,abs_tol=1e-9):
        raise ValueError('ボタンは基準・奥3mm・左3mm・右3mmの4条件が必要です')


def validate_button_plan(plan):
    """Reject stale three-condition, incomplete, renamed or displaced plans before motion."""
    message = 'ボタン用の基準・奥3mm・左3mm・右3mmの4条件合格が必要です'
    if not plan.get('accepted') or plan.get('accepted_conditions')!=BUTTON_CONDITION_COUNT:
        raise ValueError(message)
    scene = plan.get('scene',{})
    if scene.get('task')!='button':
        raise ValueError(message)
    try:
        positions = button_condition_positions(scene.get('normal',()))
    except (TypeError,ValueError) as exc:
        raise ValueError(message) from exc
    conditions = plan.get('conditions',[])
    if len(conditions)!=BUTTON_CONDITION_COUNT:
        raise ValueError(message)
    selected = [row for row in plan.get('candidates',[]) if row.get('index')==plan.get('trajectory',{}).get('candidate')]
    if len(selected)!=1 or selected[0].get('accepted') is not True:
        raise ValueError(message)
    trials = selected[0].get('trials',[])
    if len(trials)!=BUTTON_CONDITION_COUNT:
        raise ValueError(message)
    for name,position,condition,trial in zip(BUTTON_CONDITION_NAMES,positions,conditions,trials):
        if (condition.get('name')!=name or trial.get('condition_name')!=name
                or trial.get('completed') is not True or trial.get('success') is not True):
            raise ValueError(message)
        for actual in (condition.get('position',()),trial.get('condition_position_m',())):
            if len(actual)!=3 or not all(math.isclose(a,b,rel_tol=0.,abs_tol=1e-9) for a,b in zip(actual,position)):
                raise ValueError(message)
