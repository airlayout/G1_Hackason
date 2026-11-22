from dataclasses import dataclass, asdict
from brain.split_prompt import PERCEPTION_SYSTEM, PERCEPTION_USER, PLANNER_SYSTEM
from .codecs import PLANNER_CLASS


COMPACT_FORMAT = ('Reply only visible(0/1)|direction(L/C/R/U)|distance(N/M/F/U)|blocking(L/C/R/0/U). '
                  'L=LEFT,C=CENTER,R=RIGHT,U=UNKNOWN; N=NEAR,M=MID,F=FAR; blocking 0=NONE. '
                  'Absent requires direction U and distance U. No prose or fences.')
COMPACT_PERCEPTION = (PERCEPTION_SYSTEM.split('Return ONLY')[0] +
                     'Blocking means obstacle blocking travel, not merely furniture. ' + COMPACT_FORMAT)
MINIMAL_PERCEPTION = ('Ignore furniture. Person body: L<40%,C40-60%,R>60%. '
                      'Emit visible|side|range|blocking:01,LCRU,NMFU,LCR0U. Absent:0|U|U|*.')
PLANNER_FORMAT = ('Reply only action|direction|target|distance. '
                  'Action: L LOOK,A APPROACH,X AVOID_LEFT,Y AVOID_RIGHT,S SEARCH,F FOUND,R SURPRISE,W WAIT. '
                  'Direction L/C/R/U = LEFT/CENTER/RIGHT/UNKNOWN. '
                  'Target P/B/T/O/0 = PERSON/BANANA/PLUSHIE/OBSTACLE/NONE. '
                  'Distance N/M/F/U = NEAR/MID/FAR/UNKNOWN. No prose or fences.')
COMPACT_PLANNER = PLANNER_SYSTEM.split('Return ONLY')[0] + PLANNER_FORMAT
MINIMAL_PLANNER = ('Current observation is truth; state is history. '
                   'Visible L/R:LOOK there; C MID/FAR:APPROACH C, else LOOK C. APPEARED allows FOUND. '
                   'Absent + known last side + (DISAPPEARED or missed 1..3):SEARCH last side,PERSON,UNKNOWN. '
                   'Otherwise blocking LEFT:AVOID_RIGHT,RIGHT:AVOID_LEFT; else WAIT UNKNOWN NONE UNKNOWN. '
                   'Visible actions target PERSON,use observed range; avoidance target OBSTACLE. ' + PLANNER_FORMAT)
CLASS_PERCEPTION = ('Inspect current image only. Real people including partial people, not furniture. '
                    'Body center: left <40%, center 40-60%, right >60%. '
                    'Reply exactly one code: A=no person, B=person LEFT, C=person CENTER, D=person RIGHT. '
                    'No whitespace, explanations or other tokens.')
EXACT_PERCEPTION = (PERCEPTION_SYSTEM.split('Return ONLY')[0] +
                   'Report travel-blocking obstacles, not merely furniture. '
                   'Output EXACTLY seven characters using this grammar: [01]|[LCRU]|[NMFU]|[LCR0U]. '
                   'Fields: person visible, body direction, qualitative distance, blocking obstacle. '
                   'L LEFT,C CENTER,R RIGHT,U UNKNOWN; N NEAR,M MID,F FAR; obstacle 0 NONE. '
                   'Absent person requires 0|U|U followed by obstacle. '
                   'Serialization examples: person RIGHT FAR with LEFT obstacle -> 1|R|F|L; '
                   'no person and no obstacle -> 0|U|U|0. Return codes only, no words.')
EXACT_PLANNER = (PLANNER_SYSTEM.split('Return ONLY')[0] + PLANNER_FORMAT +
                 'Output EXACTLY seven characters. Example serialization: '
                 'LOOK RIGHT PERSON FAR -> L|R|P|F; WAIT UNKNOWN NONE UNKNOWN -> W|U|0|U. '
                 'Do not spell out any field name or enum. Return the four code characters separated by pipes only.')


@dataclass(frozen=True)
class Profile:
    name: str
    format: str = 'json'
    edge: int = 384
    minimal: bool = False
    dtype: str = 'bf16'
    attention: str = 'default'
    instrument: bool = True
    exact: bool = False
    constrained: bool = False
    planner_json: bool = False
    planner_class: bool = False
    linear_patch: bool = False
    minified: bool = False

    def __post_init__(self):
        if self.format not in {'json','compact','class'} or self.dtype not in {'bf16','fp16'}:
            raise ValueError('Unsupported profile')
        if self.edge not in {384,336,320,288,256} or self.attention not in {'default','flash','efficient','efficient_expanded'}:
            raise ValueError('Unsupported image/attention budget')

    def prompt(self, stage):
        if stage=='planner' and self.planner_class:
            choices='; '.join(f"{code}={' '.join(fields)}" for code,fields in PLANNER_CLASS.items())
            return ('Observation is current truth; State is history. Visible LEFT/RIGHT: LOOK there. '
                    'Visible CENTER MID/FAR: APPROACH; otherwise LOOK CENTER. APPEARED allows FOUND. '
                    'Absent: SEARCH last known direction ONLY if transition DISAPPEARED or missed frames 1..3. '
                    'Missed frames >=4 do not authorize SEARCH. Otherwise avoid opposite a LEFT/RIGHT blocking obstacle; '
                    'CENTER/UNKNOWN/NONE obstacle or uncertain: WAIT UNKNOWN NONE UNKNOWN. '
                    'Visible actions target PERSON and observed distance. Search distance UNKNOWN. '
                    'Choose exactly one code for action/direction/target/distance. '+choices+
                    '. Return exactly one code character; no other tokens.',None)
        if self.format == 'json' or (stage=='planner' and self.planner_json):
            extra='Return minified one-line JSON with no spaces or newlines.' if self.minified else ''
            return (PERCEPTION_SYSTEM+extra, PERCEPTION_USER) if stage=='perception' else (PLANNER_SYSTEM+extra, None)
        if stage=='perception':
            if self.format=='class': return (None, CLASS_PERCEPTION)
            if self.exact: return (EXACT_PERCEPTION, 'Return seven code characters only.')
            if self.minimal: return (None, MINIMAL_PERCEPTION)
            return (COMPACT_PERCEPTION, 'Observe the current image.')
        return ((EXACT_PLANNER if self.exact else MINIMAL_PLANNER if self.minimal or self.format=='class' else COMPACT_PLANNER), None)

    def budget(self, stage):
        if stage=='planner' and self.planner_class: return 1
        return (64 if self.minified else 96) if self.stage_format(stage)=='json' else (1 if self.format=='class' and stage=='perception' else 8)

    def stage_format(self,stage):
        if stage=='planner' and self.planner_class: return 'planner_class'
        return 'json' if stage=='planner' and self.planner_json else self.format

    def to_dict(self): return asdict(self)


PROFILES = {p.name:p for p in [Profile('json'),Profile('compact',format='compact'),
            *[Profile(f'minimal_{edge}',format='compact',edge=edge,minimal=True) for edge in [384,336,320,288,256]],
            Profile('class_384',format='class',minimal=True),
            Profile('json_uninstrumented',instrument=False),Profile('compact_exact',format='compact',exact=True),
            Profile('class_constrained',format='class',constrained=True,planner_json=True)]}
PROFILES.update({p.name:p for p in [Profile('class_both',format='class',constrained=True,planner_class=True),
                *[Profile(f'class_{edge}',format='class',edge=edge,constrained=True,planner_json=True) for edge in [336,320,288,256]],
                *[Profile(f'class_both_{edge}',format='class',edge=edge,constrained=True,planner_class=True) for edge in [336,320,288,256]]]})
PROFILES.update({p.name:p for p in [Profile('json_linear',linear_patch=True),Profile('json_linear_uninstrumented',linear_patch=True,instrument=False),
                *[Profile(f'minified_linear_{edge}',edge=edge,linear_patch=True,minified=True) for edge in [384,336,320,288,256]]]})
