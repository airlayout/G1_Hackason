"""Kinematic G1 visualization and conservative convex-mesh clearance checks."""
import xml.etree.ElementTree as ET
import time
import numpy as np
from common import ROOT, MODEL, NAMES, ARM_NAMES

def make_scene():
    import mujoco
    tree=ET.parse(MODEL);root=tree.getroot()
    root.find('compiler').set('meshdir',str(ROOT/'external/g1_model/meshes'))
    visual=ET.SubElement(root,'visual')
    ET.SubElement(visual,'global',offwidth='1280',offheight='720')
    ET.SubElement(visual,'headlight',ambient='.5 .5 .5',diffuse='.8 .8 .8',specular='.1 .1 .1',active='1')
    ET.SubElement(visual,'rgba',haze='.2 .24 .3 1')
    for body in root.findall('worldbody'):
        for geom in list(body.findall('geom')):
            if geom.get('type')=='plane':body.remove(geom)
    world=root.find('worldbody')
    ET.SubElement(world,'light',pos='2 -3 5',dir='-0.2 0.3 -1',diffuse='0.8 0.8 0.8')
    ET.SubElement(world,'light',pos='-2 3 3',dir='0.2 -0.3 -1',diffuse='0.5 0.5 0.5')
    ET.SubElement(world,'geom',name='preview_floor',type='plane',size='8 8 .1',rgba='.32 .37 .43 1')
    model=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
    data=mujoco.MjData(model)
    for name in NAMES:
        model.joint(name)  # fail if names disagree, never assign by qpos offset assumption
    return model,data

def set_full(model,data,a,origin=None):
    import mujoco
    data.qpos[:]=model.qpos0
    data.qpos[:3]=a[:3]-(origin if origin is not None else 0)
    data.qpos[3:7]=a[3:7]/np.linalg.norm(a[3:7])
    for i,name in enumerate(NAMES):data.qpos[model.joint(name).qposadr[0]]=a[7+i]
    mujoco.mj_forward(model,data)

def set_joints(model,data,q,names,base=None):
    import mujoco
    data.qpos[:]=model.qpos0 if base is None else base
    for i,name in enumerate(names):data.qpos[model.joint(name).qposadr[0]]=q[i]
    mujoco.mj_forward(model,data)

def set_arms(model,data,q,base=None):
    set_joints(model,data,q,ARM_NAMES,base)

def clearance_pairs(model):
    """Query visual meshes too: some heads/hands have contype=0 in the MJCF.

    Distance is a convex hull approximation. Same-arm connected housings overlap
    intentionally; we check distal arms against torso, head, pelvis, legs and
    opposite arms. This is not a certified collision model.
    """
    import mujoco
    meshes={}
    for g in range(model.ngeom):
        if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
            mesh=model.mesh(int(model.geom_dataid[g])).name
            meshes.setdefault(mesh,g)
    pairs={}
    for side,other in [('left','right'),('right','left')]:
        distal=[n for n in meshes if n.startswith(side+'_') and any(x in n for x in ('elbow','wrist','rubber_hand'))]
        targets=[n for n in meshes if n in ('torso_link','head_link','pelvis','pelvis_contour_link')
                 or (n.startswith(other+'_') and any(x in n for x in ('shoulder','elbow','wrist','rubber_hand')))
                 or any(n.startswith(s+'_') and any(x in n for x in ('hip','knee')) for s in ('left','right'))]
        for a in distal:
            for b in targets:
                key=tuple(sorted((meshes[a],meshes[b])))
                pairs[key]=(a,b)
    return [(a,b,names) for (a,b),names in pairs.items()]

def clearance(model,data,pairs):
    import mujoco
    best=0.15;names=None
    for a,b,n in pairs:
        dist=mujoco.mj_geomDistance(model,data,a,b,0.15,None)
        if dist<best:best=float(dist);names=n
    return best,names

def pair_clearances(model,data,pairs):
    """Return every configured pair distance, keyed by stable mesh names."""
    import mujoco
    return {" | ".join(names):float(mujoco.mj_geomDistance(model,data,a,b,0.15,None))
            for a,b,names in pairs}

def measured_pose_clearances(model,data,reference_base,pairs,q):
    """The real runner's exact 29-joint measured-pose FK/pair evaluation.

    Shared with receive-only noise characterization; thresholds are applied
    by callers. Timings separate mj_forward from pair distance evaluation.
    """
    start=time.perf_counter()
    base=reference_base.copy()
    for i,name in enumerate(NAMES):base[model.joint(name).qposadr[0]]=q[i]
    set_arms(model,data,q[15:],base)
    fk_end=time.perf_counter()
    distances=pair_clearances(model,data,pairs)
    return distances,{'fk_s':fk_end-start,'collision_s':time.perf_counter()-fk_end}

def check_baseline_contact_path(entry,clip,return_path,base,moving_indices,
                                minimum=0.015,numerical_tolerance=1e-6,
                                path_joint_names=ARM_NAMES,
                                moving_mesh_prefixes=('right_',)):
    """Evaluate a q0-relative path without creating a permanent ignored pair.

    Pairs already below the configured margin at this measured q0 may approach
    that same q0 distance during entry/return, but may not get worse.  Baseline
    pairs involving the moving right arm must move away at motion onset and meet
    the full margin throughout the clip.  Static baseline pairs (for example the
    uncommanded left arm beside the thigh) must remain no worse than q0.  Every
    other pair retains the full margin for every phase.
    """
    entry=np.asarray(entry);clip=np.asarray(clip);return_path=np.asarray(return_path)
    width=len(path_joint_names)
    if any(x.ndim!=2 or x.shape[1]!=width or len(x)<1 for x in (entry,clip,return_path)):
        raise ValueError(f'Entry, clip and return must be non-empty Nx{width} paths')
    if not np.allclose(entry[0],return_path[-1],atol=1e-9):
        raise ValueError('Entry must start and return must end at the same measured q0')
    model,data=make_scene();pairs=clearance_pairs(model)

    def evaluate(path):
        rows=[]
        for pose in path:
            set_joints(model,data,pose,path_joint_names,base)
            rows.append(pair_clearances(model,data,pairs))
        return rows

    phase_rows={'entry':evaluate(entry),'clip':evaluate(clip),'return':evaluate(return_path)}
    baseline=phase_rows['entry'][0]
    baseline_keys={key for key,value in baseline.items() if value<minimum}
    baseline_collision_keys={key for key,value in baseline.items() if value<0}
    # Mesh naming is not one-to-one with joints. A commanded arm moves all of
    # that side's distal housings; waist motion may move both sides.
    moving_baseline={key for key in baseline_keys
                     if any(part.startswith(moving_mesh_prefixes)
                            for part in key.split(' | '))}
    inactive_baseline=baseline_keys-moving_baseline
    all_keys=set(baseline)

    motion_index=None
    for i,pose in enumerate(entry):
        if np.max(np.abs(pose[list(moving_indices)]-entry[0,list(moving_indices)]))>1e-4:
            motion_index=i;break
    onset={}
    for key in sorted(moving_baseline):
        delta=None if motion_index is None else phase_rows['entry'][motion_index][key]-baseline[key]
        onset[key]={'first_motion_frame':motion_index,'delta_m':delta,
                    'increased':delta is not None and delta>numerical_tolerance}

    phase_reports={}
    baseline_minima={key:min(row[key] for rows in phase_rows.values() for row in rows)
                     for key in baseline_keys}
    worsened=[key for key in sorted(baseline_keys)
              if baseline_minima[key]<baseline[key]-numerical_tolerance]
    for phase,rows in phase_rows.items():
        minima={key:min(row[key] for row in rows) for key in all_keys}
        new_deficits=[key for key in sorted(all_keys-baseline_keys) if minima[key]<minimum]
        # A pair that was positive at q0 but penetrates later is a new collision,
        # even when q0 was already inside the conservative 15 mm margin.
        new_collisions=[key for key in sorted(all_keys-baseline_collision_keys)
                        if minima[key]<0]
        exempt=inactive_baseline | (moving_baseline if phase!='clip' else set())
        effective=all_keys-exempt
        effective_key=min(effective,key=lambda key:minima[key])
        raw_key=min(all_keys,key=lambda key:minima[key])
        phase_reports[phase]={
            'frames_checked':len(rows),
            'raw_minimum_clearance_m':minima[raw_key],
            'raw_closest_pair':raw_key.split(' | '),
            'minimum_required_pair_clearance_m':minima[effective_key],
            'minimum_required_pair':effective_key.split(' | '),
            'new_clearance_deficit_pairs':new_deficits,
            'new_collision_pairs':new_collisions,
            'passed':not new_deficits and not new_collisions and
                     (phase!='clip' or all(minima[key]>=minimum for key in moving_baseline)),
        }

    onset_pass=not moving_baseline or all(x['increased'] for x in onset.values())
    baseline_rules={
        'no_baseline_pair_worsened':not worsened,
        'worsened_pairs':worsened,
        'moving_baseline_clearance_increased_at_onset':onset_pass,
        'onset_measurements':onset,
        'numerical_tolerance_m':numerical_tolerance,
    }
    passed=(all(x['passed'] for x in phase_reports.values()) and
            baseline_rules['no_baseline_pair_worsened'] and onset_pass)
    raw_min=min(x['raw_minimum_clearance_m'] for x in phase_reports.values())
    return {
        'passed':passed,'minimum_clearance_m':raw_min,
        'required_clearance_m':minimum,
        'baseline':{
            'measured_q0_only':True,
            'subthreshold_pairs':[
                {'pair':key.split(' | '),'distance_m':baseline[key],
                 'penetrating':baseline[key]<0,
                 'moves_with_commanded_right_arm':key in moving_baseline}
                for key in sorted(baseline_keys)
            ],
            'penetrating_pairs':[key for key in sorted(baseline_keys) if baseline[key]<0],
            'moving_pairs':sorted(moving_baseline),'inactive_pairs':sorted(inactive_baseline),
            'pair_minimum_over_full_path_m':baseline_minima,
        },
        'baseline_rules':baseline_rules,'phases':phase_reports,
        'new_collision_pairs':sorted(set().union(
            *(set(x['new_collision_pairs']) for x in phase_reports.values()))),
        'new_clearance_deficit_pairs':sorted(set().union(
            *(set(x['new_clearance_deficit_pairs']) for x in phase_reports.values()))),
        'method':'q0-relative convex mesh FK; baseline is per measured pose, never a permanent ignore list',
    }

def check_arm_path(q, base=None, minimum=0.015):
    model,data=make_scene();pairs=clearance_pairs(model)
    best=0.15;hit=None
    for i,pose in enumerate(q):
        set_arms(model,data,pose,base)
        d,n=clearance(model,data,pairs)
        if d<best:best=d;hit={'frame':i,'meshes':n}
    return {'minimum_clearance_m':best,'closest':hit,'required_clearance_m':minimum,
            'passed':best>=minimum,'frames_checked':len(q),
            'method':'convex mesh FK, distal arms vs torso/head/pelvis/legs/opposite arms; not dynamics'}
