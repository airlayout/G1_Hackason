#!/usr/bin/env python3
"""Offline closest-point audit for the two q0 right-arm/hip penetrations."""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from common import ROOT, MODEL, URDF, NAMES, digest, output_path, write_json
from scene import make_scene, clearance_pairs, measured_pose_clearances

TARGETS=[('right_wrist_yaw_link','right_hip_pitch_link'),
         ('right_rubber_hand','right_hip_pitch_link')]


def _list(value):return np.asarray(value).tolist()


def urdf_link_record(mesh_name):
    root=ET.parse(URDF).getroot();link=root.find(f"./link[@name='{mesh_name}']")
    result={}
    for kind in ('visual','collision'):
        element=link.find(kind) if link is not None else None
        mesh=element.find('./geometry/mesh') if element is not None else None
        origin=element.find('origin') if element is not None else None
        result[kind]={'mesh':None if mesh is None else mesh.get('filename'),
                      'origin_xyz':None if origin is None else origin.get('xyz','0 0 0'),
                      'origin_rpy':None if origin is None else origin.get('rpy','0 0 0')}
    joint=root.find(f"./joint/child[@link='{mesh_name}']/..")
    if joint is None:
        joint=next((j for j in root.findall('joint')
                    if j.find('child') is not None and j.find('child').get('link')==mesh_name),None)
    if joint is not None:
        origin=joint.find('origin');parent=joint.find('parent')
        result['parent_joint']={'name':joint.get('name'),'type':joint.get('type'),
            'parent':None if parent is None else parent.get('link'),
            'origin_xyz':None if origin is None else origin.get('xyz','0 0 0'),
            'origin_rpy':None if origin is None else origin.get('rpy','0 0 0')}
    else:result['parent_joint']=None
    return result


def geom_record(model,data,geom_id,selected):
    import mujoco
    mesh_id=int(model.geom_dataid[geom_id]);mesh=model.mesh(mesh_id).name
    body_id=int(model.geom_bodyid[geom_id])
    source=ROOT/'external/g1_model/meshes'/f'{mesh}.STL'
    return {'geom_id':geom_id,'selected_by_runtime_pair_builder':geom_id==selected,
            'mesh_name':mesh,'mesh_source':str(source),'mesh_sha256':digest(source),
            'geom_group':int(model.geom_group[geom_id]),
            'contype':int(model.geom_contype[geom_id]),
            'conaffinity':int(model.geom_conaffinity[geom_id]),
            'body_name':model.body(body_id).name,
            'world_position_m':_list(data.geom_xpos[geom_id]),
            'world_rotation_matrix':_list(data.geom_xmat[geom_id].reshape(3,3)),
            'compiled_geom_position_m':_list(model.geom_pos[geom_id]),
            'compiled_geom_quaternion_wxyz':_list(model.geom_quat[geom_id]),
            'compiled_mesh_scale':_list(model.mesh_scale[mesh_id]),
            'compiled_mesh_position_m':_list(model.mesh_pos[mesh_id]),
            'compiled_mesh_quaternion_wxyz':_list(model.mesh_quat[mesh_id]),
            'is_mesh':bool(model.geom_type[geom_id]==mujoco.mjtGeom.mjGEOM_MESH)}


def render_pair(model,data,pair,point_a,point_b,distance,path):
    import mujoco
    from PIL import Image,ImageDraw
    model.geom_rgba[:,3]=.12
    colors={pair[0]:[1,.15,.05,1],pair[1]:[.05,.45,1,1]}
    for geom_id in range(model.ngeom):
        if model.geom_type[geom_id]==mujoco.mjtGeom.mjGEOM_MESH:
            name=model.mesh(int(model.geom_dataid[geom_id])).name
            if name in colors:model.geom_rgba[geom_id]=colors[name]
    midpoint=(point_a+point_b)/2;renderer=mujoco.Renderer(model,height=600,width=800)
    panels=[]
    for azimuth,elevation in ((90,-10),(180,-10),(45,-35),(270,0)):
        camera=mujoco.MjvCamera();camera.type=mujoco.mjtCamera.mjCAMERA_FREE
        camera.lookat=midpoint;camera.distance=.35;camera.azimuth=azimuth;camera.elevation=elevation
        renderer.update_scene(data,camera)
        image=Image.fromarray(renderer.render());draw=ImageDraw.Draw(image)
        draw.text((10,10),f'visual mesh; az={azimuth}; signed convex distance={distance*1000:.3f} mm',
                  fill='white',stroke_width=2,stroke_fill='black')
        panels.append(image)
    renderer.close();sheet=Image.new('RGB',(1600,1200))
    for index,image in enumerate(panels):sheet.paste(image,((index%2)*800,(index//2)*600))
    output_path(path);sheet.save(path)


def audit(q0_path,output_dir,preview=True,current_fresh=False):
    import mujoco
    q0_path=Path(q0_path);source=json.loads(q0_path.read_text());q=np.asarray(source['all_q'])
    model,data=make_scene();pairs=clearance_pairs(model);base=model.qpos0.copy()
    measured_pose_clearances(model,data,base,pairs,q)
    output_dir=Path(output_dir);output_dir.mkdir(parents=True,exist_ok=True)
    records=[]
    for pair in TARGETS:
        selected=next((value for value in pairs if tuple(value[2])==pair),None)
        if selected is None:raise RuntimeError('Runtime pair missing: '+' / '.join(pair))
        geom_1,geom_2,_=selected;fromto=np.zeros(6)
        distance=float(mujoco.mj_geomDistance(model,data,geom_1,geom_2,.15,fromto))
        selected_by_name={model.mesh(int(model.geom_dataid[geom_id])).name:geom_id
                          for geom_id in (geom_1,geom_2)}
        points_by_name={model.mesh(int(model.geom_dataid[geom_1])).name:fromto[:3].copy(),
                        model.mesh(int(model.geom_dataid[geom_2])).name:fromto[3:].copy()}
        point_a=points_by_name[pair[0]];point_b=points_by_name[pair[1]]
        geometries={}
        for name in pair:
            selected_id=selected_by_name[name]
            ids=[i for i in range(model.ngeom)
                 if model.geom_type[i]==mujoco.mjtGeom.mjGEOM_MESH and
                 model.mesh(int(model.geom_dataid[i])).name==name]
            geometries[name]={'runtime_selected_geom_id':selected_id,
                              'compiled_geometries':[geom_record(model,data,i,selected_id) for i in ids],
                              'urdf':urdf_link_record(name),
                              'body_world_position_m':_list(data.xpos[int(model.geom_bodyid[selected_id])]),
                              'body_world_rotation_matrix':_list(data.xmat[int(model.geom_bodyid[selected_id])].reshape(3,3))}
        stem='right_wrist_yaw_vs_hip' if 'wrist' in pair[0] else 'right_hand_vs_hip'
        preview_path=output_dir/f'{stem}.png'
        if preview:render_pair(model,data,pair,point_a,point_b,distance,preview_path)
        records.append({'pair':list(pair),'signed_distance_m':distance,
            'signed_distance_mm':distance*1000,'closest_point_on_body_a_m':_list(point_a),
            'closest_point_on_body_b_m':_list(point_b),
            'closest_point_vector_a_to_b_m':_list(point_b-point_a),
            'closest_point_vector_norm_m':float(np.linalg.norm(point_b-point_a)),
            'distance_method':'same scene clearance pair and mujoco.mj_geomDistance used by runtime; mesh distance uses MuJoCo convex representation',
            'geometry':geometries,'joint_state_used_rad':dict(zip(NAMES,q.tolist())),
            'preview':str(preview_path) if preview else None,
            'visual_interpretation':'Preview renders the source mesh surfaces; numerical closest points are recorded in geometry_audit.json. The signed value is the runtime convex-mesh result and the convex hull itself is not rendered. This is a model view, not independent evidence of physical contact.',
            'classification':'GEO-E',
            'classification_reason':'A saved joint state and convex mesh model cannot distinguish physical contact from a convex-hull/model artifact without a fresh matching pose and independent physical observation.'})
    timestamp=source.get('timestamp') or source.get('checked_local_time')
    report={'schema':'motiondecode-test.geometry-audit.v1','model':str(MODEL),
            'model_sha256':digest(MODEL),'q0_source':str(q0_path.resolve()),
            'q0_source_sha256':digest(q0_path),'q0_timestamp':timestamp,
            'q0_is_current_fresh':bool(current_fresh),'pairs':records,
            'overall_classification':'GEO-E',
            'allowlist_change':False,'threshold_change':False,
            'limitations':['MuJoCo mesh distance is based on its convex collision representation.',
                           'The preview renders model geometry and cannot establish real-world contact.',
                           'GEO-E pairs must not be allowlisted or used to authorize positive-weight HOLD.']}
    write_json(output_dir.parent/'geometry_audit.json',report)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--q0',type=Path,required=True)
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--current-fresh',action='store_true')
    parser.add_argument('--no-preview',action='store_true')
    args=parser.parse_args()
    audit(args.q0,args.output_dir,not args.no_preview,args.current_fresh)
    return 0


if __name__=='__main__':raise SystemExit(main())
