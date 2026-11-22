import os
import sys
import argparse
import csv
import struct
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation as R

try:
    # The Autodesk FBX Python SDK is proprietary and not installable from PyPI
    # (see README). It's only required for --output *.fbx; BVH export works
    # without it, so importing this module must not fail when it's missing.
    import fbx
except ImportError:
    fbx = None


STL_TRIANGLE_DTYPE = np.dtype([
    ("normal", "<f4", 3),
    ("v0", "<f4", 3),
    ("v1", "<f4", 3),
    ("v2", "<f4", 3),
    ("attr", "<u2"),
])


def load_stl_triangles(stl_path):
    """
    Reads a binary STL file (as used by g1_description/meshes/*.STL) and
    returns (vertices, normals): vertices has shape (num_triangles, 3, 3),
    normals has shape (num_triangles, 3). Vertex coordinates are already in
    meters, in the owning link's local frame, matching the URDF joint offsets.
    """
    with open(stl_path, "rb") as f:
        f.read(80)  # 80-byte header, ignored
        triangle_count = struct.unpack("<I", f.read(4))[0]
        data = np.fromfile(f, dtype=STL_TRIANGLE_DTYPE, count=triangle_count)
    vertices = np.stack([data["v0"], data["v1"], data["v2"]], axis=1)
    normals = data["normal"]
    return vertices, normals


def create_fbx_mesh_from_stl(scene, name, stl_path):
    vertices, normals = load_stl_triangles(stl_path)
    num_triangles = vertices.shape[0]

    mesh = fbx.FbxMesh.Create(scene, name)
    mesh.InitControlPoints(num_triangles * 3)

    flat_vertices = vertices.reshape(-1, 3)
    for i, (x, y, z) in enumerate(flat_vertices):
        mesh.SetControlPointAt(fbx.FbxVector4(float(x), float(y), float(z), 1.0), i)

    # Per-polygon-vertex (not per-polygon) direct normals: this is the mapping/reference
    # mode combination every mainstream FBX exporter and the Autodesk SDK samples use,
    # so it's the best-tested path through third-party importers (e.g. MotionBuilder).
    normal_element = mesh.CreateElementNormal()
    normal_element.SetMappingMode(fbx.FbxLayerElement.EMappingMode.eByPolygonVertex)
    normal_element.SetReferenceMode(fbx.FbxLayerElement.EReferenceMode.eDirect)
    normal_array = normal_element.GetDirectArray()

    material_element = mesh.CreateElementMaterial()
    material_element.SetMappingMode(fbx.FbxLayerElement.EMappingMode.eAllSame)
    material_element.SetReferenceMode(fbx.FbxLayerElement.EReferenceMode.eIndexToDirect)
    material_element.GetIndexArray().Add(0)

    for i in range(num_triangles):
        mesh.BeginPolygon()
        mesh.AddPolygon(i * 3 + 0)
        mesh.AddPolygon(i * 3 + 1)
        mesh.AddPolygon(i * 3 + 2)
        mesh.EndPolygon()
        nx, ny, nz = normals[i]
        for _ in range(3):
            normal_array.Add(fbx.FbxVector4(float(nx), float(ny), float(nz), 0.0))

    return mesh


def parse_urdf_visuals(urdf_path):
    """
    Parses g1_29dof.urdf and returns:
      materials     : material name -> (r, g, b, a)
      link_visuals  : link name -> (mesh_filename, material_name)
      joints        : joint name -> {type, parent, child, xyz, rpy(radians)}
    """
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    materials = {}
    for mat_el in root.findall("material"):
        color_el = mat_el.find("color")
        if color_el is not None:
            materials[mat_el.get("name")] = tuple(float(v) for v in color_el.get("rgba").split())

    link_visuals = {}
    for link_el in root.findall("link"):
        visual_el = link_el.find("visual")
        if visual_el is None:
            continue
        mesh_el = visual_el.find("geometry/mesh")
        if mesh_el is None:
            continue
        mesh_filename = mesh_el.get("filename").split("/")[-1]
        material_el = visual_el.find("material")
        material_name = material_el.get("name") if material_el is not None else None
        link_visuals[link_el.get("name")] = (mesh_filename, material_name)

    joints = {}
    for joint_el in root.findall("joint"):
        origin_el = joint_el.find("origin")
        xyz = tuple(float(v) for v in origin_el.get("xyz").split()) if origin_el is not None else (0.0, 0.0, 0.0)
        rpy = (tuple(float(v) for v in origin_el.get("rpy").split())
               if origin_el is not None and origin_el.get("rpy") else (0.0, 0.0, 0.0))
        joints[joint_el.get("name")] = {
            "type": joint_el.get("type"),
            "parent": joint_el.find("parent").get("link"),
            "child": joint_el.find("child").get("link"),
            "xyz": xyz,
            "rpy": rpy,
        }

    return materials, link_visuals, joints


def get_or_create_material(scene, material_cache, urdf_materials, material_name):
    cache_key = material_name or "default"
    if cache_key in material_cache:
        return material_cache[cache_key]

    r, g, b = urdf_materials.get(material_name, (0.6, 0.6, 0.6, 1.0))[:3]
    material = fbx.FbxSurfacePhong.Create(scene, cache_key)
    material.Diffuse.Set(fbx.FbxDouble3(r, g, b))
    material_cache[cache_key] = material
    return material


def attach_mesh_node(scene, parent_node, node_name, stl_path, material):
    mesh = create_fbx_mesh_from_stl(scene, node_name + "_mesh", stl_path)
    mesh_node = fbx.FbxNode.Create(scene, node_name + "_mesh")
    mesh_node.SetNodeAttribute(mesh)
    mesh_node.AddMaterial(material)
    parent_node.AddChild(mesh_node)
    return mesh_node


def attach_fixed_mesh_node(scene, parent_node, link_name, xyz, rpy, stl_path, material):
    node = fbx.FbxNode.Create(scene, link_name)
    node.LclTranslation.Set(fbx.FbxDouble3(*xyz))
    if any(rpy):
        rx_deg, ry_deg, rz_deg = (np.degrees(v) for v in rpy)
        node.LclRotation.Set(fbx.FbxDouble3(rx_deg, ry_deg, rz_deg))

    mesh = create_fbx_mesh_from_stl(scene, link_name + "_mesh", stl_path)
    node.SetNodeAttribute(mesh)
    node.AddMaterial(material)
    parent_node.AddChild(node)
    return node


def attach_meshes(scene, fbx_nodes, urdf_path, mesh_dir):
    """
    Attaches a visual FbxMesh child to every skeleton node built by
    build_g1_skeleton_nodes, plus the handful of rigidly-fixed links
    (head, logo, waist support, rubber hands, pelvis contour) that hang
    off a fixed (non-DOF) joint in the URDF.
    """
    urdf_materials, link_visuals, joints = parse_urdf_visuals(urdf_path)
    material_cache = {}

    # Every skeleton node maps 1:1 to the URDF link it represents: "pelvis" is
    # the root link, and every "dof_<joint_name>" node is that joint's child link.
    link_to_node = {"pelvis": fbx_nodes["pelvis"]}
    for key, node in fbx_nodes.items():
        if key == "pelvis":
            continue
        joint_name = key[len("dof_"):]
        link_to_node[joints[joint_name]["child"]] = node

    for link_name, node in list(link_to_node.items()):
        if link_name not in link_visuals:
            continue
        mesh_filename, material_name = link_visuals[link_name]
        material = get_or_create_material(scene, material_cache, urdf_materials, material_name)
        attach_mesh_node(scene, node, link_name, os.path.join(mesh_dir, mesh_filename), material)

    # Links only reachable through a fixed joint (rubber hands, head, logo, ...)
    # aren't in fbx_nodes yet; hang their mesh directly off the fixed joint's
    # parent, using the joint's static origin as the node's rest transform.
    for info in joints.values():
        if info["type"] != "fixed":
            continue
        child_link = info["child"]
        if child_link not in link_visuals or child_link in link_to_node:
            continue
        parent_node = link_to_node.get(info["parent"])
        if parent_node is None:
            continue
        mesh_filename, material_name = link_visuals[child_link]
        material = get_or_create_material(scene, material_cache, urdf_materials, material_name)
        new_node = attach_fixed_mesh_node(
            scene, parent_node, child_link, info["xyz"], info["rpy"],
            os.path.join(mesh_dir, mesh_filename), material
        )
        link_to_node[child_link] = new_node


def get_g1_joint_definitions():
    """
    Returns the Unitree G1 29-DOF joint hierarchy as plain data: name -> {parent,
    translation, axis, csv_col, rpy (optional), is_root (optional)}. This has no
    FBX dependency so it can be shared by both the FBX and BVH export paths.

    Translations and static mounting tilts (rpy, in radians) are taken directly
    from g1_description/urdf/g1_29dof.urdf joint origins, so the FK chain
    matches the real robot geometry instead of an approximated straight-segment rig.
    """
    return {
        "pelvis": {
            "parent": None,
            "translation": (0.0, 0.0, 0.0),
            "is_root": True
        },
        # Left Leg
        "dof_left_hip_pitch_joint": {
            "parent": "pelvis",
            "translation": (0.0, 0.064452, -0.1027),
            "axis": "Y",
            "csv_col": "dof_left_hip_pitch_joint(rad)"
        },
        "dof_left_hip_roll_joint": {
            "parent": "dof_left_hip_pitch_joint",
            "translation": (0.0, 0.052, -0.030465),
            "rpy": (0.0, -0.1749, 0.0),
            "axis": "X",
            "csv_col": "dof_left_hip_roll_joint(rad)"
        },
        "dof_left_hip_yaw_joint": {
            "parent": "dof_left_hip_roll_joint",
            "translation": (0.025001, 0.0, -0.12412),
            "axis": "Z",
            "csv_col": "dof_left_hip_yaw_joint(rad)"
        },
        "dof_left_knee_joint": {
            "parent": "dof_left_hip_yaw_joint",
            "translation": (-0.078273, 0.0021489, -0.17734),
            "rpy": (0.0, 0.1749, 0.0),
            "axis": "Y",
            "csv_col": "dof_left_knee_joint(rad)"
        },
        "dof_left_ankle_pitch_joint": {
            "parent": "dof_left_knee_joint",
            "translation": (0.0, -0.000094445, -0.30001),
            "axis": "Y",
            "csv_col": "dof_left_ankle_pitch_joint(rad)"
        },
        "dof_left_ankle_roll_joint": {
            "parent": "dof_left_ankle_pitch_joint",
            "translation": (0.0, 0.0, -0.017558),
            "axis": "X",
            "csv_col": "dof_left_ankle_roll_joint(rad)"
        },

        # Right Leg
        "dof_right_hip_pitch_joint": {
            "parent": "pelvis",
            "translation": (0.0, -0.064452, -0.1027),
            "axis": "Y",
            "csv_col": "dof_right_hip_pitch_joint(rad)"
        },
        "dof_right_hip_roll_joint": {
            "parent": "dof_right_hip_pitch_joint",
            "translation": (0.0, -0.052, -0.030465),
            "rpy": (0.0, -0.1749, 0.0),
            "axis": "X",
            "csv_col": "dof_right_hip_roll_joint(rad)"
        },
        "dof_right_hip_yaw_joint": {
            "parent": "dof_right_hip_roll_joint",
            "translation": (0.025001, 0.0, -0.12412),
            "axis": "Z",
            "csv_col": "dof_right_hip_yaw_joint(rad)"
        },
        "dof_right_knee_joint": {
            "parent": "dof_right_hip_yaw_joint",
            "translation": (-0.078273, -0.0021489, -0.17734),
            "rpy": (0.0, 0.1749, 0.0),
            "axis": "Y",
            "csv_col": "dof_right_knee_joint(rad)"
        },
        "dof_right_ankle_pitch_joint": {
            "parent": "dof_right_knee_joint",
            "translation": (0.0, 0.000094445, -0.30001),
            "axis": "Y",
            "csv_col": "dof_right_ankle_pitch_joint(rad)"
        },
        "dof_right_ankle_roll_joint": {
            "parent": "dof_right_ankle_pitch_joint",
            "translation": (0.0, 0.0, -0.017558),
            "axis": "X",
            "csv_col": "dof_right_ankle_roll_joint(rad)"
        },

        # Waist / Torso
        "dof_waist_yaw_joint": {
            "parent": "pelvis",
            "translation": (0.0, 0.0, 0.0),
            "axis": "Z",
            "csv_col": "dof_waist_yaw_joint(rad)"
        },
        "dof_waist_roll_joint": {
            "parent": "dof_waist_yaw_joint",
            "translation": (-0.0039635, 0.0, 0.035),
            "axis": "X",
            "csv_col": "dof_waist_roll_joint(rad)"
        },
        "dof_waist_pitch_joint": {
            "parent": "dof_waist_roll_joint",
            "translation": (0.0, 0.0, 0.019),
            "axis": "Y",
            "csv_col": "dof_waist_pitch_joint(rad)"
        },

        # Left Arm
        "dof_left_shoulder_pitch_joint": {
            "parent": "dof_waist_pitch_joint",
            "translation": (0.0039563, 0.10022, 0.23778),
            "rpy": (0.27931, 0.000054949, -0.00019159),
            "axis": "Y",
            "csv_col": "dof_left_shoulder_pitch_joint(rad)"
        },
        "dof_left_shoulder_roll_joint": {
            "parent": "dof_left_shoulder_pitch_joint",
            "translation": (0.0, 0.038, -0.013831),
            "rpy": (-0.27925, 0.0, 0.0),
            "axis": "X",
            "csv_col": "dof_left_shoulder_roll_joint(rad)"
        },
        "dof_left_shoulder_yaw_joint": {
            "parent": "dof_left_shoulder_roll_joint",
            "translation": (0.0, 0.00624, -0.1032),
            "axis": "Z",
            "csv_col": "dof_left_shoulder_yaw_joint(rad)"
        },
        "dof_left_elbow_joint": {
            "parent": "dof_left_shoulder_yaw_joint",
            "translation": (0.015783, 0.0, -0.080518),
            "axis": "Y",
            "csv_col": "dof_left_elbow_joint(rad)"
        },
        "dof_left_wrist_roll_joint": {
            "parent": "dof_left_elbow_joint",
            "translation": (0.100, 0.00188791, -0.010),
            "axis": "X",
            "csv_col": "dof_left_wrist_roll_joint(rad)"
        },
        "dof_left_wrist_pitch_joint": {
            "parent": "dof_left_wrist_roll_joint",
            "translation": (0.038, 0.0, 0.0),
            "axis": "Y",
            "csv_col": "dof_left_wrist_pitch_joint(rad)"
        },
        "dof_left_wrist_yaw_joint": {
            "parent": "dof_left_wrist_pitch_joint",
            "translation": (0.046, 0.0, 0.0),
            "axis": "Z",
            "csv_col": "dof_left_wrist_yaw_joint(rad)"
        },

        # Right Arm
        "dof_right_shoulder_pitch_joint": {
            "parent": "dof_waist_pitch_joint",
            "translation": (0.0039563, -0.10021, 0.23778),
            "rpy": (-0.27931, 0.000054949, 0.00019159),
            "axis": "Y",
            "csv_col": "dof_right_shoulder_pitch_joint(rad)"
        },
        "dof_right_shoulder_roll_joint": {
            "parent": "dof_right_shoulder_pitch_joint",
            "translation": (0.0, -0.038, -0.013831),
            "rpy": (0.27925, 0.0, 0.0),
            "axis": "X",
            "csv_col": "dof_right_shoulder_roll_joint(rad)"
        },
        "dof_right_shoulder_yaw_joint": {
            "parent": "dof_right_shoulder_roll_joint",
            "translation": (0.0, -0.00624, -0.1032),
            "axis": "Z",
            "csv_col": "dof_right_shoulder_yaw_joint(rad)"
        },
        "dof_right_elbow_joint": {
            "parent": "dof_right_shoulder_yaw_joint",
            "translation": (0.015783, 0.0, -0.080518),
            "axis": "Y",
            "csv_col": "dof_right_elbow_joint(rad)"
        },
        "dof_right_wrist_roll_joint": {
            "parent": "dof_right_elbow_joint",
            "translation": (0.100, -0.00188791, -0.010),
            "axis": "X",
            "csv_col": "dof_right_wrist_roll_joint(rad)"
        },
        "dof_right_wrist_pitch_joint": {
            "parent": "dof_right_wrist_roll_joint",
            "translation": (0.038, 0.0, 0.0),
            "axis": "Y",
            "csv_col": "dof_right_wrist_pitch_joint(rad)"
        },
        "dof_right_wrist_yaw_joint": {
            "parent": "dof_right_wrist_pitch_joint",
            "translation": (0.046, 0.0, 0.0),
            "axis": "Z",
            "csv_col": "dof_right_wrist_yaw_joint(rad)"
        },
    }


def build_g1_skeleton_nodes(scene):
    """
    Builds the Unitree G1 29-DOF skeleton node hierarchy in FBX.
    Returns a dictionary mapping joint/bone names to FbxNode instances,
    along with joint axis information and default local translations.
    """
    nodes_def = get_g1_joint_definitions()
    fbx_nodes = {}
    root_scene_node = scene.GetRootNode()

    for name, info in nodes_def.items():
        node = fbx.FbxNode.Create(scene, name)
        skeleton = fbx.FbxSkeleton.Create(scene, name)
        if info.get("is_root"):
            skeleton.SetSkeletonType(fbx.FbxSkeleton.EType.eRoot)
        else:
            skeleton.SetSkeletonType(fbx.FbxSkeleton.EType.eLimbNode)
        skeleton.Size.Set(1.0)
        node.SetNodeAttribute(skeleton)

        # Set default translation
        tx, ty, tz = info["translation"]
        node.LclTranslation.Set(fbx.FbxDouble3(tx, ty, tz))
        node.LclRotation.Set(fbx.FbxDouble3(0.0, 0.0, 0.0))

        # Static mounting tilt from the URDF joint's <origin rpy="...">, applied
        # before the animated single-axis DOF rotation (LclRotation) via PreRotation.
        rpy_rad = info.get("rpy")
        if rpy_rad:
            rx_deg, ry_deg, rz_deg = (np.degrees(v) for v in rpy_rad)
            node.SetRotationActive(True)
            node.SetPreRotation(fbx.FbxNode.EPivotSet.eSourcePivot, fbx.FbxVector4(rx_deg, ry_deg, rz_deg, 0.0))

        parent_name = info["parent"]
        if parent_name is None:
            root_scene_node.AddChild(node)
        else:
            fbx_nodes[parent_name].AddChild(node)

        fbx_nodes[name] = node

    return fbx_nodes, nodes_def


def load_motion_csv(csv_path):
    """
    Reads the motion CSV and returns (rows, root_pos, root_quats):
      rows       : list of csv.DictReader rows (still needed for per-joint columns)
      root_pos   : (num_frames, 3) array of root_pos_x/y/z(m)
      root_quats : (num_frames, 4) array [x, y, z, w], sign-corrected for
                   continuity across frames (no antipodal quaternion flips)
    Shared by both the FBX and BVH export paths.
    """
    with open(csv_path, 'r', newline='') as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    num_frames = len(rows)
    root_pos = np.zeros((num_frames, 3), dtype=np.float64)
    root_quats = np.zeros((num_frames, 4), dtype=np.float64)  # [x, y, z, w]

    for i, row in enumerate(rows):
        root_pos[i] = [
            float(row["root_pos_x(m)"]),
            float(row["root_pos_y(m)"]),
            float(row["root_pos_z(m)"])
        ]
        qw = float(row["root_rot_w"])
        qx = float(row["root_rot_x"])
        qy = float(row["root_rot_y"])
        qz = float(row["root_rot_z"])
        root_quats[i] = [qx, qy, qz, qw]

    # Ensure quaternion sign continuity to avoid sign flips
    for i in range(1, num_frames):
        if np.dot(root_quats[i], root_quats[i - 1]) < 0:
            root_quats[i] = -root_quats[i]

    return rows, root_pos, root_quats


def add_keyframe(curve, time_obj, val):
    curve.KeyModifyBegin()
    key_idx = curve.KeyAdd(time_obj)[0]
    curve.KeySetValue(key_idx, float(val))
    curve.KeySetInterpolation(key_idx, fbx.FbxAnimCurveDef.EInterpolationType.eInterpolationLinear)
    curve.KeyModifyEnd()


def convert_csv_to_fbx(csv_path, fbx_path, fps=30.0, unwrap=True, add_mesh=True,
                        urdf_path="g1_description/urdf/g1_29dof.urdf",
                        mesh_dir="g1_description/meshes"):
    if fbx is None:
        raise RuntimeError(
            "The Autodesk FBX Python SDK is not installed, so .fbx export is "
            "unavailable. Install it (see README) or export to BVH instead "
            "with an --output path ending in .bvh."
        )

    print(f"Reading CSV dataset: {csv_path}")
    rows, root_pos, root_quats = load_motion_csv(csv_path)
    num_frames = len(rows)
    print(f"Loaded {num_frames} frames of motion data.")

    # Convert root quaternions to Euler angles, resolving gimbal lock (see
    # _continuous_euler) and optionally unwrapping +/- 180 deg boundaries across time
    root_eulers_final = _continuous_euler(R.from_quat(root_quats), 'xyz', unwrap)

    # Initialize FBX Manager and Scene
    manager = fbx.FbxManager.Create()
    scene = fbx.FbxScene.Create(manager, "Unitree_G1_Scene")

    # Declare the authoring frame rate explicitly (as a custom rate, not just the
    # eFrames30 enum) so every importer converts our FbxTime keys (placed via
    # SetSecondDouble) back into frame numbers the same way. Without this, some
    # importers (e.g. Blender's FBX importer) fall back to their own default
    # frame rate and misinterpret the timing/length of the whole animation.
    global_settings = scene.GetGlobalSettings()
    global_settings.SetTimeMode(fbx.FbxTime.EMode.eCustom)
    custom_frame_rate_prop = global_settings.FindProperty("CustomFrameRate")
    if custom_frame_rate_prop.IsValid():
        custom_frame_rate_prop.Set(fps)
    fbx.FbxTime.SetGlobalTimeMode(fbx.FbxTime.EMode.eCustom, fps)

    anim_start_time = fbx.FbxTime()
    anim_stop_time = fbx.FbxTime()
    anim_start_time.SetSecondDouble(0.0)
    anim_stop_time.SetSecondDouble((num_frames - 1) / fps)
    anim_time_span = fbx.FbxTimeSpan(anim_start_time, anim_stop_time)
    global_settings.SetTimelineDefaultTimeSpan(anim_time_span)

    # Set scene axis system (Z-Up, Right-Handed)
    axis_system = fbx.FbxAxisSystem(
        fbx.FbxAxisSystem.EUpVector.eZAxis,
        fbx.FbxAxisSystem.EFrontVector.eParityEven,
        fbx.FbxAxisSystem.ECoordSystem.eRightHanded
    )
    axis_system.ConvertScene(scene)

    # Set System Unit (Meters)
    system_unit = fbx.FbxSystemUnit.m
    system_unit.ConvertScene(scene)

    # Create Skeleton Nodes
    fbx_nodes, nodes_def = build_g1_skeleton_nodes(scene)

    if add_mesh:
        print(f"Attaching meshes from: {mesh_dir}")
        attach_meshes(scene, fbx_nodes, urdf_path, mesh_dir)

    # Create Animation Stack & Layer
    anim_stack = fbx.FbxAnimStack.Create(scene, "G1_Crawling_Animation")
    anim_stack.SetLocalTimeSpan(anim_time_span)
    anim_stack.SetReferenceTimeSpan(anim_time_span)
    anim_layer = fbx.FbxAnimLayer.Create(scene, "Base Layer")
    anim_stack.AddMember(anim_layer)

    # Get animation curves for root translation & rotation
    pelvis_node = fbx_nodes["pelvis"]
    root_tx = pelvis_node.LclTranslation.GetCurve(anim_layer, "X", True)
    root_ty = pelvis_node.LclTranslation.GetCurve(anim_layer, "Y", True)
    root_tz = pelvis_node.LclTranslation.GetCurve(anim_layer, "Z", True)

    root_rx = pelvis_node.LclRotation.GetCurve(anim_layer, "X", True)
    root_ry = pelvis_node.LclRotation.GetCurve(anim_layer, "Y", True)
    root_rz = pelvis_node.LclRotation.GetCurve(anim_layer, "Z", True)

    # Prepare joint animation data
    joint_curves = {}
    joint_final_deg = {}

    for name, info in nodes_def.items():
        if info.get("is_root"):
            continue
        node = fbx_nodes[name]
        axis = info["axis"]
        curve = node.LclRotation.GetCurve(anim_layer, axis, True)
        joint_curves[name] = curve

        col_name = info["csv_col"]
        raw_rad = np.array([float(row[col_name]) for row in rows], dtype=np.float64)
        if unwrap:
            # Unwrap 1-DOF joint angles to avoid continuous 180 deg flips
            processed_rad = np.unwrap(raw_rad)
        else:
            processed_rad = raw_rad
        joint_final_deg[name] = np.degrees(processed_rad)

    # Keyframe all animation curves across all frames
    time_obj = fbx.FbxTime()

    print("Keyframing FBX animation curves across all frames...")
    for frame_idx in range(num_frames):
        time_obj.SetSecondDouble(frame_idx / fps)

        # 1. Root Position
        px, py, pz = root_pos[frame_idx]
        add_keyframe(root_tx, time_obj, px)
        add_keyframe(root_ty, time_obj, py)
        add_keyframe(root_tz, time_obj, pz)

        # 2. Root Rotation (Euler XYZ in degrees)
        rx_deg, ry_deg, rz_deg = root_eulers_final[frame_idx]
        add_keyframe(root_rx, time_obj, rx_deg)
        add_keyframe(root_ry, time_obj, ry_deg)
        add_keyframe(root_rz, time_obj, rz_deg)

        # 3. Joint DOFs (Degrees)
        for name, curve in joint_curves.items():
            val_deg = joint_final_deg[name][frame_idx]
            add_keyframe(curve, time_obj, val_deg)

    print(f"Exporting FBX file to: {fbx_path}")
    os.makedirs(os.path.dirname(os.path.abspath(fbx_path)), exist_ok=True)
    exporter = fbx.FbxExporter.Create(manager, "")
    ios = manager.GetIOSettings()
    
    # Select Binary FBX format if available
    format_idx = -1
    for i in range(manager.GetIOPluginRegistry().GetWriterFormatCount()):
        if manager.GetIOPluginRegistry().WriterIsFBX(i):
            desc = manager.GetIOPluginRegistry().GetWriterFormatDescription(i)
            if "binary" in desc.lower():
                format_idx = i
                break

    if not exporter.Initialize(fbx_path, format_idx, ios):
        print(f"FBX Exporter Initialization Error: {exporter.GetStatus().GetErrorString()}")
        sys.exit(1)

    exporter.Export(scene)
    exporter.Destroy()
    manager.Destroy()
    print("FBX conversion completed successfully!")


_AXIS_VECTORS = {"X": (1.0, 0.0, 0.0), "Y": (0.0, 1.0, 0.0), "Z": (0.0, 0.0, 1.0)}
# Uppercase = extrinsic (world-fixed axes), matching how Blender's BVH importer
# (and BVH readers generally) actually interprets a "CHANNELS 3 Zrotation
# Xrotation Yrotation" declaration — verified empirically against Blender 5.1,
# NOT the intrinsic composition its axis-name order would suggest.
_BVH_ROTATION_ORDER = "ZXY"

# This rig is authored Z-up (matching the FBX export's axis system), but BVH
# carries no up-axis metadata and every mainstream reader (Blender included)
# assumes the traditional mocap convention of Y-up/-Z-forward by default. This
# fixed rotation converts every position and rotation into that convention at
# write time, so a plain default/drag-and-drop BVH import lands right-side up
# without the user having to override axis-up/axis-forward import options.
#
# It's composed of the Z-up -> Y-up swap (x, y, z) -> (x, z, -y) followed by a
# -90 deg yaw about the rig's own native Z axis, which corrects a residual 90
# deg mismatch (verified empirically in Blender 5.1) between this conversion
# and the axis system Blender's FBX importer derives from this rig's FBX
# export (FbxAxisSystem Z-up/eParityEven) — without the extra yaw, BVH and FBX
# imports of the same motion end up rotated 90 deg apart from each other.
_BVH_AXIS_ROTATION = R.from_matrix([
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0],
    [1.0, 0.0, 0.0],
])


def _to_bvh_position(vec):
    # Works for a single (3,) vector or a batched (num_frames, 3) array —
    # scipy's Rotation.apply broadcasts over either shape.
    return _BVH_AXIS_ROTATION.apply(vec)


def _wrap_deg(angle_deg):
    """Wraps degrees to [-180, 180)."""
    return (angle_deg + 180.0) % 360.0 - 180.0


def _continuous_euler(rotation, seq, unwrap):
    """
    Decomposes a batch Rotation into a 3-axis Euler sequence, frame by frame,
    resolving the gimbal-lock ambiguity instead of leaving it in the output.

    Any 3-distinct-axis (Tait-Bryan) Euler sequence has two equally valid
    solutions for the same rotation: (a1, a2, a3) and its "flipped" twin
    (a1+180, 180-a2, a3+180). Ordinarily this is harmless, but as a2 (the
    *middle* axis) approaches +/-90 deg, the decomposition becomes numerically
    unstable and can jump between the two solutions from one frame to the
    next even though the underlying rotation is changing smoothly — this
    shows up as a spurious ~180 deg "twist" in the outer two channels.
    `np.unwrap` cannot fix this: it only corrects a single channel's own
    +/-360 deg wraps, not a rotation redistributed *between* two channels.

    At each frame, this picks whichever of the two solutions stays closest to
    the previous frame's chosen values, which keeps the whole sequence
    continuous through gimbal lock. `unwrap` still runs afterward to also
    remove ordinary +/-360 deg wraps (e.g. a yaw angle spinning past +/-180).
    """
    euler_deg = rotation.as_euler(seq, degrees=True)
    alt_deg = np.empty_like(euler_deg)
    alt_deg[:, 0] = euler_deg[:, 0] + 180.0
    alt_deg[:, 1] = 180.0 - euler_deg[:, 1]
    alt_deg[:, 2] = euler_deg[:, 2] + 180.0

    chosen = euler_deg.copy()
    for i in range(1, len(chosen)):
        prev = chosen[i - 1]
        primary_dist = np.sum(_wrap_deg(euler_deg[i] - prev) ** 2)
        alt_dist = np.sum(_wrap_deg(alt_deg[i] - prev) ** 2)
        if alt_dist < primary_dist:
            chosen[i] = alt_deg[i]

    if unwrap:
        chosen = np.degrees(np.unwrap(np.radians(chosen), axis=0))
    return chosen


def _static_rotation_from_rpy(rpy):
    """
    Builds the constant tilt rotation from a URDF joint's <origin rpy="...">
    (radians), composed in a fixed X, then Y, then Z order — matching the same
    convention used to verify this rig's forward kinematics against the URDF.
    """
    rx, ry, rz = rpy
    rotation = R.identity()
    if rx:
        rotation = rotation * R.from_rotvec([rx, 0.0, 0.0])
    if ry:
        rotation = rotation * R.from_rotvec([0.0, ry, 0.0])
    if rz:
        rotation = rotation * R.from_rotvec([0.0, 0.0, rz])
    return rotation


def _to_bvh_euler(rotation, unwrap):
    """
    Conjugates a local-rotation (expressed in this rig's native Z-up frame) into
    the BVH file's Y-up frame, then decomposes it to Zrotation/Xrotation/Yrotation
    degrees (a batch Rotation in, (num_frames, 3) array out).
    """
    converted = _BVH_AXIS_ROTATION * rotation * _BVH_AXIS_ROTATION.inv()
    return _continuous_euler(converted, _BVH_ROTATION_ORDER, unwrap)


def compute_joint_euler_channels(nodes_def, rows, unwrap):
    """
    Returns {joint_name: (num_frames, 3) array of Zrotation/Xrotation/Yrotation
    degrees}, for every non-root joint. BVH has no equivalent of FBX's
    PreRotation, so joints with a static URDF tilt (hip_roll, knee,
    shoulder_pitch, shoulder_roll) have that tilt composed directly into the
    per-frame rotation here before conversion to BVH's axis convention.
    """
    channels = {}
    for name, info in nodes_def.items():
        if info.get("is_root"):
            continue

        axis = info["axis"]
        raw_rad = np.array([float(row[info["csv_col"]]) for row in rows], dtype=np.float64)
        if unwrap:
            raw_rad = np.unwrap(raw_rad)

        dof_rotation = R.from_rotvec(np.outer(raw_rad, _AXIS_VECTORS[axis]))
        rpy = info.get("rpy")
        local_rotation = _static_rotation_from_rpy(rpy) * dof_rotation if rpy and any(rpy) else dof_rotation

        channels[name] = _to_bvh_euler(local_rotation, unwrap)

    return channels


def _write_bvh_joint(lines, name, nodes_def, children_by_parent, joint_order, depth):
    info = nodes_def[name]
    pad = "  " * depth
    is_root = bool(info.get("is_root"))
    joint_order.append(name)

    lines.append(f"{pad}{'ROOT' if is_root else 'JOINT'} {name}")
    lines.append(f"{pad}{{")
    tx, ty, tz = _to_bvh_position(info["translation"])
    lines.append(f"{pad}  OFFSET {tx:.6f} {ty:.6f} {tz:.6f}")
    if is_root:
        lines.append(f"{pad}  CHANNELS 6 Xposition Yposition Zposition Zrotation Xrotation Yrotation")
    else:
        lines.append(f"{pad}  CHANNELS 3 Zrotation Xrotation Yrotation")

    children = children_by_parent.get(name, [])
    if children:
        for child in children:
            _write_bvh_joint(lines, child, nodes_def, children_by_parent, joint_order, depth + 1)
    else:
        # A zero-offset End Site gives this bone zero length, which leaves its
        # rest-pose direction undefined — Blender (and other tools) then fall
        # back to some arbitrary default (observed: pointing straight up)
        # instead of the joint's actual forward direction. Every leaf here
        # (ankle_roll, wrist_yaw) extends from its parent chain along local
        # +X, same as the rest of that chain's own translations, so a small
        # nominal tip length in that direction gives a sane rest orientation
        # without affecting any animated joint's position.
        ex, ey, ez = _to_bvh_position((0.08, 0.0, 0.0))
        lines.append(f"{pad}  End Site")
        lines.append(f"{pad}  {{")
        lines.append(f"{pad}    OFFSET {ex:.6f} {ey:.6f} {ez:.6f}")
        lines.append(f"{pad}  }}")
    lines.append(f"{pad}}}")


def convert_csv_to_bvh(csv_path, bvh_path, fps=30.0, unwrap=True):
    """
    Exports the Unitree G1 skeleton animation as BVH: skeleton-only (BVH has
    no mesh support), with no dependency on the Autodesk FBX SDK.
    """
    print(f"Reading CSV dataset: {csv_path}")
    rows, root_pos, root_quats = load_motion_csv(csv_path)
    num_frames = len(rows)
    print(f"Loaded {num_frames} frames of motion data.")

    root_pos_bvh = _to_bvh_position(root_pos)
    root_eulers_deg = _to_bvh_euler(R.from_quat(root_quats), unwrap)

    nodes_def = get_g1_joint_definitions()
    joint_channels = compute_joint_euler_channels(nodes_def, rows, unwrap)

    children_by_parent = {}
    root_name = None
    for name, info in nodes_def.items():
        if info.get("is_root"):
            root_name = name
        else:
            children_by_parent.setdefault(info["parent"], []).append(name)

    lines = ["HIERARCHY"]
    joint_order = []
    _write_bvh_joint(lines, root_name, nodes_def, children_by_parent, joint_order, depth=0)

    lines.append("MOTION")
    lines.append(f"Frames: {num_frames}")
    lines.append(f"Frame Time: {1.0 / fps:.6f}")

    print("Writing BVH motion data across all frames...")
    for frame_idx in range(num_frames):
        values = []
        for name in joint_order:
            if name == root_name:
                px, py, pz = root_pos_bvh[frame_idx]
                rz, rx, ry = root_eulers_deg[frame_idx]
                values.extend([px, py, pz, rz, rx, ry])
            else:
                rz, rx, ry = joint_channels[name][frame_idx]
                values.extend([rz, rx, ry])
        lines.append(" ".join(f"{v:.6f}" for v in values))

    print(f"Exporting BVH file to: {bvh_path}")
    os.makedirs(os.path.dirname(os.path.abspath(bvh_path)), exist_ok=True)
    with open(bvh_path, "w", newline="\n") as f:
        f.write("\n".join(lines) + "\n")
    print("BVH conversion completed successfully!")


def main():
    parser = argparse.ArgumentParser(
        description="Convert Unitree G1 robot motion CSV to an FBX or BVH skeleton animation "
                    "(format is chosen from --output's file extension)."
    )
    parser.add_argument("--input", required=True, help="Path to input CSV file")
    parser.add_argument("--output", required=True, help="Path to output file; use a .fbx or .bvh extension to pick the format")
    parser.add_argument("--fps", type=float, default=30.0, help="Frames per second for animation")
    parser.add_argument("--unwrap", action="store_true", dest="unwrap", help="Enable continuous angle unwrapping (default)")
    parser.add_argument("--no-unwrap", action="store_false", dest="unwrap", help="Disable continuous angle unwrapping")
    parser.set_defaults(unwrap=True)
    parser.add_argument("--mesh", action="store_true", dest="add_mesh",
                         help="Attach visual meshes from --mesh-dir (default; FBX output only, ignored for BVH)")
    parser.add_argument("--no-mesh", action="store_false", dest="add_mesh", help="Export skeleton only, without meshes")
    parser.set_defaults(add_mesh=True)
    parser.add_argument("--urdf", default="g1_description/urdf/g1_29dof.urdf", help="Path to the G1 URDF (for mesh/material lookup)")
    parser.add_argument("--mesh-dir", default="g1_description/meshes", help="Directory containing the G1 visual mesh STL files")
    args = parser.parse_args()

    ext = os.path.splitext(args.output)[1].lower()
    if ext == ".bvh":
        convert_csv_to_bvh(args.input, args.output, args.fps, args.unwrap)
    elif ext == ".fbx":
        convert_csv_to_fbx(args.input, args.output, args.fps, args.unwrap, args.add_mesh, args.urdf, args.mesh_dir)
    else:
        parser.error(f"Unsupported output extension '{ext}' — use a path ending in .fbx or .bvh")


if __name__ == "__main__":
    main()
