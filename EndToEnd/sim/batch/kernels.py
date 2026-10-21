"""物理周期のIMUと時刻付きLivox rayをGPU上でまとめる。"""

import warp as wp


@wp.kernel(enable_backward=False)
def ray_distances(
    mesh: wp.uint64,
    origins: wp.array(dtype=wp.vec3f),
    directions: wp.array(dtype=wp.vec3f),
    distances: wp.array(dtype=wp.float32),
):
    i = wp.tid()
    t, u, v, sign = float(0.0), float(0.0), float(0.0), float(0.0)
    normal, face = wp.vec3f(), int(0)
    hit = wp.mesh_query_ray(mesh, origins[i], directions[i], 100.0, t, u, v, sign, normal, face)
    distances[i] = wp.where(hit, t, -1.0)


@wp.kernel(enable_backward=False)
def capture_inertial(
    positions: wp.array2d(dtype=wp.vec3f),
    quaternions: wp.array2d(dtype=wp.quatf),
    com_positions: wp.array2d(dtype=wp.vec3f),
    accelerations: wp.array2d(dtype=wp.vec3f),
    velocities: wp.array2d(dtype=wp.vec3f),
    angular_accelerations: wp.array2d(dtype=wp.vec3f),
    origins: wp.array(dtype=wp.vec3f),
    parents: wp.array(dtype=wp.int32),
    mount_positions: wp.array(dtype=wp.vec3f),
    mount_quats: wp.array(dtype=wp.quatf),
    pelvis: int,
    lidar_parent: int,
    lidar_position: wp.vec3f,
    lidar_quat: wp.quatf,
    gravity: wp.vec3f,
    tick: int,
    imu: wp.array4d(dtype=wp.float32),
    poses: wp.array3d(dtype=wp.transformf),
):
    env, sensor = wp.tid()
    body = parents[sensor]
    sensor_q = quaternions[env, body] * mount_quats[sensor]
    sensor_p = positions[env, body] + wp.quat_rotate(quaternions[env, body], mount_positions[sensor])
    lever = sensor_p - com_positions[env, body]
    omega = velocities[env, body]
    accel = accelerations[env, body] + wp.cross(angular_accelerations[env, body], lever)
    accel = accel + wp.cross(omega, wp.cross(omega, lever))
    gyro = wp.quat_rotate_inv(sensor_q, omega)
    force = wp.quat_rotate_inv(sensor_q, accel - gravity)
    projected = wp.quat_rotate_inv(sensor_q, wp.normalize(gravity))
    root = wp.transform(positions[env, pelvis] - origins[env], quaternions[env, pelvis])
    world = wp.transform(sensor_p - origins[env], sensor_q)
    base = wp.transform_multiply(wp.transform_inverse(root), world)
    world_matrix = wp.transform_to_matrix(world)
    base_matrix = wp.transform_to_matrix(base)
    for j in range(4):
        imu[tick, env, sensor, j] = sensor_q[j]
    for j in range(3):
        imu[tick, env, sensor, 4 + j] = gyro[j]
        imu[tick, env, sensor, 7 + j] = force[j]
        imu[tick, env, sensor, 10 + j] = projected[j]
    for r in range(4):
        for c in range(4):
            imu[tick, env, sensor, 13 + 4 * r + c] = world_matrix[r, c]
            imu[tick, env, sensor, 29 + 4 * r + c] = base_matrix[r, c]
    if sensor == 0:
        q = quaternions[env, lidar_parent] * lidar_quat
        p = (
            positions[env, lidar_parent]
            - origins[env]
            + wp.quat_rotate(quaternions[env, lidar_parent], lidar_position)
        )
        poses[tick, env, 0] = wp.transform(p, q)
        poses[tick, env, 1] = root


@wp.kernel(enable_backward=False)
def scan_livox(
    mesh: wp.uint64,
    poses: wp.array3d(dtype=wp.transformf),
    pattern: wp.array(dtype=wp.vec3f),
    start_index: int,
    point_rate: float,
    physics_dt: float,
    near: float,
    far: float,
    output: wp.array3d(dtype=wp.float32),
):
    env, point = wp.tid()
    step = wp.min(int(wp.ceil(float(point + 1) / (point_rate * physics_dt) - 1.0e-5)) - 1, poses.shape[0] - 2)
    fraction = wp.clamp((float(point + 1) / point_rate - float(step) * physics_dt) / physics_dt, 0.0, 1.0)
    a, b = poses[step, env, 0], poses[step + 1, env, 0]
    origin = wp.lerp(wp.transform_get_translation(a), wp.transform_get_translation(b), fraction)
    q = wp.quat_slerp(wp.transform_get_rotation(a), wp.transform_get_rotation(b), fraction)
    direction = wp.quat_rotate(q, pattern[(start_index + point) % pattern.shape[0]])
    t, u, v, sign = float(0.0), float(0.0), float(0.0), float(0.0)
    normal, face = wp.vec3f(), int(0)
    hit = wp.mesh_query_ray(mesh, origin, direction, far, t, u, v, sign, normal, face)
    valid = hit and t >= near
    distance = float(0.0)
    world_point = wp.vec3f()
    if valid:
        distance = t
        world_point = origin + direction * t
    output[env, point, 0] = distance
    output[env, point, 1] = float(valid)
    for j in range(3):
        output[env, point, 2 + j] = world_point[j]
        output[env, point, 5 + j] = origin[j]
    for j in range(4):
        output[env, point, 8 + j] = q[j]
