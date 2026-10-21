"""GPU用sensor kernelを同じWarp CPU backendでも数学的な参照実装と照合する。"""

import numpy as np
import pytest

wp = pytest.importorskip("warp")

from sim.batch.kernels import capture_inertial, scan_livox  # noqa: E402


def test_lidar_points_use_their_own_interpolated_measurement_pose():
    vertices = wp.array(
        [[-20, -20, 0], [20, -20, 0], [20, 20, 0], [-20, 20, 0]], dtype=wp.vec3f, device="cpu"
    )
    mesh = wp.Mesh(vertices, wp.array([0, 1, 2, 0, 2, 3], dtype=wp.int32, device="cpu"))
    poses = np.zeros((3, 2, 2, 7), np.float32)
    poses[..., 2], poses[..., 6] = 1, 1
    poses[:, :, 0, 0] = np.arange(3)[:, None] * 0.1 + np.arange(2)[None] * 2
    pattern = np.tile([0, 0, -1], (20, 1)).astype(np.float32)
    output = wp.empty((2, 20, 12), dtype=wp.float32, device="cpu")
    wp.launch(
        scan_livox,
        dim=(2, 20),
        inputs=[
            mesh.id,
            wp.array(poses, dtype=wp.transformf, device="cpu"),
            wp.array(pattern, dtype=wp.vec3f, device="cpu"),
            0,
            2000.0,
            0.005,
            0.1,
            30.0,
            output,
        ],
        device="cpu",
    )
    data = output.numpy()
    np.testing.assert_allclose(data[..., 0], 1, atol=1e-6)
    np.testing.assert_allclose(data[..., 1], 1)
    np.testing.assert_allclose(
        data[..., 5], np.arange(1, 21)[None] * 0.01 + np.arange(2)[:, None] * 2, atol=1e-6
    )
    np.testing.assert_allclose(data[..., 4], 0, atol=1e-6)


def test_imu_kernel_matches_lever_arm_math_and_removes_clone_offsets():
    from scipy.spatial.transform import Rotation
    from g1_e2e.data.geometry import pose_matrix
    from g1_e2e.data.imu import measure_imu

    rng = np.random.default_rng(42)
    origins = np.array([[100, -200, 0], [-500, 500, 0]], np.float32)
    positions = rng.normal(size=(2, 2, 3)).astype(np.float32) + origins[:, None]
    quats = Rotation.random(4, random_state=rng).as_quat().astype(np.float32).reshape(2, 2, 4)
    com_positions = positions + rng.normal(size=positions.shape).astype(np.float32) * 0.05
    acceleration, omega, alpha = [rng.normal(size=positions.shape).astype(np.float32) for _ in range(3)]
    mount_p = np.array([[0.1, -0.2, 0.3], [-0.2, 0.4, -0.1]], np.float32)
    mount_q = Rotation.random(2, random_state=rng).as_quat().astype(np.float32)
    output = wp.empty((1, 2, 2, 45), dtype=wp.float32, device="cpu")
    poses = wp.empty((1, 2, 2), dtype=wp.transformf, device="cpu")
    arrays = [
        wp.array(a, dtype=d, device="cpu")
        for a, d in (
            (positions, wp.vec3f),
            (quats, wp.quatf),
            (com_positions, wp.vec3f),
            (acceleration, wp.vec3f),
            (omega, wp.vec3f),
            (alpha, wp.vec3f),
        )
    ]
    wp.launch(
        capture_inertial,
        dim=(2, 2),
        inputs=[
            *arrays,
            wp.array(origins, dtype=wp.vec3f, device="cpu"),
            wp.array([0, 1], dtype=wp.int32, device="cpu"),
            wp.array(mount_p, dtype=wp.vec3f, device="cpu"),
            wp.array(mount_q, dtype=wp.quatf, device="cpu"),
            0,
            1,
            wp.vec3f(0.0),
            wp.quatf(0, 0, 0, 1),
            wp.vec3f(0, 0, -9.81),
            0,
            output,
            poses,
        ],
        device="cpu",
    )
    data = output.numpy()[0]
    for env in range(2):
        for sensor in range(2):
            expected = measure_imu(
                episode_id=0,
                time_s=0,
                world_from_body=pose_matrix(positions[env, sensor], quats[env, sensor]),
                body_from_sensor=pose_matrix(mount_p[sensor], mount_q[sensor]),
                com_position_world_m=com_positions[env, sensor],
                com_acceleration_world_m_s2=acceleration[env, sensor],
                angular_velocity_world_rad_s=omega[env, sensor],
                angular_acceleration_world_rad_s2=alpha[env, sensor],
                gravity_world_m_s2=np.array([0, 0, -9.81]),
                world_from_base=pose_matrix(positions[env, 0], quats[env, 0]),
            )
            expected.world_from_sensor[:3, 3] -= origins[env]
            np.testing.assert_allclose(
                data[env, sensor, 4:7], expected.angular_velocity_sensor_rad_s, atol=1e-5
            )
            np.testing.assert_allclose(
                data[env, sensor, 7:10], expected.specific_force_sensor_m_s2, atol=2e-4
            )
            np.testing.assert_allclose(data[env, sensor, 10:13], expected.projected_gravity_sensor, atol=1e-5)
            np.testing.assert_allclose(
                data[env, sensor, 13:29].reshape(4, 4), expected.world_from_sensor, atol=1e-4
            )
            np.testing.assert_allclose(
                data[env, sensor, 29:45].reshape(4, 4), expected.base_from_sensor, atol=1e-4
            )
