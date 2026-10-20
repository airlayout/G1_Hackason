"""実機仕様書と、実際の収集設定の差を隠さずmetadataへ残す。"""


def sensor_fidelity(sensors, environment):
    return {
        "reference": "G1_IO_SPECIFICATION.md",
        "hardware_equivalence_verified": False,
        "rgb": {
            "reference_sections": ["3.1", "Appendix B"],
            "simulation_resolution_wh": [sensors.camera.width, sensors.camera.height],
            "simulation_rate_hz": 1 / sensors.camera.period_s,
            "reference_calibration_profile_rate_hz": 30,
            "reference_profile_rate_status": "enumerated capability; simultaneous real acquisition not measured",
            "intrinsics_source": sensors.camera.pinhole.source if sensors.camera.pinhole else "synthetic",
            "intrinsics": sensors.camera.intrinsics.tolist(),
            "mount": sensors.camera.mount.urdf_joint or "manual",
            "nominal_mount_parent": sensors.camera.mount.body,
            "nominal_mount_position_m": list(sensors.camera.mount.position_m),
            "nominal_mount_rpy_deg": list(sensors.camera.mount.rpy_deg),
            "fov_deg_horizontal_vertical": list(sensors.camera.fov_deg),
            "reference_distortion_coefficients": [0, 0, 0, 0, 0],
            "matched": ["RGB channel order", "pixel intrinsics when pinhole calibration is selected"],
            "unverified": [
                "individual base-to-camera extrinsics and RGB optical origin relative to d435_link",
                "real exposure, gain, ISP response, lens/rolling-shutter behavior, noise and transport delay",
            ],
            "rate_matches_reference_calibration_profile": abs(1 / sensors.camera.period_s - 30) < 1e-6,
            "timing_note": "simulation sampling is configured independently; real acquisition timing is unmeasured",
        },
        "depth": {
            "reference_sections": ["3.1", "3.2", "Appendix B"],
            "reference_model": "active stereo; native Z16, measured depth scale 0.0010000000474974513 m/unit",
            "reference_native_intrinsics_640x480": [
                [384.808441, 0, 325.35614],
                [0, 384.808441, 242.072098],
                [0, 0, 1],
            ],
            "simulation_model": "separate native-depth pinhole, Z16 quantization, calibrated depth-to-color projection; native depth Z as in RealSense align"
            if sensors.camera.depth
            else "ideal ray-traced float32 Z on RGB optical grid",
            "native_geometry_and_z16_enabled": sensors.camera.depth is not None,
            "simulation_clip_limits_m": [sensors.camera.near_m, sensors.camera.far_m],
            "reference_typical_operating_range_m": [0.3, 3.0],
            "reference_range_note": "typical working range, not a hard near/far cutoff",
            "mismatches": [
                "no active-stereo matching, texture-dependent failures or IR emitter",
                "pixel validity is renderer visibility, not physical stereo confidence",
                "alignment follows generic librealsense pixel-corner rectangles and minimum native Z16; optional filtering unmodeled",
            ],
        },
        "body_imu": {
            "reference_sections": ["5.3", "15"],
            "simulation_source": "URDF-mounted pelvis and torso IMUs; rigid-body lever-arm acceleration",
            "simulation_rate_hz": 1 / environment.physics_dt,
            "model_input_rate_hz": 1 / environment.control_dt,
            "mount_joints": {name: mount.urdf_joint for name, mount in sensors.body_imus.mounts.items()},
            "primary": sensors.body_imus.primary,
            "mapping_source": sensors.body_imus.mapping_source,
            "matched": [
                "URDF mount positions and rotations",
                "centripetal and angular acceleration at sensor position",
                "gyro rad/s",
                "specific force m/s^2",
                "xyzw internal / wxyz SDK conversion",
            ],
            "unverified": [
                "individual mounting calibration and firmware yaw alignment; nominal mapping follows official simulator",
                "hardware bias, noise spectrum, bandwidth, filtering, sampling latency and temperature drift",
            ],
            "mismatch": "URDF geometry and simulation sampling do not reproduce electronic transfer functions",
        },
        "joints": {
            "reference_sections": ["5.1", "5.2", "10.1", "Appendix A"],
            "simulation_body_joints": 29,
            "simulation_rate_hz": 1 / environment.control_dt,
            "reference_lowstate_observed_hz": [955.92, 1035.5],
            "matched": ["SDK index order", "PR coordinates", "rad", "rad/s", "N m"],
            "mismatches": [
                "PhysX q/dq are simulated states, not noisy encoders",
                "applied_torque is an implicit PD estimate, not calibrated motor tau_est",
                "motor compliance, friction, backlash, actuator delays, thermal and electrical behavior uncalibrated",
                "controller recording rate differs from real LowState receive rate",
            ],
        },
        "lidar": {
            "reference_sections": ["4.1", "4.2"],
            "simulation_model": sensors.lidar.model,
            "simulation_frame_rate_hz": 1 / sensors.lidar.period_s,
            "simulation_range_limits_m": [sensors.lidar.near_m, sensors.lidar.far_m],
            "mount": sensors.lidar.mount.urdf_joint or "manual",
            "matched_for_livox_csv": [
                "manufacturer public simulation beam angle sequence",
                "200000 first-return rays/s derived from the published specification",
                "per-point acquisition times and interpolated sensor motion within physics steps",
            ],
            "unverified_or_unmodeled": [
                "CSV does not include captured timestamps, emitter/ring IDs or individual device calibration",
                "angle table has finite duration and small deviations from nominal FOV",
                "physical beam divergence, material reflectance, range-dependent noise and dropouts",
                "self occlusion and dynamic geometry",
                "UDP packet boundaries, firmware buffering, receive jitter and time-offset header convention",
            ],
            "deskew": "simulation truth is saved separately; deployment requires causal estimated sensor poses",
        },
        "additional_imus_not_simulated": [
            "D435i accelerometer 100/200Hz and gyro 200/400Hz",
            "MID-360 ICM40609 at nominal 200Hz; real route may retain g units",
        ],
        "requires_for_device_equivalence": [
            "synchronized raw real-device RGB/depth/IMU/LowState/point-cloud logs with original timestamps",
            "individual sensor-to-body and optical-frame calibration",
            "measured noise, latency, dropout and actuator response under representative motion/materials",
            "held-out real-device evaluation",
        ],
    }
