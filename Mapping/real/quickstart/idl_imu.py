#!/usr/bin/env python3
"""`sensor_msgs/msg/Imu` の IDL 型を unitree_sdk2py に足す。

PC2 の `unitree_sdk2py` は `PointCloud2_` と `Odometry_` は持っているが
**`Imu_` を持っていない**（2026-09-04 実測。`unitree_sdk2py/idl/sensor_msgs/
msg/dds_/__init__.py` は PointCloud2_ / PointField_ のみを公開している）。

一方 DDS 側では `rt/utlidar/imu_livox_mid360` が
`sensor_msgs::msg::dds_::Imu_` として**実際に配信されている**（builtin discovery
で確認済み）。足りないのは購読側の Python 型だけなので、ここで定義して
SDK のモジュールに注入する。

FAST-LIO2 は LiDAR-Inertial で IMU が無いと動かない。ここが通らないと
オフライン再構成のパイプライン全体が止まる。

使い方（`unitree_sdk2py.idl...` を触る前に import するだけ）:

    import idl_imu  # noqa: F401  Imu_ を SDK モジュールへ注入する
    from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import Imu_

SDK のファイルは書き換えない。実行時に足すだけなので、SDK を再クローンしても
壊れないし、こちらのリポジトリだけで完結する。
"""
import importlib
import sys
from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
import cyclonedds.idl.types as types

_SENSOR_MSGS = "unitree_sdk2py.idl.sensor_msgs.msg.dds_"

# 依存する型（Header_ / Quaternion_ / Vector3_）は SDK に既にある。
# cyclonedds は前方参照を完全修飾名の文字列で解決するので、先に読み込んでおく。
importlib.import_module("unitree_sdk2py.idl.std_msgs.msg.dds_")
importlib.import_module("unitree_sdk2py.idl.geometry_msgs.msg.dds_")


@dataclass
@annotate.final
@annotate.autoid("sequential")
class Imu_(idl.IdlStruct, typename="sensor_msgs.msg.dds_.Imu_"):
    """ROS 2 の `sensor_msgs/msg/Imu` そのまま。

    フィールド順・型は ROS 2 の定義に一致させること。1つでもずれると CDR の
    バイト位置が狂い、**エラーにならずに値だけ壊れる**。
    """
    header: 'unitree_sdk2py.idl.std_msgs.msg.dds_.Header_'
    orientation: 'unitree_sdk2py.idl.geometry_msgs.msg.dds_.Quaternion_'
    orientation_covariance: types.array[types.float64, 9]
    angular_velocity: 'unitree_sdk2py.idl.geometry_msgs.msg.dds_.Vector3_'
    angular_velocity_covariance: types.array[types.float64, 9]
    linear_acceleration: 'unitree_sdk2py.idl.geometry_msgs.msg.dds_.Vector3_'
    linear_acceleration_covariance: types.array[types.float64, 9]


def install():
    """SDK 側に `Imu_` が無ければ注入する。既にあるならそちらを優先する。

    将来 SDK が `Imu_` を持つようになったら、こちらの定義は使わない。
    """
    module = importlib.import_module(_SENSOR_MSGS)
    existing = getattr(module, "Imu_", None)
    if existing is not None:
        return existing
    setattr(module, "Imu_", Imu_)
    # `from ... import Imu_` を通すために __all__ にも足す
    all_names = getattr(module, "__all__", None)
    if isinstance(all_names, list) and "Imu_" not in all_names:
        all_names.append("Imu_")
    # 完全修飾名での前方参照解決に備えて、モジュールの位置も揃えておく
    sys.modules[__name__].Imu_ = Imu_
    return Imu_


install()
