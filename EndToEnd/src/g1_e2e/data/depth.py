"""native Z16を、校正済みdepth→color変換でRGB画素格子へ投影する。"""

import numpy as np


def quantize_depth(depth_m, valid, scale_m):
    units = np.rint(np.where(valid, depth_m, 0) / scale_m)
    valid = valid & np.isfinite(units) & (units >= 1) & (units <= np.iinfo(np.uint16).max)
    return np.where(valid, units, 0).astype(np.uint16), valid


def align_depth(z16, valid, *, depth_intrinsics, color_intrinsics, color_from_depth, scale_m, color_shape):
    """librealsense generic align方式の画素隅投影・矩形転送・最小Z16。

    出力値はSDKと同じnative depthのZをmへ換算する。RGB光学系のZではない。
    空間穴埋めやステレオ信頼度を追加しない。
    """
    v, u = np.nonzero(valid)
    z = z16[v, u].astype(np.float64) * scale_m
    k = depth_intrinsics

    def project_corner(offset):
        points = np.column_stack(
            ((u + offset - k[0, 2]) * z / k[0, 0], (v + offset - k[1, 2]) * z / k[1, 1], z)
        )
        points = points @ color_from_depth[:3, :3].T + color_from_depth[:3, 3]
        good = np.isfinite(points).all(axis=1) & (points[:, 2] > 0)
        safe_z = np.where(good, points[:, 2], 1)
        kc = color_intrinsics
        px = np.trunc(kc[0, 0] * points[:, 0] / safe_z + kc[0, 2] + 0.5).astype(np.int64)
        py = np.trunc(kc[1, 1] * points[:, 1] / safe_z + kc[1, 2] + 0.5).astype(np.int64)
        return px, py, good

    x0, y0, good0 = project_corner(-0.5)
    x1, y1, good1 = project_corner(0.5)
    height, width = color_shape
    inside = good0 & good1 & (x0 >= 0) & (y0 >= 0) & (x1 < width) & (y1 < height) & (x1 >= x0) & (y1 >= y0)
    values = z16[v[inside], u[inside]].astype(np.int32)
    x0, y0, x1, y1 = (a[inside] for a in [x0, y0, x1, y1])
    target = np.full(height * width, 65536, np.int32)
    if len(values):
        for dy in range(int((y1 - y0).max()) + 1):
            for dx in range(int((x1 - x0).max()) + 1):
                mask = (x0 + dx <= x1) & (y0 + dy <= y1)
                np.minimum.at(target, (y0[mask] + dy) * width + x0[mask] + dx, values[mask])
    target = target.reshape(height, width)
    valid_color = target < 65536
    return np.where(valid_color, target * scale_m, 0).astype(np.float32), valid_color
