"""レイ単位の Gaussian 合成が、手で解ける形と一致するかを確かめる。

ここが狂っていると「3DGS を当てた結果」が丸ごと無意味になるので、
学習を回す前に解析解と突き合わせておく。
"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from g1_mapping.lidar_gs import (  # noqa: E402
    RenderConfig,
    associate,
    init_from_points,
    precision_matrices,
    query_density,
    read_splat_ply,
    render_rays,
    to_parameters,
    write_splat_ply,
)

DEVICE = "cpu"


def make_params(means, scales, quats, opacities):
    arrays = {
        "means": np.asarray(means, dtype=np.float32),
        "scales": np.log(np.asarray(scales, dtype=np.float32)),
        "quats": np.asarray(quats, dtype=np.float32),
        "opacities": np.log(np.asarray(opacities, dtype=np.float32)
                            / (1 - np.asarray(opacities, dtype=np.float32))),
    }
    return to_parameters(arrays, DEVICE)


class PrecisionTest(unittest.TestCase):
    def test_isotropic_precision_is_inverse_scale(self):
        params = make_params([[0, 0, 0]], [[0.5, 0.5, 0.5]], [[1, 0, 0, 0]], [0.5])
        m = precision_matrices(params["quats"], params["scales"])[0]
        np.testing.assert_allclose(m.detach().numpy(), np.eye(3) * 2.0, atol=1e-6)

    def test_precision_reproduces_covariance_inverse(self):
        """MᵀM が Σ⁻¹ になっていること。任意の姿勢・スケールで。"""
        torch.manual_seed(0)
        quats = torch.randn(5, 4)
        log_scales = torch.randn(5, 3) * 0.3
        m = precision_matrices(quats, log_scales)
        from gsplat.utils import normalized_quat_to_rotmat
        import torch.nn.functional as F
        rotation = normalized_quat_to_rotmat(F.normalize(quats, dim=-1))
        scale = torch.diag_embed(torch.exp(log_scales))
        covariance = rotation @ scale @ scale @ rotation.transpose(1, 2)
        np.testing.assert_allclose(
            (m.transpose(1, 2) @ m).detach().numpy(),
            torch.linalg.inv(covariance).numpy(), atol=1e-4)


class SingleGaussianTest(unittest.TestCase):
    """1 個のガウシアンに 1 本の光線。答えが手で書ける場合。"""

    def test_ray_through_centre_returns_centre_distance(self):
        params = make_params([[3.0, 0.0, 0.0]], [[0.1, 0.1, 0.1]], [[1, 0, 0, 0]], [0.9])
        origin = torch.zeros(3)
        directions = torch.tensor([[1.0, 0.0, 0.0]])
        depth, acc, _ = render_rays(params, origin, directions, RenderConfig())
        # 中心を通るので t* = 3.0、q = 0 なので α = o = 0.9
        self.assertAlmostEqual(float(acc), 0.9, places=5)
        self.assertAlmostEqual(float(depth / acc), 3.0, places=4)

    def test_offset_ray_matches_analytic_mahalanobis(self):
        """光線が中心から d だけ外れていれば q = (d/σ)²。"""
        sigma, offset, opacity = 0.2, 0.3, 0.9
        params = make_params([[4.0, offset, 0.0]], [[sigma] * 3], [[1, 0, 0, 0]], [opacity])
        origin = torch.zeros(3)
        directions = torch.tensor([[1.0, 0.0, 0.0]])
        _, acc, _ = render_rays(params, origin, directions, RenderConfig())
        expected = opacity * math.exp(-0.5 * (offset / sigma) ** 2)
        self.assertAlmostEqual(float(acc), expected, places=4)

    def test_anisotropic_gaussian_uses_rotated_axis(self):
        """y 方向に細長いガウシアンは、y にずれた光線には強く応答する。"""
        params = make_params([[4.0, 0.0, 0.0]], [[0.05, 0.5, 0.05]], [[1, 0, 0, 0]], [0.9])
        origin = torch.zeros(3)
        along_y = torch.tensor([[4.0, 0.3, 0.0]])     # 長い軸の方向にずれる
        along_z = torch.tensor([[4.0, 0.0, 0.3]])     # 短い軸の方向にずれる
        _, acc_y, _ = render_rays(params, origin, along_y, RenderConfig())
        _, acc_z, _ = render_rays(params, origin, along_z, RenderConfig())
        self.assertGreater(float(acc_y), 0.5)
        self.assertLess(float(acc_z), 1e-3)


class CompositingTest(unittest.TestCase):
    def test_near_opaque_gaussian_hides_the_far_one(self):
        near, far = 0.99, 0.99
        params = make_params([[2.0, 0, 0], [6.0, 0, 0]], [[0.1] * 3] * 2,
                             [[1, 0, 0, 0]] * 2, [near, far])
        depth, acc, _ = render_rays(params, torch.zeros(3),
                                    torch.tensor([[1.0, 0.0, 0.0]]), RenderConfig())
        # 奥は透過率 1% ぶんしか効かないので、期待距離はほぼ手前の 2.0 m になる
        w_near, w_far = near, far * (1 - near)
        self.assertAlmostEqual(float(depth / acc),
                               (w_near * 2.0 + w_far * 6.0) / (w_near + w_far), places=4)
        self.assertLess(float(depth / acc), 2.05)

    def test_transparent_front_lets_the_back_through(self):
        """手前が薄いときは、期待距離が 2 つの重み付き平均になる。"""
        near_alpha, far_alpha = 0.5, 0.99
        params = make_params([[2.0, 0, 0], [6.0, 0, 0]], [[0.1] * 3] * 2,
                             [[1, 0, 0, 0]] * 2, [near_alpha, far_alpha])
        depth, acc, _ = render_rays(params, torch.zeros(3),
                                    torch.tensor([[1.0, 0.0, 0.0]]), RenderConfig())
        w_near = near_alpha
        w_far = far_alpha * (1 - near_alpha)
        self.assertAlmostEqual(float(acc), w_near + w_far, places=4)
        self.assertAlmostEqual(float(depth), w_near * 2.0 + w_far * 6.0, places=3)

    def test_order_does_not_depend_on_input_order(self):
        """入力の並び順を変えても同じ絵になること（手前から合成しているか）。"""
        forward = make_params([[2.0, 0, 0], [6.0, 0, 0]], [[0.1] * 3] * 2,
                              [[1, 0, 0, 0]] * 2, [0.5, 0.99])
        backward = make_params([[6.0, 0, 0], [2.0, 0, 0]], [[0.1] * 3] * 2,
                               [[1, 0, 0, 0]] * 2, [0.99, 0.5])
        ray = torch.tensor([[1.0, 0.0, 0.0]])
        d1, a1, _ = render_rays(forward, torch.zeros(3), ray, RenderConfig())
        d2, a2, _ = render_rays(backward, torch.zeros(3), ray, RenderConfig())
        self.assertAlmostEqual(float(d1), float(d2), places=5)
        self.assertAlmostEqual(float(a1), float(a2), places=5)


class AssociationTest(unittest.TestCase):
    def test_gaussian_on_the_ray_is_found(self):
        rng = np.random.default_rng(0)
        means = rng.uniform(-10, 10, size=(2000, 3)).astype(np.float32)
        params = make_params(means, np.full((2000, 3), 0.05),
                             np.tile([1, 0, 0, 0], (2000, 1)), np.full(2000, 0.5))
        origin = torch.zeros(3)
        directions = torch.from_numpy(means[:200]) - origin
        ids, valid = associate(params["means"], params["scales"], origin,
                               torch.nn.functional.normalize(directions, dim=-1),
                               RenderConfig())
        found = [(torch.tensor(i) == ids[i][valid[i]]).any().item() for i in range(200)]
        self.assertGreaterEqual(sum(found), 195, "自分の中心を通る光線から見つからない")

    def test_rays_in_all_directions_get_candidates(self):
        """真上・真下を含めて、極付近でも候補が拾えること。"""
        means = np.array([[0, 0, 5.0], [0, 0, -5.0], [5.0, 0, 0], [-5.0, 0, 0]],
                         dtype=np.float32)
        params = make_params(means, np.full((4, 3), 0.1),
                             np.tile([1, 0, 0, 0], (4, 1)), np.full(4, 0.9))
        depth, acc, _ = render_rays(params, torch.zeros(3),
                                    torch.from_numpy(means), RenderConfig())
        np.testing.assert_allclose(acc.detach().numpy(), 0.9, atol=1e-4)
        np.testing.assert_allclose((depth / acc).detach().numpy(), 5.0, atol=1e-3)


class GradientTest(unittest.TestCase):
    def test_opacity_drops_when_a_gaussian_blocks_a_longer_measurement(self):
        """奥に実測があるのに手前を塞いでいるガウシアンは、α を下げる勾配を受ける。

        これが追従者を消す仕組みそのものなので、符号を確かめておく。
        """
        params = make_params([[2.0, 0, 0], [6.0, 0, 0]], [[0.1] * 3] * 2,
                             [[1, 0, 0, 0]] * 2, [0.8, 0.8])
        ray = torch.tensor([[1.0, 0.0, 0.0]])
        depth, acc, _ = render_rays(params, torch.zeros(3), ray, RenderConfig())
        loss = ((depth / acc.clamp_min(1e-6)) - 6.0).abs().sum()
        loss.backward()
        blocker_grad = params["opacities"].grad[0].item()
        target_grad = params["opacities"].grad[1].item()
        self.assertGreater(blocker_grad, 0.0, "手前を塞ぐ側の不透明度が下がらない")
        self.assertLess(target_grad, 0.0, "奥の実測に合う側の不透明度が上がらない")


class InitTest(unittest.TestCase):
    def test_anisotropic_init_flattens_on_a_plane(self):
        """平面上の点なら、法線方向のスケールが一番小さくなること。"""
        rng = np.random.default_rng(1)
        xy = rng.uniform(-1, 1, size=(400, 2))
        points = np.column_stack([xy, np.zeros(len(xy))]).astype(np.float64)
        arrays = init_from_points(points, min_scale=1e-4, max_scale=1.0)
        scales = np.exp(arrays["scales"])
        # 3 軸のうち最小が、面内の 2 軸よりはっきり小さい
        smallest = scales.min(axis=1)
        largest = scales.max(axis=1)
        self.assertLess(float(np.median(smallest / largest)), 0.2)

    def test_quaternions_are_unit(self):
        rng = np.random.default_rng(2)
        points = rng.uniform(-1, 1, size=(200, 3))
        arrays = init_from_points(points)
        norms = np.linalg.norm(arrays["quats"], axis=1)
        np.testing.assert_allclose(norms, 1.0, atol=1e-5)


class ColourTest(unittest.TestCase):
    """反射強度を 1 チャンネルの「色」として合成できているか。"""

    @staticmethod
    def with_colours(means, scales, opacities, intensities):
        params = make_params(means, scales, [[1, 0, 0, 0]] * len(means), opacities)
        from g1_mapping.lidar_gs import INTENSITY_SCALE
        normalized = np.clip(np.asarray(intensities) / INTENSITY_SCALE, 1e-3, 1 - 1e-3)
        params["colors"] = torch.nn.Parameter(
            torch.from_numpy(np.log(normalized / (1 - normalized)).astype(np.float32)[:, None]))
        return params

    def test_single_gaussian_returns_alpha_times_colour(self):
        opacity, raw = 0.8, 128.0
        params = self.with_colours([[3.0, 0, 0]], [[0.1] * 3], [opacity], [raw])
        _, acc, _, colour = render_rays(params, torch.zeros(3),
                                        torch.tensor([[1.0, 0.0, 0.0]]),
                                        RenderConfig(), with_color=True)
        from g1_mapping.lidar_gs import INTENSITY_SCALE
        self.assertAlmostEqual(float(colour / acc) * INTENSITY_SCALE, raw, places=1)

    def test_opaque_front_hides_the_colour_behind(self):
        params = self.with_colours([[2.0, 0, 0], [6.0, 0, 0]], [[0.1] * 3] * 2,
                                   [0.99, 0.99], [200.0, 10.0])
        _, acc, _, colour = render_rays(params, torch.zeros(3),
                                        torch.tensor([[1.0, 0.0, 0.0]]),
                                        RenderConfig(), with_color=True)
        from g1_mapping.lidar_gs import INTENSITY_SCALE
        shown = float(colour / acc) * INTENSITY_SCALE
        self.assertGreater(shown, 190.0, "手前の色が出ていない")

    def test_transparent_front_blends_both(self):
        near, far = 0.5, 0.99
        params = self.with_colours([[2.0, 0, 0], [6.0, 0, 0]], [[0.1] * 3] * 2,
                                   [near, far], [200.0, 0.0])
        _, acc, _, colour = render_rays(params, torch.zeros(3),
                                        torch.tensor([[1.0, 0.0, 0.0]]),
                                        RenderConfig(), with_color=True)
        from g1_mapping.lidar_gs import INTENSITY_SCALE
        w_near, w_far = near, far * (1 - near)
        expected = (w_near * 200.0 / INTENSITY_SCALE) / (w_near + w_far)
        self.assertAlmostEqual(float(colour / acc), expected, places=2)

    def test_init_from_points_carries_intensity(self):
        rng = np.random.default_rng(7)
        points = rng.uniform(-1, 1, size=(50, 3))
        raw = rng.uniform(0, 155, size=50)
        arrays = init_from_points(points, intensities=raw)
        from g1_mapping.lidar_gs import INTENSITY_SCALE
        back = 1 / (1 + np.exp(-arrays["colors"][:, 0])) * INTENSITY_SCALE
        np.testing.assert_allclose(back, raw, atol=0.5)


class FreeSpaceTest(unittest.TestCase):
    """自由空間の罰則。OctoMap の「空」の票を微分可能にしたもの。"""

    def test_gaussian_in_front_of_the_measurement_is_penalised(self):
        params = make_params([[2.0, 0, 0]], [[0.1] * 3], [[1, 0, 0, 0]], [0.8])
        ray = torch.tensor([[1.0, 0.0, 0.0]])
        *_, penalty = render_rays(params, torch.zeros(3), ray, RenderConfig(),
                                  free_space=torch.tensor([6.0]))
        self.assertAlmostEqual(float(penalty), 0.8, places=4)

    def test_gaussian_at_the_measurement_is_not_penalised(self):
        params = make_params([[6.0, 0, 0]], [[0.1] * 3], [[1, 0, 0, 0]], [0.8])
        ray = torch.tensor([[1.0, 0.0, 0.0]])
        *_, penalty = render_rays(params, torch.zeros(3), ray, RenderConfig(),
                                  free_space=torch.tensor([6.0]))
        self.assertAlmostEqual(float(penalty), 0.0, places=6)

    def test_margin_protects_gaussians_just_before_the_surface(self):
        """実測より 5 cm 手前は、測定誤差の範囲なので罰しない。"""
        params = make_params([[5.95, 0, 0]], [[0.02] * 3], [[1, 0, 0, 0]], [0.8])
        ray = torch.tensor([[1.0, 0.0, 0.0]])
        *_, penalty = render_rays(params, torch.zeros(3), ray, RenderConfig(),
                                  free_space=torch.tensor([6.0]), free_space_margin=0.10)
        self.assertAlmostEqual(float(penalty), 0.0, places=6)

    def test_beyond_max_range_is_not_penalised(self):
        """OctoMap と同じ理由で、遠い光線には効かせない（床の斜入射対策）。"""
        params = make_params([[5.0, 0, 0]], [[0.1] * 3], [[1, 0, 0, 0]], [0.8])
        ray = torch.tensor([[1.0, 0.0, 0.0]])
        *_, near = render_rays(params, torch.zeros(3), ray, RenderConfig(),
                               free_space=torch.tensor([20.0]), free_space_max_range=3.0)
        *_, far = render_rays(params, torch.zeros(3), ray, RenderConfig(),
                              free_space=torch.tensor([20.0]), free_space_max_range=10.0)
        self.assertAlmostEqual(float(near), 0.0, places=6)
        self.assertAlmostEqual(float(far), 0.8, places=4)

    def test_penalty_pushes_opacity_down_regardless_of_what_is_in_front(self):
        """透過率が尽きていても勾配が届くこと。これが depth loss との違い。"""
        params = make_params([[1.0, 0, 0], [2.0, 0, 0], [3.0, 0, 0]], [[0.1] * 3] * 3,
                             [[1, 0, 0, 0]] * 3, [0.99, 0.99, 0.99])
        ray = torch.tensor([[1.0, 0.0, 0.0]])
        *_, penalty = render_rays(params, torch.zeros(3), ray, RenderConfig(),
                                  free_space=torch.tensor([8.0]),
                                  free_space_max_range=10.0)
        penalty.backward()
        grads = params["opacities"].grad
        for index in range(3):
            self.assertGreater(float(grads[index]), 0.0,
                               f"{index} 番目（手前から）に勾配が来ていない")
        # 一番奥のものにも、手前と同じ桁の勾配が来ている
        self.assertGreater(float(grads[2]) / float(grads[0]), 0.5)


class QueryDensityTest(unittest.TestCase):
    """地図の点を残すか消すかを決める問い合わせ。OctoMap の getLabels() に相当する。"""

    def test_point_on_a_gaussian_is_supported(self):
        params = make_params([[1.0, 0, 0]], [[0.05] * 3], [[1, 0, 0, 0]], [0.8])
        density = query_density(params, torch.tensor([[1.0, 0.0, 0.0]]), neighbors=1)
        self.assertAlmostEqual(float(density[0]), 0.8, places=5)

    def test_point_far_from_every_gaussian_is_empty(self):
        params = make_params([[1.0, 0, 0]], [[0.05] * 3], [[1, 0, 0, 0]], [0.8])
        density = query_density(params, torch.tensor([[5.0, 0.0, 0.0]]), neighbors=1)
        self.assertLess(float(density[0]), 1e-6)

    def test_density_falls_off_with_mahalanobis_distance(self):
        sigma, offset, opacity = 0.05, 0.075, 0.8
        params = make_params([[1.0, 0, 0]], [[sigma] * 3], [[1, 0, 0, 0]], [opacity])
        density = query_density(params, torch.tensor([[1.0, offset, 0.0]]), neighbors=1)
        expected = opacity * math.exp(-0.5 * (offset / sigma) ** 2)
        self.assertAlmostEqual(float(density[0]), expected, places=5)

    def test_takes_the_strongest_of_several_neighbours(self):
        """近くに薄いのが、少し遠くに濃いのがある場合、濃いほうが採られること。"""
        params = make_params([[1.0, 0.02, 0], [1.0, -0.04, 0]], [[0.05] * 3] * 2,
                             [[1, 0, 0, 0]] * 2, [0.1, 0.9])
        density = query_density(params, torch.tensor([[1.0, 0.0, 0.0]]), neighbors=2)
        weak = 0.1 * math.exp(-0.5 * (0.02 / 0.05) ** 2)
        strong = 0.9 * math.exp(-0.5 * (0.04 / 0.05) ** 2)
        self.assertAlmostEqual(float(density[0]), max(weak, strong), places=5)


class SplatPlyTest(unittest.TestCase):
    """3DGS の .ply。plyfile（GPLv3）を避けて自前で書いているので往復を確かめる。"""

    def test_round_trip_is_exact(self):
        import tempfile

        rng = np.random.default_rng(3)
        points = rng.uniform(-3, 3, size=(200, 3))
        params = to_parameters(init_from_points(points), DEVICE)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "g.ply"
            write_splat_ply(path, params)
            back = read_splat_ply(path)
            for key in ("means", "scales", "opacities"):
                np.testing.assert_allclose(back[key], params[key].detach().numpy(), atol=1e-6)
            self.assertNotIn("colors", back, "色なしで書いたのに色が読めている")
            # 四元数は書き出し時に正規化される
            np.testing.assert_allclose(np.linalg.norm(back["quats"], axis=1), 1.0, atol=1e-5)

    def test_colours_survive_the_round_trip(self):
        """反射強度を入れて書いたら、読み戻したときに同じ値になること。"""
        import tempfile
        from g1_mapping.lidar_gs import INTENSITY_SCALE

        rng = np.random.default_rng(4)
        points = rng.uniform(-2, 2, size=(120, 3))
        raw = rng.uniform(5, 150, size=120)
        params = to_parameters(init_from_points(points, intensities=raw), DEVICE)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "g.ply"
            write_splat_ply(path, params)
            back = read_splat_ply(path)
        self.assertIn("colors", back)
        recovered = 1 / (1 + np.exp(-back["colors"][:, 0])) * INTENSITY_SCALE
        np.testing.assert_allclose(recovered, raw, atol=1.0)

    def test_header_is_the_inria_layout(self):
        """既存の 3DGS ビューアが読める並びであること。"""
        import tempfile

        params = to_parameters(init_from_points(np.zeros((3, 3)) + np.arange(3)[:, None]),
                               DEVICE)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "g.ply"
            write_splat_ply(path, params)
            head = path.read_bytes().split(b"end_header")[0].decode("ascii")
        self.assertIn("format binary_little_endian 1.0", head)
        for name in ("x", "y", "z", "opacity", "scale_0", "scale_2", "rot_0", "rot_3",
                     "f_dc_0", "f_dc_2", "nx"):
            self.assertIn(f"property float {name}\n", head)


if __name__ == "__main__":
    unittest.main(verbosity=2)
