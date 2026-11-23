"""動きを滑らかにする補完処理（関節のブルブルを抑える）。依存なし。

- OneEuro: 手の入力のような、ゆっくりのときは強く平らにし、速く動くときは遅れを小さくする適応型ローパス（One Euro フィルタ）。
- CriticallyDamped: 関節の指令を、速度・加速度が連続になるよう、ばね・ダンパで追わせる 2 次フィルタ（行ったり来たりの切り替えを平らにする）。
- interpolate: フレーム間の指令を、物理の刻みごとに直線で補う（階段状の指令でサーボが毎フレーム叩かれるのを防ぐ）。
- jitter: 動きのブルブルの大きさ（2 階差分の RMS と、微小な向き反転の回数）。before/after の比較用。
"""
import math


class OneEuro:
    """One Euro フィルタ（Casiez ら 2012）。min_cutoff が小さいほど静止時に強く平らに、beta が大きいほど速い動きの遅れが小さい。"""

    def __init__(self, min_cutoff=1.0, beta=2.0, d_cutoff=1.0):
        self.min_cutoff, self.beta, self.d_cutoff = min_cutoff, beta, d_cutoff
        self.x = None
        self.dx = 0.0

    @staticmethod
    def _alpha(cutoff, dt):
        tau = 1.0 / (2 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def __call__(self, value, dt):
        if self.x is None or dt <= 0:
            self.x = float(value)
            return self.x
        d = (value - self.x) / dt
        self.dx += self._alpha(self.d_cutoff, dt) * (d - self.dx)
        cutoff = self.min_cutoff + self.beta * abs(self.dx)
        self.x += self._alpha(cutoff, dt) * (value - self.x)
        return self.x


class CriticallyDamped:
    """臨界減衰の 2 次フィルタ。time_constant（秒）が小さいほど速く追う。オーバーシュートしない。"""

    def __init__(self, time_constant=0.08):
        self.omega = 2.0 / time_constant
        self.x = None
        self.v = 0.0

    def __call__(self, target, dt):
        if self.x is None:
            self.x = float(target)
            return self.x
        remaining = dt
        while remaining > 1e-9:                          # 大きい dt でも安定なよう、刻んで進める（半陰的オイラー）
            h = min(remaining, 0.5 / self.omega)
            accel = self.omega ** 2 * (target - self.x) - 2 * self.omega * self.v
            self.v += accel * h
            self.x += self.v * h
            remaining -= h
        return self.x


def smooth_series(values, times, min_cutoff=1.0, beta=2.0):
    """値の列を One Euro で滑らかにした列を返す（元は変えない）。times は各値の時刻（秒）。"""
    f, out = OneEuro(min_cutoff, beta), []
    for i, v in enumerate(values):
        out.append(f(v, times[i] - times[i - 1] if i else 0.0))
    return out


def interpolate(previous, current, fraction):
    """previous から current へ fraction（0〜1）だけ進めた値の dict。キーは current のもの。previous に無い値は current。"""
    return {k: previous.get(k, v) + (v - previous.get(k, v)) * fraction for k, v in current.items()}


def jitter(series, dt=1.0, noise=0.05):
    """ブルブルの大きさ。(2 階差分の RMS [単位/dt²], 微小な向き反転の回数)。
    向き反転 = 1 ステップの動きが noise 以下の小さな動きで、前のステップと逆向きになった回数（大きな折り返しは数えない）。"""
    if len(series) < 3:
        return 0.0, 0
    second = [series[i + 1] - 2 * series[i] + series[i - 1] for i in range(1, len(series) - 1)]
    rms = math.sqrt(sum(s * s for s in second) / len(second)) / dt ** 2
    steps = [series[i + 1] - series[i] for i in range(len(series) - 1)]
    flips = sum(1 for a, b in zip(steps, steps[1:]) if a * b < 0 and abs(a) <= noise and abs(b) <= noise and abs(a) > 1e-9 and abs(b) > 1e-9)
    return rms, flips
