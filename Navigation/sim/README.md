# Navigation / sim

**実機なしで巡回を最後まで走らせる場所。** 実機 G1 は 1 台しかなく歩行検証の時間が
限られるため、実機を使わずに進められる範囲をここで最大化する。

`nav/mission.py` から見た相手は `slam_service.py` で、実機の PC1 と同じ
`slam_operate`（1804/1102/1201/1202）の顔をしている。

## 構成

| ファイル | 役割 |
|---|---|
| `fetch_assets.sh` | `unitree_rl_gym` から `motion.pt` と G1 の MJCF を取る（24MB・`.gitignore`済み） |
| `rooms.py` | 部屋の寸法から **MJCF（物理）と点群（地図）の両方**を出す |
| `g1_walker.py` | 学習済み 12DoF ポリシー + MuJoCo。速度指令を受けて歩く。`mujoco_lidar` もここ |
| `slam_service.py` | 1804/1102/1201/1202 を受け、目標との差から速度指令を作る。507 の注入もここ |
| `run_sim.py` | シナリオを 1 本走らせて評価する |
| `npz_to_pcd.py` | `scans.npz`（実機データ）→ 地図 PCD + 真値軌跡 |
| `maps/` | 上の出力（`.gitignore`済み。再生成できる） |

## 使い方

`Navigation/navctl` から叩く。

```bash
cd Navigation
./navctl setup     # 歩行資産(24MB)の取得と依存の導入。最初に1回
./navctl view      # GUI で見る（既定は実時間）
./navctl sim       # ヘッドレス。約50倍速
./navctl help      # 一覧
```

詳しい実行例と実測値は `Navigation/README.md` の「動かす」節にある。

## なぜ自作の運動学モックをやめたか

以前は `fake_service.py` が目標へ等速直線で進む運動学モデルを持っていた。
これだと**経路が引ければ必ず歩けてしまう**ので、歩行の失敗を一件も見つけられない。
Phase 3.5 で Unitree 公式の学習済みポリシー + MuJoCo に差し替えたところ、
その場で不具合が 2 件出た（`Navigation/README.md`「物理で歩かせて初めて見つかった不具合2件」）。

**地図と MuJoCo の世界が一致していること**が要で、それを保証するのが `rooms.py`。
寸法を 1 か所に持ち、そこから両方を生成するので食い違いが原理的に起きない。
