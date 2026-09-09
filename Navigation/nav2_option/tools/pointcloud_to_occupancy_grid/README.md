# pointcloud_to_occupancy_grid

Planning.md A-7 に対応。点群地図(`.pcd`/`.ply`)から Nav2 `map_server` 形式(`.pgm`+`.yaml`)への
変換ツール。

## セットアップ

```bash
python3.12 -m venv .venv
source .venv/bin/activate
pip install numpy open3d
```

## 使い方

```bash
source .venv/bin/activate
python pointcloud_to_occupancy_grid.py <入力.ply> --out map
```

生成される `map.pgm` / `map.yaml` はそのまま Nav2 の `map_server` に渡せる。

### 主なオプション

| オプション | 既定値 | 意味 |
|---|---|---|
| `--resolution` | 0.05 | グリッド解像度[m/cell] |
| `--min-height` / `--max-height` | 0.3 / 1.8 | 障害物とみなす高さ範囲[m](D-21) |
| `--padding` | 1.0 | 点群の外周に足す余白[m] |

## 設計上のポイント

**未観測を「自由」にしない。** 高さフィルタ範囲外の点しか無いセルを一律「自由」と
判定すると、実際には未観測なだけの領域が「自由に通れる」と誤判定され、Nav2の
経路計画が建物の外側や壁の裏を通ろうとする問題が起きる(姉妹プロジェクト`Navigation/`の
知見を踏襲)。そのため3値で出力する:

- **occupied(黒, 0)**: 高さフィルタ範囲内(既定0.3〜1.8m)に点があるセル
- **free(白, 254)**: 高さに関係なく点はあるが、フィルタ範囲内には無いセル(観測済みだが障害物ではない)
- **unknown(灰, 205)**: どの高さにも点が1つも無いセル(未観測)

## 動作確認(2026-09-09)

vendoring した `../../vendor/fast_lio_localization_humanoid/data/map.ply`
(221,330点)で実行し、壁の輪郭・観測済み領域が正しく分離されることを目視確認済み。

```
[info] 読み込んだ点数: 221330
[info] z範囲: -6.138 .. 5.216
[info] グリッドサイズ: 1120 x 1102 (0.05 m/cell)
[info] occupied=25543 (2.1%) free=92635 (7.5%) unknown=1116062 (90.4%)
```

生成された地図をPNG変換して目視したところ、部屋・廊下の壁の輪郭が明瞭に現れ、
観測済み領域(白)と未観測領域(灰)が意図通り分離されていることを確認した。

## 未検証・既知の制約

- G1実機のMID-360で取得した点群での検証はまだ（サンプル地図はUnitree公式の別データ）
- 高さフィルタの既定値(0.3〜1.8m)は姉妹プロジェクト`Navigation/`の実測に基づく値の踏襲であり、
  G1実機・MID-360の実際の取付高さ(Planning.md U-09で確定)に応じて調整が必要になる可能性がある
- 大きい孤立した未観測領域(部屋の内部の死角等)をどう扱うか(自由のまま残すか、通行不可にするか)は
  今回実装していない。Nav2のGlobal costmapは`unknown_cost_value`パラメータで未知領域の扱いを
  設定できるため、そちらで調整する想定
