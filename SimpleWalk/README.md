# SimpleWalk（前進歩行）

G1にキーボード/スクリプトから前進コマンドを送って歩かせる、最初に動作確認できた機能。
`Perception/`・`Mapping/`・`Navigation/`と同じ「シムで作る→実機で動かす」の型のテンプレート元。

## 構成

- `sim/` — シミュレーション（MuJoCo, `lerobot/unitree-g1-mujoco`）上での検証
  - `verify_g1_sim_command.py` — remote入力がコントローラに反映されるか検証
  - `patch_mujoco_elastic_band.py` — MuJoCo環境のelastic bandバグへのパッチ
  - `release_band_and_walk_forward.py` — elastic band解除→前進歩行の検証
- `real/` — 実機G1での実行（2方式あり、詳細は下記「2方式の違い」参照）
  - `walk_forward_real.py` — lerobot + 独自ONNXポリシー方式
  - `walk_forward_real_sdk.py` — Unitree SDK標準の高レベル歩行(sport_mode)方式

## 2方式の違い

`real/`には目的が違う2つのスクリプトがある。**同時には使えない**（片方が
ロボットの高レベル制御モードを占有するため）。

| | `walk_forward_real.py`<br>(lerobot方式) | `walk_forward_real_sdk.py`<br>(SDK方式) |
|---|---|---|
| 歩行の頭脳 | 独自ONNXポリシー(GrootLocomotionController、`nepyope/GR00T-WholeBodyControl_g1`) | Unitree製品版のsport_mode(`LocoClient`) |
| 通信経路 | `--robot-ip`。操作PC→ZMQ→G1本体の`run_g1_server.py`(DDS-ZMQブリッジ)→DDS | `--network-interface`。操作PC→直接DDS(G1本体への追加ソフトウェア不要) |
| G1本体側の追加セットアップ | 必要（Python 3.12 conda環境 + `run_g1_server.py`の起動） | 不要（工場出荷時のままで動く） |
| 依存パッケージ | `lerobot`, `torch`, `onnxruntime`, CycloneDDS, `unitree_sdk2py` | CycloneDDS, `unitree_sdk2py`のみ |
| 対応動作 | 前進/横移動/旋回のみ（ポリシーが対応する範囲） | 前進/横移動/旋回に加え、しゃがみ・立ち上がり・手を振る・握手など豊富 |
| 独自データでの学習・VLAファインチューニング(把持動作など) | ◎ `lerobot-record`でデータ収集→`lerobot_train.py`で学習(pi0.5等)→実機推論、という一連の仕組みが使える | ✕ Unitreeの固定動作のみで、独自モデルを載せる仕組みが無い |
| シムでの事前検証 | ◎ MuJoCo環境(`sim/`)で検証可能 | ✕ 対応するシム環境が無く、実機で直接試すことになる |
| 動作確認状況 | シミュレーション・実機ともに確認済み | 未検証（実機での動作確認はまだ行っていない） |

**使い分けの指針**: 決まった歩行だけで良い・G1本体側のセットアップを増やしたくない
場合はSDK方式。将来的に把持動作や独自ポリシーの学習・`Perception/`等と組み合わせた
VLA的な発展を考えるならlerobot方式（今回の環境構築の本命）。

## 環境

`G1_HuggingFace/venv/`（操作PC側）を使う。`walk_forward_real.py`（lerobot方式）は
さらにG1本体側のPython 3.12 conda環境（`lerobot`）と`run_g1_server.py`の起動が必要。
`walk_forward_real_sdk.py`（SDK方式）はG1本体側の追加セットアップ不要
（`unitree_sdk2py`が既にDDSでロボットと直接通信するため）。
セットアップ手順・動作確認結果は`G1_HuggingFace/README.md`と`G1_HuggingFace/Note`を参照。
ネットワーク接続・疎通確認は`Common/network/`を参照。

## 状態

- lerobot方式（`walk_forward_real.py`）: シミュレーション・実機ともに前進歩行の動作確認済み
- SDK方式（`walk_forward_real_sdk.py`）: 未検証（実機での動作確認はまだ行っていない）

失敗した内容は`FAILURES.md`に記録している（実機で歩行後にdisconnect()で脱力し
転倒した件など）。
