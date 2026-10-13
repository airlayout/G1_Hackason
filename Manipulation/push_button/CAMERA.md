# J1-gen方式の頭カメラ接続

実機の`real/localize_button.py`と`real/push_button.py --align`は、J1-genの`RgbdZmqSource`でRGB-Dを受け取る。
PC2ではカラーをUnitreeのvideohubから受け取り、深度だけをRealSenseから開く。videohubは停止しない。
受信側でpyrealsense2やカメラデバイスを開く処理は使わない。

```text
PC2: videohubのカラー + RealSenseの深度
  → J1-gen rgbd_server --source videohub（位置合わせ）
  → ZMQ 5556
  → RgbdZmqSource → YOLO → 3D推定・再計測 → 既存の押下処理
```

## 受信PCの準備

以下はリポジトリ直下で実行する。画像処理側のPython環境に依存を入れる。
SDKブリッジのPython環境と、PC2で配信するシステムのPythonは変更しない。

```bash
python -m pip install -r Manipulation/push_button/real/requirements_camera.txt
python Manipulation/push_button/real/prepare_camera.py
```

`prepare_camera.py`は`origin/Dev/ButtonPress`のJ1-genと、その版のPerception共通部品を
`_local/button_press_camera/<コミット>/`に取り出し、公式G1モデルを用意する。
作業ブランチの変更、merge、commit、pushは行わない。完了した版を`current.json`に記録する。
取得済みの参照を使うときは`--no-fetch`、版を固定して再現するときは`--ref <コミット>`を付ける。
既存の公式モデルを再利用する場合は`--model-cache /absolute/path/to/g1_description`。
この場合は`SOURCE.txt`のコミットがJ1-genの設定と一致するか検査する。

通常配置の`Button_Press/J1-gen`がある場合はそちらを使う。
配置を明示する場合は`--j1gen-root /absolute/path/to/Button_Press/J1-gen`を付ける。

## PC2側の配信

PC2にあるJ1-genの作業ディレクトリで、既存の配信手順に従って起動する。

```bash
bash Button_Press/J1-gen/real/depth_server/start_rgbd_server.sh --source videohub
```

J1-genの部品を`prepare_camera.py`で用意した配置なら、上の`Button_Press/J1-gen`を
`_local/button_press_camera/<コミット>/Button_Press/J1-gen`に読み替える。
配信側のPython・オフライン依存の手順は、J1-genの`real/depth_server/README.md`を参照する。
この変更でPC2のOS設定やUnitreeサービスを変更する必要はない。

## 接続先と受信確認

接続先はJ1-genの設定をそのまま使う。

| 指定 | 接続先 |
| --- | --- |
| 既定の`camera.yaml` | 有線PC2 `192.168.123.164:5556` |
| `--camera-config camera_wifi.yaml` | J1-genの無線設定。IPは現場で確認する |
| `--host <PC2のIP>` | 設定ファイルの接続先を上書きする |

独自の設定ファイルも`--camera-config /absolute/path/to/camera.yaml`で指定できる。
ポート番号はそのYAMLの`rgbd.port`で指定する。画像サイズ・深度の単位・内部パラメータは配信メッセージから読む。
videohub経由の640×360も、通常の640×480も受け取れる。画像はBGRのままYOLOへ渡す。

```bash
python Manipulation/push_button/real/localize_button.py \
  --weights /absolute/path/to/best.pt \
  --host 192.168.123.164 --frames 10 \
  --preview _local/button_press_camera/preview.png \
  --output _local/button_press_camera/target_optical.json
```

この確認では腕を動かさない。`--target-pixel U V`でプレビュー上の対象を指定できる。
出力は既定で`camera_optical`。そのまま腕の計画に渡さず、床基準への変換を行う`--align`を使う。
`localize_button.py --base-transform`は、従来の実測4×4行列による静的な確認用。
v2の補正値と現在姿勢を使う変換は`real/push_button.py --align`が行う。

## 頭カメラの座標変換と校正

頭カメラの光学座標から骨盤座標までは、J1-genの`HeadCameraTransform`と公式
`g1_29dof_rev_1_0`モデルで腰角度から計算する。次に骨盤高とIMUのroll/pitchで、
既存の腕FKと同じ床基準へ変換する。yawの基準は停止時の前方を0とする。
腕のモデル・IK・押下制御は今回変更していない。

新しい校正形式`g1-button-calibration-v2`では、READMEの共通の手先寸法・停止姿勢に加えて、
次の2つの3要素配列を実測して指定する。

- `camera_offset_m`: 公称の光学座標の右・下・前に沿った取り付け位置の補正 [m]。
- `camera_rpy_offset_rad`: 同じ光学座標のX・Y・Z軸に対するRPY補正 [rad]。

変換は`T_base_pelvis × T_pelvis_optical(腰角度) × T_camera_correction`。
補正が不要と確認できた場合だけ0を指定する。公式モデルの値だけでは光学中心・取り付け・手先の実測誤差は保証できない。
従来のv1校正も受け付け、校正姿勢で元の実測行列を再現する固定補正へ変換する。
どちらもG1の頭カメラ用で、`camera_body`は`torso_link`。

既存の計画表示・実行コマンドに`--camera-config`や`--host`を付けられる。
実行用の`--execute --calibrated`、静止監視、押込量の上限、再計測の3フレーム条件は維持する。

## 画像の時刻

PC2と受信PCの時計を同期しておく。配信元のtimestampから受信側の単調時計へ換算し、
0.5秒より古い画像、未来に0.05秒以上ずれた画像、同じID・時刻の画像を採用しない。
再計測を開始する前の画像も除外し、新しい3フレームだけで位置の安定性を確認する。
受信は短い待機を繰り返すため、配信が止まっても再計測の期限を大幅には超えない。

videohubのカラーとRealSenseの深度は別経路なので、撮影の瞬間は完全には一致しない。
配信timestampの検査は、露光時刻の関節角との厳密な同期を保証するものではない。
実機では停止して計測し、配信側の時刻差表示と校正誤差を確認する。

## 検証

カメラ部品を準備した環境で、次を実行する。ローカルのダミー配信を使い、実機へは接続しない。

```bash
python -m pytest Manipulation/push_button/tests/test_j1gen_camera.py -q
```

J1-genの実際の送受信部品による640×360の受信、BGR・深度単位、新しい3フレーム、
古い画像・重複・時計ずれの拒否、公式モデルの腰FK、床基準への変換、v1校正の引き継ぎを検査する。
本物のPC2・videohub・RealSenseを使った動作確認は別途必要。
