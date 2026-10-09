# Entame / real

実機G1上での実行コードをここに置く。

## `arm_wave_real.py`（SDK方式・実機確認済み）

`../README.md`の「実現方法の選択」でいうSDK方式（`sport_mode`）の実例。
`G1ArmActionClient.ExecuteAction()`でUnitree標準の定型モーション（"high wave"等）を
呼び出す。実機で"high wave"の動作を確認済み（`LocoClient.WaveHand()`はcode=0でも
実際には動かないという既知の落とし穴があるので、こちらを使うこと。詳細は
`FAILURES.md`とスクリプト冒頭のコメントを参照）。

```bash
python Entame/real/arm_wave_real.py --network-interface enp3s0
```

lerobot方式（独自振り付け）はまだ未着手。

## Pepper VLM Brain snapshot

[pepper_vlm_brain/](pepper_vlm_brain/README.md) にPepper向けVLM Brainのtracked source snapshotを収録しています。G1統合デモとは独立しており、source SHA・Ubuntu利用の前提・未検証事項は同READMEを参照してください。

## Pepper 認識アプリ

[pepper_app/](pepper_app/README.md) は、カメラの映像から人・椅子・手の動き（手を挙げる・振る）を CPU の YOLO で認識し、画面で割り当てた操作（見る・手を振る）を NAOqi API（ポート 9559）で Pepper に送る Streamlit アプリです。回る・進むはありません。
