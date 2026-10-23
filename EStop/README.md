# EStop — G1 緊急停止（USB 接続ボタン）

ZB2-BE102（1NC）のボタンを Jetson の USB に繋ぎ、押されたら G1 を安全停止する。
トルクはゼロにしない。**ゼロ速度で立ったまま静止**し、自動ではダンピングしない（D-13）。

設計・状態遷移・未確認事項は [DESIGN.md](DESIGN.md)。

```bash
python -m estop --port /dev/ttyUSB0 --iface eth0   # 端末で reset / damp を入力
PYTHONPATH=. python3 -m unittest discover -s tests -v
```
