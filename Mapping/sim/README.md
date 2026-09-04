# Mapping / sim

Isaac Simから実機と同じROSトピック契約でLiDAR・IMU・RGBを配信し、実機raw backendと
同じadapter・FAST-LIO2・recorder・RVizを検証する。

## 起動

```bash
# 端末1: Isaac Simセンサ
cd Mapping/sim
./run_mapping_sensors.sh

# 端末2: FAST-LIO2と記録
cd ../real
./mapctl doctor --backend sim
./mapctl start --backend sim --name warehouse

# 端末3: 密度地図・登録点群・自己位置・軌跡・RGB
cd Mapping/real
./mapctl view --live
```

終了時は`./mapctl stop`、`./mapctl validate`を実行する。
`/g1_sim/ground_truth/*`は評価専用であり、FAST-LIO2へは入力しない。
