# Mapping / real

G1の部屋Mappingを、内蔵LIOと生LiDAR＋IMUの2方式で実行する現場用環境。
操作はすべて`mapctl`から行い、成果物形式はbackendによらず共通になる。

## backend

| backend | 処理場所 | 主な依存 | PCD保存 |
|---|---|---|---|
| `onboard` | G1内蔵SLAMサービス | Unitree SDK2 | G1へ保存後、SCPで回収 |
| `raw` | 現場Ubuntu PC | ROS 2 Humble、FAST-LIO2 | 現場PCへ直接保存 |

いずれも、購読できるLiDAR・IMU・odometryをrosbag2へ並行記録する。

## 必要環境

- Ubuntu PC
- Docker Engine + Docker Compose
- G1と直結するEthernet NIC
- PCD自動回収を使う場合は、G1へログインできるSSH鍵

ROS 2やPCLをホストへ直接インストールする必要はない。

## 初回準備

```bash
cp .env.example .env
# .envのG1_IFACEと必要なトピック名を編集
chmod +x mapctl scripts/*.sh docker/*.sh
./mapctl build
```

## 実機なしでの動作確認

```bash
./mapctl doctor --mock
./mapctl start --backend mock --name local_test
./mapctl status
./mapctl stop
./mapctl validate
```

mockでも実際と同じセッション状態遷移を通り、PCD、trajectory、rosbag metadata、
品質レポートを生成する。

## 現場操作

```bash
./mapctl doctor --backend auto
./mapctl start --backend auto --name room_a
./mapctl status
./mapctl stop
./mapctl validate
```

## rosbagから地図を作り直す

通信断などでG1へ「Mapping終了」を送れないと、G1側にPCDが書き出されずセッションが
`map_raw.pcd`なしで終わる。この場合はrosbagに残った地図点群から作り直せる。

```bash
./mapctl rebuild                          # 最新セッション
./mapctl rebuild <session_id>             # セッション指定
./mapctl rebuild --voxel 0.02             # 解像度を上げる（既定 0.05m）
./mapctl rebuild --voxel 0                # 間引きなし
./mapctl rebuild --topic /unitree/slam_mapping/points
./mapctl rebuild --force                  # 既存のmap_raw.pcdを上書きする
```

`db3`を直接読むため**ROS 2もDockerも要らず**、`metadata.yaml`が欠けたセッションからも
復旧できる。処理はホストのPythonだけで完結する（標準ライブラリのみ）。

既定では`onboard`セッションは`ONBOARD_POINTS_TOPIC`、それ以外は`RAW_POINTS_TOPIC`を使う。
`RAW_POINTS_TOPIC`はセンサー座標系なので、姿勢で変換しない限り重ならない点に注意する。
つまり実用になるのは`onboard`セッションの再構成である。

同一ボクセルに落ちた点は最初の1点だけを残す。既存の`map_raw.pcd`は`--force`なしでは
上書きしない（実機から回収した地図のほうが正であるため）。

作り直したら`./mapctl validate`で検証し、`./mapctl view <session_id>`で確認する。

## RViz表示

Mapping中のライブ表示は、別のGUI端末から実行する。

```bash
./mapctl view --live
```

保存済みPCDはG1に接続していない開発PCでも表示できる。

```bash
./mapctl view                         # 最新セッション
./mapctl view <session_id>            # セッション指定
./mapctl view <session_id> --publish-only
```

`--publish-only`はPCDを`/g1_mapping/map`へ配信するだけで、RVizを起動しない。
GUI版はX11/XWaylandの`DISPLAY`を可視化専用コンテナへ渡す。Mappingコンテナへ
RVizやディスプレイソケットを渡すことはない。

GUI接続で失敗する場合は`echo $DISPLAY`を確認する。Xauthorityが標準位置にない環境では、
実在する認証ファイルを`XAUTHORITY`へ設定してから実行する（例:
`export XAUTHORITY=/run/user/<UID>/gdm/Xauthority`）。

詳細は[`../FIELD_RUNBOOK.md`](../FIELD_RUNBOOK.md)を参照。

## オフライン配備

開発PCでimagesを構築した後、次を実行する。

```bash
./mapctl bundle
```

`dist/`に3つのDocker images（onboard、raw、visualization）、ソース、チェックサムを
まとめたfield kitが作られる。
現場PCで展開し、`install-field-kit.sh`を実行する。

## 外部依存の固定

利用するcommitは[`vendor/mapping.repos`](vendor/mapping.repos)と
[`docker/Dockerfile`](docker/Dockerfile)のbuild argsで固定している。更新時は両方を同時に
変更し、mock、Docker build、rosbag replayを再検証する。
