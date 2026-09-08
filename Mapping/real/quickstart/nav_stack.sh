#!/usr/bin/env bash
# TF・OctoMap・Nav2 を立ち上げる。データ源は「記録の再生」と「実機」から選ぶ。
#
#   offline  記録した bag を再生する。実機に触らない（既定）
#   live     実機の LiDAR と odom を使う
#
# ## なぜ実機なしで組めるのか
#
# 記録は正規の rosbag2（型も CDR も ROS 標準）なので、`ros2 bag play` で当時のトピックが
# そのまま蘇る。TF は odom から作れる。障害物も生 LiDAR から作れる。
# **実機が要るのは /cmd_vel を足に渡すところだけ**で、そこまでは全部これで詰められる。
#
# ## 前提
#
#   bash quickstart/start_rviz_mac.sh offline     # コンテナを実機なしで起動
#
# ## 使い方
#
#   bash quickstart/nav_stack.sh                    # 記録を再生して起動（実機に触らない）
#   bash quickstart/nav_stack.sh --rate 1           # 等速で再生
#   G1_BAG_OFFSET=130 bash quickstart/nav_stack.sh --rate 1   # 記録の +130 秒から再生
#                                                   #（初期姿勢を軌跡から作り直す）
#   G1_BAG_LOOP=1 bash quickstart/nav_stack.sh      # ループ再生。**測定には使えない**
#                                                   #（巻き戻しで TF が全部死ぬ。下記）
#   bash quickstart/nav_stack.sh live               # 実機のデータで起動
#   bash quickstart/nav_stack.sh stop               # 全部止める
#
#   G1_USE_MOLA=1 bash quickstart/nav_stack.sh      # map->odom を MOLA-LO に出させる（段 3）
#
# ## G1_USE_MOLA=1 が何を変えるか
#
# 既定では odom_to_tf.py が map->odom を**固定変換**として出す。内蔵SLAM の odom は
# 歩行中に roll/pitch が中央値 10.5° 狂うので（2026-09-07 実測）、固定変換では
# 事前地図とライブ点群が重ならない。
# G1_USE_MOLA=1 にすると odom_to_tf.py から map->odom を外し（--no-map-odom）、
# MOLA-LO が測位のみモードで scan-to-map の結果として map->odom を毎スキャン出す。
# 地図は runs/<SESSION>/mola_floor0/map.mm を読む（無ければ止まる。G1_MOLA_MAP で上書き可）。
# その .mm は **SLAM 地図座標系に x/y/yaw を合わせて作ってある**ので、map_server の
# nav_map.pgm とそのまま噛み合う（合わせ方は下の MOLA_MAP の注記と run_mola_lo.sh）。
#
# ⚠️ live でも**足は動かない**。Nav2 は /cmd_vel を出すだけで、それを足に渡すのは
# PC2 側の loco_driver.py（Navigation/real/）である。動かすにはあちらを --arm で起こす。
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# DDS の URI と IP/名前の既定値は _common.sh に 1 箇所だけ置いてある
# shellcheck source=_common.sh
. "$HERE/_common.sh"

NAME="$G1_RVIZ_NAME"
SESSION="${G1_SESSION:-20260906T135940_UiS_room_v3}"
BAG="/work/G1_Hackason/Mapping/real/runs/$SESSION/raw/rosbag2"
RATE="3"
NAV2_PARAMS="/work/G1_Hackason/Navigation/nav2/g1_nav2.yaml"
# 実測値（2026-09-05）。**度**で渡す。--livox-rpy はラジアンなので取り違えないこと
LIVOX_RPY_DEG="178.35 -8.41 -0.72"
LIVOX_XYZ="-0.004 0.016 -0.037"
# OctoMap の maxRange。2026-09-06 の掃引で 2m が誤除去最少だった
OCTO_MAX_RANGE="${G1_OCTO_MAX_RANGE:-2.0}"
# 段 3: map->odom を MOLA-LO に出させるか
USE_MOLA="${G1_USE_MOLA:-0}"
# ⚠️ 2 つ間違えやすいので注意（どちらも 2026-09-07 に実機で踏んだ）:
#   1. **`map_full.mm`（sm2mm 製）では測位できない。** パイプラインの ICP は
#      `localmap` 層（HashedVoxelPointCloud）を見るが、sm2mm の既定生成器は
#      `raw` 層しか作らない。対応点が 0 で ICP 品質が 0.00 に張り付く。
#      → `MOLA_SAVE_MM` が出す `map.mm` を使う（実機で品質 0.85〜0.89）
#   2. **`mola_floor0` と `mola_aligned` で map の z 原点が 1.3 m 違う。**
#      floor0 は MOLA_INITIAL_Z=1.228（床上高さ）で作り直したもので **床が z≈0**。
#      g1_nav2.yaml の高さ帯はこちら用（min_obstacle_height 0.23 .. 1.80）。
#      0.23 は床の振れ幅（+0.126）+10cm から check_calibration.py が算出した値。
#      aligned に戻すなら帯も 1.30 m 下げる（実測の z 差。0.23 → -1.07）。
#      ⚠️ 以前ここに書いていた -1.10 は**床の振れ幅を知る前の値**で、床の最大に対する
#      余裕がほぼ無い。そのまま戻すと今日直した間欠失敗が再発する。
#      値は焼かず check_calibration.py で測り直すこと（一対で動かす）
MOLA_MAP="${G1_MOLA_MAP:-/work/G1_Hackason/Mapping/real/runs/$SESSION/mola_floor0/map.mm}"
MOLA_PIPELINE="/work/G1_Hackason/Mapping/real/quickstart/mola/g1_lidar3d_icp_imu.yaml"
# MOLA の初期姿勢（測位のみモードの種）。**[x, y, z, yaw, pitch, roll]・角度は度。**
#
# ⚠️ **省略すると収束しない。** パイプライン YAML の既定は全部 0 なので、
# roll が 178° 違う（LiDAR は逆さま取付）ところから ICP を始めることになる。
#
# ⚠️ **これは `base_link` の map 系での姿勢**であって、地図の datum そのものではない。
# `ignore_lidar_pose_from_tf:=false` で走らせるので、MOLA は TF の
# `base_link -> livox_frame`（odom_to_tf.py が LIVOX_RPY_DEG/LIVOX_XYZ で流す静的変換）
# を噛ませてセンサ姿勢を作る。よって渡すべき値は
#
#   base_link_in_map = livox_in_map ∘ (base_link->livox)^-1
#
# 地図 `mola_floor0` の datum（= 記録開始時の livox 姿勢）は rebuild.log に出ている:
#   "Initial re-localization done with pose: (x,y,z,yaw,pitch,roll)=
#    (-0.0168, 0.0264, 1.2280, 0.24deg, 3.32deg, 177.93deg)"
# これに上の静的変換の逆を掛けると下の値になる（2026-09-08 算出）。
#
# ⚠️ **pitch が -11.75° なのは誤りではない。** base_link は G1 の内蔵 odom が定義する
# body 系で、それ自体が map（重力基準）に対して傾いている
# （odom の roll/pitch は静止中でも 10〜20° 狂う。それを毎スキャン直すのが MOLA の役目）。
# センサ姿勢は上の合成で地図 datum に一致するので、ICP は正しい場所から始まる。
#
# ⚠️ **地図（MOLA_MAP）を替えたらこの値も替える。** rebuild.log の datum を読み直し、
#    上の式で計算し直すこと。z だけは `mola_aligned` なら 1.228 → -0.0437 相当になる。
#
# ⚠️⚠️ **合成の向きを間違えやすい（2026-09-08 に 1 度間違えた）。**
# rpy は ZYX（R = Rz(yaw)·Ry(pitch)·Rx(roll)）で、MRPT の CPose3D と
# odom_to_tf.py の quaternion_from_rpy がこの規約。**scipy の `from_euler('zyx', ...)`
# は小文字＝外因性なので順序が逆になり、pitch と yaw の符号を間違える。**
# 自分で合成せず `initial_pose_at.py` に出させること（あれは軌跡の quaternion から
# rebuild.log の datum を厳密に再現するところまで検算済み）。
MOLA_LOC_METHOD="${G1_MOLA_LOC_METHOD:-InitLocalization::FixedPose}"
MOLA_INIT_POSE="${G1_MOLA_INIT_POSE:-[-0.0051, 0.0108, 1.2635, 0.911, 11.736, -0.282]}"

# 記録のどこから再生するか（秒）。**0 以外なら初期姿勢を軌跡から作り直す。**
#
# なぜ要るか: 20260906T135940_UiS_room_v3 は **t+418 秒以降ずっと開始点に戻って静止**
# しており、その開始点は静的地図で ±2.5 m の **42% が占有**という散らかった場所である。
# そこで経路計画を測ると「機体が囲まれている」しか出ない（2026-09-08 実測: 0/20）。
# 開けた場所は t+230s の (0.86, +15.93) あたり。
# MOLA に食わせる IMU トピック。**PitchAndRollFromIMU を使うなら差し替えが要る。**
# G1 の生 IMU は orientation が (0,0,0,0) で、MOLA の ImuInitialCalibrator が
# MRPT の CQuaternion に渡して例外 → **致命停止**する（2026-09-08 実測）。
# G1_IMU_FIX=1 で imu_orientation_fix.py を挟み、単位クォータニオンに差し替える。
IMU_FIX="${G1_IMU_FIX:-0}"
IMU_TOPIC="${G1_IMU_TOPIC:-/utlidar/imu_livox_mid360}"
[ "$IMU_FIX" = "1" ] && IMU_TOPIC="/imu_fixed"

BAG_OFFSET="${G1_BAG_OFFSET:-0}"
MOLA_TRAJ="${G1_MOLA_TRAJ:-$(dirname "$MOLA_MAP")/traj.txt}"
# Nav2 の global_costmap.static_layer が読む /map の出どころ。
# 既定を nav_map_clean にしてある。2026-09-08 の実測で、元の nav_map.pgm は
# **機体が実際に歩いた道の 60.4%（軌跡セル）を自分で塞いでいた**（占有セルまでの
# 距離 中央値 0.224m < robot_radius 0.30m）。原因は 2 つで、どちらも作り方の側にあった:
#   1. 追従者が軌跡沿いに焼き込まれていた（近距離除去 + OctoMap で落とした）
#   2. 障害物帯の下限 0.15m が min_obstacle_height 0.23m と食い違い、床を撃っていた
# 作り直した nav_map_clean は 中央値 0.781m / 0.30m 未満 1.9%。
# 昔の測定を再現したいときは G1_NAV_MAP で nav_map.yaml を明示すること。
# 作り方: filter_scans_near.py → run_octomap.py → pcd_to_occupancy.py
# 合否:   check_map_clearance.py <地図> <traj.txt> --baseline <元の地図>
NAV_MAP="${G1_NAV_MAP:-/work/G1_Hackason/Mapping/real/runs/$SESSION/map/nav_map_clean.yaml}"

MODE="offline"
[ "${1:-}" = "live" ] && { MODE="live"; shift; }

# offline はブリッジ NIC が無いので DDS をループバックに閉じる。
# live は G1 の L2 に載っている col0 に載せる
# （コンテナは --network host なので VM と同じ netns を見る）
DDS="$(g1_dds_uri "$MODE")"

say() { echo "[stack] $*"; }

# コンテナ内でノードを 1 つ起こす。環境は毎回明示する（bash -lc は XAUTHORITY を落とすので使わない）
spawn() {
    local log="$1"; shift
    docker exec -d -u ubuntu \
        -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI="$DDS" -e ROS_DOMAIN_ID=0 \
        "$NAME" bash -c "source /opt/ros/humble/setup.bash && $* > /home/ubuntu/$log 2>&1"
}

if [ "${1:-}" = "stop" ]; then
    # パターンに自分自身が入らないよう [] で括る（pkill -f は自分のコマンド行にも当たる）
    #
    # ⚠️ `nav2_[a-z]` のような大雑把なパターンを使わないこと。
    # これは実行ファイルのパス `/opt/ros/humble/lib/nav2_map_server/map_server` にも
    # 当たるので、Nav2 だけ落としたつもりで **map_server も落ちる**。
    # 2026-09-07 にこれで 2 回 map_server を殺し、そのたびに
    # 「global_costmap の静的レイヤに地図が来ない（Can't update static costmap layer）」
    # を別件としてデバッグした。ノードは実行ファイル名で個別に止める。
    #
    # ⚠️ **`mola-cli` は SIGTERM を無視する。** 2026-09-08 に実測: pkill（既定 SIGTERM）の
    # あとも `Sl` で生き続け、1 時間 55 分前に起こしたプロセスがそのまま残っていた。
    # SIGINT なら 3 秒以内に落ちる。**古い MOLA が生き残ると古い地図で map->odom を
    # 出し続ける**ので、次の起動で「新しい地図を読ませたのに測位がおかしい」という
    # 追いにくい形で出る。だから mola 系は -INT で送り、最後に生存確認 → SIGKILL する。
    # ⚠️⚠️ **パターンは配列で持つ。空白区切りの文字列にしてはいけない。**
    # 2026-09-08 に踏んだ: `PATTERNS="ros2 bag pla[y] ..."` を `for p in $PATTERNS` で
    # 回すと単語分割で `ros2` / `bag` / `pla[y]` の 3 語になり、最初の
    # **`pkill -f ros2` が停止スクリプト自身を殺す**（自分のコマンド行に "ros2" が入る）。
    # 結果、後半のパターンに到達せず odom_to_tf / mola-cli / octomap / map_server が
    # 生き残り、**次の起動でスタックが二重になった**（トピックが 2 つの発行元を持つ）。
    docker exec "$NAME" bash -c '
        PATTERNS=(
            "ros2 bag pla[y]" "odom_to_t[f]" "octomap_serve[r]" "navigation_launc[h]"
            "imu_orientation_fi[x]"
            "nav2_controller/controller_serve[r]" "nav2_planner/planner_serve[r]"
            "nav2_bt_navigator/bt_navigato[r]" "nav2_behaviors/behavior_serve[r]"
            "nav2_velocity_smoother/velocity_smoothe[r]"
            "nav2_lifecycle_manager/lifecycle_manage[r]"
            "nav2_map_server/map_serve[r]"
        )
        # mola は SIGTERM を無視するので SIGINT で送る
        MOLA_PATTERNS=("mola-cl[i]" "ros2-lidar-odometr[y]")

        for p in "${PATTERNS[@]}";      do pkill -f      "$p"; done
        for p in "${MOLA_PATTERNS[@]}"; do pkill -INT -f "$p"; done

        # 落ちきるのを待ってから、残ったものだけ SIGKILL
        for _ in 1 2 3 4 5 6; do
            alive=0
            for p in "${PATTERNS[@]}" "${MOLA_PATTERNS[@]}"; do
                pgrep -f "$p" >/dev/null && alive=1
            done
            [ "$alive" = "0" ] && break
            sleep 1
        done
        for p in "${PATTERNS[@]}" "${MOLA_PATTERNS[@]}"; do pkill -KILL -f "$p"; done
        exit 0' >/dev/null 2>&1

    # 本当に消えたかを**呼び出し側で確かめる**（黙って生き残るのが一番困る）
    left="$(docker exec "$NAME" bash -c '
        for p in "ros2 bag pla[y]" odom_to_t[f] octomap_serve[r] mola-cl[i] \
                 controller_serve[r] planner_serve[r] bt_navigato[r] map_serve[r]; do
            pgrep -f "$p" >/dev/null && echo "$p"
        done' 2>/dev/null)"
    if [ -n "$left" ]; then
        say "⚠️ 残っています: $(echo "$left" | tr '\n' ' ')"
        exit 1
    fi
    say "止めました（RViz2 は残す）"
    exit 0
fi
if [ "${1:-}" = "--rate" ]; then RATE="$2"; shift 2; fi

docker inspect "$NAME" >/dev/null 2>&1 || { echo "[stack] コンテナ $NAME が無い。先に start_rviz_mac.sh" >&2; exit 1; }

# 再生位置をずらすなら初期姿勢も作り直す（G1_MOLA_INIT_POSE を明示したときは触らない）
if [ "$BAG_OFFSET" != "0" ] && [ -z "${G1_MOLA_INIT_POSE:-}" ]; then
    docker exec "$NAME" test -f "$MOLA_TRAJ" || {
        echo "[stack] 軌跡が無い: $MOLA_TRAJ（G1_MOLA_TRAJ で指定できる）" >&2; exit 1; }
    MOLA_INIT_POSE="$(docker exec "$NAME" python3 \
        /work/G1_Hackason/Mapping/real/quickstart/initial_pose_at.py \
        "$MOLA_TRAJ" --offset "$BAG_OFFSET" --quiet)" || {
        echo "[stack] 初期姿勢の算出に失敗した" >&2; exit 1; }
    say "再生位置 +${BAG_OFFSET}s に合わせて初期姿勢を作り直した: $MOLA_INIT_POSE"
fi

# 再生では /clock を使うので use_sim_time が要る。実機では実時刻なので使わない
SIM_TIME="true"
if [ "$MODE" = "live" ]; then
    SIM_TIME="false"
    say "[1] 実機のデータを使う（再生はしない）"
    docker exec "$NAME" bash -c "source /opt/ros/humble/setup.bash && RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI='"'"'$DDS'"'"' ROS_DOMAIN_ID=0 timeout 8 ros2 topic hz /utlidar/cloud_livox_mid360 2>&1 | tail -1"
else
    docker exec "$NAME" test -d "$BAG" || { echo "[stack] bag が無い: $BAG（/work のバインドを確認）" >&2; exit 1; }
    # ⚠️⚠️ **`--loop` は既定にしない（2026-09-08 に踏んだ）。**
    # bag が巻き戻ると `/clock` の sim time が過去に戻る。TF のバッファは未来の
    # スタンプを抱えたままなので、以降すべての変換が
    # `TF_OLD_DATA ignoring data from the past for frame base_link` で拒否され、
    # **MOLA は base_link -> livox_frame を引けずに黙ってスキャンを捨てる**
    # （/lidar_odometry/pose も pose_quality も出なくなる。落ちないので気づきにくい）。
    # 1 周 = 591 秒しか測れないので、測り直すときは**スタックごと再起動する。**
    BAG_OPTS="--rate $RATE --clock"
    [ "${G1_BAG_LOOP:-0}" = "1" ] && BAG_OPTS="$BAG_OPTS --loop"
    [ "$BAG_OFFSET" != "0" ] && BAG_OPTS="$BAG_OPTS --start-offset $BAG_OFFSET"
    say "[1] 記録を再生する（$SESSION / ${RATE}倍速 / 開始 +${BAG_OFFSET}s / ループ ${G1_BAG_LOOP:-0}）"
    spawn bagplay.log "ros2 bag play $BAG $BAG_OPTS"
    sleep 4
fi

if [ "$USE_MOLA" = "1" ]; then
    say "[2] TF を流す（map -> odom は MOLA-LO に任せるので出さない）"
    spawn odomtf.log "python3 /work/G1_Hackason/Mapping/real/quickstart/odom_to_tf.py \
        --nav2-frames --no-map-odom --livox-rpy-deg $LIVOX_RPY_DEG --livox-xyz $LIVOX_XYZ"
else
    say "[2] TF を流す（map -> odom は固定変換で出す）"
    spawn odomtf.log "python3 /work/G1_Hackason/Mapping/real/quickstart/odom_to_tf.py \
        --nav2-frames --livox-rpy-deg $LIVOX_RPY_DEG --livox-xyz $LIVOX_XYZ"
fi
sleep 4

if [ "$IMU_FIX" = "1" ]; then
    say "[2b] IMU の姿勢クォータニオンを直して中継する（-> ${IMU_TOPIC}）"   # 全角の直後は ${} で括る（そうしないと変数名に取り込まれる）
    IMU_FIX_ARGS="--in /utlidar/imu_livox_mid360 --out $IMU_TOPIC"
    [ "$MODE" = "live" ] && IMU_FIX_ARGS="$IMU_FIX_ARGS --no-sim-time"
    spawn imufix.log "python3 /work/G1_Hackason/Mapping/real/quickstart/imu_orientation_fix.py $IMU_FIX_ARGS"
    sleep 3
fi

if [ "$USE_MOLA" = "1" ]; then
    docker exec "$NAME" test -f "$MOLA_MAP" || {
        echo "[stack] MOLA の地図が無い: $MOLA_MAP" >&2
        echo "[stack] 先に docker exec rviz bash /work/.../run_mola_lo.sh <SESSION> で作る" >&2
        exit 1; }
    say "[3] MOLA-LO を測位のみモードで起こす（map -> odom を出させる）"
    say "    地図: $MOLA_MAP"
    say "    初期姿勢($MOLA_LOC_METHOD): $MOLA_INIT_POSE"
    # start_mapping_enabled:=False が測位のみ。地図は更新しない
    # publish_localization_following_rep105:=True（既定）で map -> odom を出す
    # min_nearby_poses_occupied:=2 は Livox の非反復スキャン向け
    spawn mola.log "ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
        mola_lo_pipeline:=$MOLA_PIPELINE \
        lidar_topic_name:=/utlidar/cloud_livox_mid360 \
        imu_topic_name:=$IMU_TOPIC \
        use_imu_for_lio:=True \
        min_nearby_poses_occupied:=2 \
        start_mapping_enabled:=False \
        mola_initial_map_mm_file:=$MOLA_MAP \
        initial_localization_method:=$MOLA_LOC_METHOD \
        initial_pose:=\"$MOLA_INIT_POSE\" \
        publish_localization_following_rep105:=True \
        ignore_lidar_pose_from_tf:=false \
        use_mola_gui:=False use_rviz:=False \
        use_sim_time:=$SIM_TIME"
    sleep 10
fi

say "[4] OctoMap を作る（生 LiDAR / maxRange ${OCTO_MAX_RANGE}m）"
# 地図点群 /unitree/slam_mapping/points は stamp=0 で TF を引けないので使わない
spawn octomap.log "ros2 run octomap_server octomap_server_node --ros-args \
    -p resolution:=0.10 -p frame_id:=map -p base_frame_id:=base_link \
    -p sensor_model.max_range:=$OCTO_MAX_RANGE -p use_sim_time:=$SIM_TIME \
    -r cloud_in:=/utlidar/cloud_livox_mid360"
sleep 6

# Nav2 の global_costmap.static_layer は /map を読む（g1_nav2.yaml:140）。
# その /map を出すノードが 2026-09-06 の構成には無く、静的レイヤが空のままだった
# （2026-09-07 に気づいた）。map_server はライフサイクルノードなので
# configure/activate を lifecycle_bringup で明示的に叩く必要がある。
if docker exec "$NAME" test -f "$NAV_MAP"; then
    say "[5] map_server を起こす（$(basename "$NAV_MAP") を /map に流す）"
    spawn mapserver.log "ros2 run nav2_map_server map_server --ros-args \
        -p yaml_filename:=$NAV_MAP -p use_sim_time:=$SIM_TIME -p frame_id:=map"
    sleep 4
    spawn mapserver_bringup.log "ros2 run nav2_util lifecycle_bringup map_server"
    sleep 6
else
    say "[5] ⚠️ $NAV_MAP が無いので map_server を飛ばす（Nav2 の静的レイヤは空になる）"
fi

say "[6] Nav2 を起動する"
spawn nav2.log "ros2 launch nav2_bringup navigation_launch.py \
    params_file:=$NAV2_PARAMS use_sim_time:=$SIM_TIME"
sleep 20

say "起動したもの:"
docker exec "$NAME" bash -c 'pgrep -af "bag pla[y]|odom_to_t[f]|octomap_serve[r]|controller_serve[r]|planner_serve[r]|bt_navigato[r]|mola_lidar_odometr[y]|map_serve[r]" | sed "s/^/  /" | cut -c1-110'
echo
say "ログ: docker exec $NAME tail -f /home/ubuntu/{bagplay,odomtf,mola,mapserver,octomap,nav2}.log"
say "RViz2: http://localhost/ （VNC パスワード ubuntu）"
