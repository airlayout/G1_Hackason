# FAST_LIO_LOCALIZATION_HUMANOID の ROS 2 Jazzy ビルド検証

検証日: 2026-09-08
検証者: Claude (Docker 内実ビルドによる検証)
対象リポジトリ: https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID
検証環境: `osrf/ros:jazzy-desktop` (Ubuntu 24.04) 上の Docker コンテナ、ホストは Ubuntu 24.04 / Docker 29.1.3、8 vCPU / 61GB RAM / 300GB+ 空きディスク

## 結論

**軽微な修正でビルド可能。ROS 2 Jazzy 継続方針を維持してよい。**

- リポジトリの **`main` ブランチは ROS1 (catkin_make) 用**であり、README にある「Ubuntu 22.04 / Humble」の記述は実態と食い違っている。**ROS2 の実装は `humble` ブランチ**にある（README 冒頭に "ROS2 (Humble) version already supports this, please check the Humble branch" と告知あり）。今回の検証はこの `humble` ブランチを対象にした。
- `humble` ブランチのコードは `ament_cmake` / `rclcpp` / `tf2_ros` (`.hpp` 系ヘッダ) を使った素直な ROS2 移植であり、Jazzy 固有の非互換（`tf2` 非推奨API、`rclcpp` シグネチャ変更、`ament_target_dependencies` 廃止など）は**一つも発見されなかった**。
- ビルドを阻害したのは ROS2/Jazzy 起因ではなく、**サードパーティ依存の Open3D のバージョン差分**（後述）のみ。1関数呼び出しから引数を1つ削除するだけの1行修正でビルドが通った。
- colcon build で `livox_ros_driver2` / `fast_lio` / `open3d_loc` の3パッケージ全てが成功（`Summary: 3 packages finished`）。warning のみで error はゼロ。
- ビルド後、`fastlio_mapping` ノードと `global_localization_node` を実際に起動し、正しい起動引数・パラメータを与えた場合はクラッシュせず、LiDAR/オドメトリのトピック購読待ち状態に正常に入ることを確認した（後述の「ランタイム確認」参照）。

## 依存関係一覧

`humble` ブランチの `package.xml` / `CMakeLists.txt` から抽出。

| パッケージ | 依存 | 備考 |
|---|---|---|
| `fast_lio` | `rclcpp`, `rclcpp_components`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs`, `std_srvs`, `visualization_msgs`, `pcl_ros`, `pcl_conversions`, `tf2`, `common_interfaces`, `livox_ros_driver2`, `Eigen3`, `PCL` | ament_cmake。`ikd-Tree` は `FAST_LIO/include/ikd-Tree` 配下にソースがそのままコミット済み（`.gitmodules` はあるが実体は同梱されておりサブモジュール初期化不要）。`find_package(PythonLibs REQUIRED)` が必須（`matplotlibcpp.h` は同梱だが未使用）。 |
| `open3d_loc` | `rclcpp`, `rclpy`, `tf2`, `tf2_ros`, `tf2_geometry_msgs`, `tf2_eigen`, `tf2_sensor_msgs`, `pcl_ros`, `pcl_conversions`, `cv_bridge`, `image_transport`, `urdf`, `eigen3_cmake_module`, `PCL`, `OpenCV`, `Boost (filesystem, system)`, `yaml-cpp` (pkg-config), **`Open3D`** | ament_cmake。Open3D は rosdep 管理外で `find_package(Open3D)` を素の CMake パスで解決する必要がある。 |
| `livox_ros_driver2` | Livox-SDK2 (C/C++ ライブラリ、rosdep管理外) | 本体は上流の別リポジトリ（`Livox-SDK/livox_ros_driver2`）から取得する必要あり（`humble` ブランチには同梱されていない）。**上流の `build.sh` は `jazzy` を明示的にサポート**しており (`readonly VERSION_JAZZY="jazzy"`, `CMakeLists.txt:285` で `DISTRO_ROS STREQUAL "jazzy"` を分岐)、ROS2 Jazzy 対応は upstream 側で担保済み。 |

### Open3D バージョンについて（最重要の依存）

- README (`humble` ブランチも `main` と同一内容が使い回されており未更新) は **Open3D 0.14.1** を前提とし、Baidu Netdisk（中国国内限定の配布）からの手動ダウンロードか、ソースからのビルドを指示している。
- ソースビルドは Open3D 単体で数時間規模になりうるため、今回は代わりに **Open3D 公式 GitHub リリースの prebuilt devel パッケージ `open3d-devel-linux-x86_64-cxx11-abi-0.18.0.tar.xz`**（`cxx11 ABI`版、Jazzy/Ubuntu24.04 の GCC13 の new ABI と一致）を採用した。0.14.1 系の C++ devel パッケージは GitHub リリースに存在しない（0.14.1 時点では Python wheel と conda パッケージのみで、C++ 向け `devel` tar.xz 配布が始まったのは概ね 0.17 以降）。
- バージョンを 0.14.1 → 0.18.0 に上げた結果、`open3d::pipelines::registration::RegistrationRANSACBasedOnFeatureMatching()` の末尾の `seed` 引数が Open3D 側で削除されており、コンパイルエラーになった（詳細は次節）。それ以外に使われている Open3D API（`RegistrationICP`, `RegistrationGeneralizedICP`, `ComputeFPFHFeature`, `VoxelDownSample`, `EstimateNormals`, `ReadPointCloud`, `Tensor` 変換まわり等）は 0.18.0 でも変更なく、そのままビルド・実行できた。

## 遭遇したエラーと原因

### 1. （唯一の実エラー）Open3D API のバージョン差分によるコンパイルエラー

```
/root/ws_loc/src/open3d_loc/src/open3d_registration/open3d_registration.cpp:34:53:
error: too many arguments to function
  'open3d::pipelines::registration::RegistrationResult
   RegistrationRANSACBasedOnFeatureMatching(const PointCloud&, const PointCloud&,
     const Feature&, const Feature&, bool, double, const TransformationEstimation&,
     int, const vector<reference_wrapper<const CorrespondenceChecker>>&,
     const RANSACConvergenceCriteria&)'
```

- **原因**: Open3D 0.14.1 時代にはこの関数に末尾 `seed`（乱数シード、`utility::optional<unsigned int>`）引数があったが、Open3D 0.18.0 では削除されている。これは **ROS2 / Jazzy とは無関係の、Open3D 単体のバージョン間 API 変更**である（Humble + Open3D 0.18 の組み合わせでも同一のエラーが出る）。
- **切り分け根拠**: `ament_cmake` / `rclcpp` / `tf2_ros` 側の記述はすべて Jazzy でもそのまま通っており、warning レベルでも Jazzy 固有の非推奨警告は出ていない（後述の warning 一覧参照）。エラーは Open3D のヘッダ (`/opt/open3d/include/open3d/pipelines/registration/Registration.h`) 側の宣言との不一致のみ。

**適用した修正**（1関数呼び出しから引数を1つ削除するのみ。`open3d_loc/src/open3d_registration/open3d_registration.cpp`）:

```diff
         registration_result = open3d::pipelines::registration::
             RegistrationRANSACBasedOnFeatureMatching(
                 *source, *target, *source_fpfh, *target_fpfh,
                 mutual_filter, distance_threshold,
                 open3d::pipelines::registration::
                     TransformationEstimationPointToPoint(false),
                 4 /*最小3*/, correspondence_checker,
-                open3d::pipelines::registration::RANSACConvergenceCriteria(1000000, 0.999), seed_);
+                open3d::pipelines::registration::RANSACConvergenceCriteria(1000000, 0.999));
+        (void)seed_;  // Open3D >= 0.16 でこのAPIから seed 引数が削除されたため
         return registration_result;
```

この修正を適用した Dockerfile（`sed` で自動適用）で `colcon build` を実行した結果:

```
Summary: 3 packages finished [1min 22s]
  3 packages had stderr output: fast_lio livox_ros_driver2 open3d_loc
```

3パッケージとも `error` はゼロ、`warning` のみで正常終了した。

### 2. Open3D_DIR のハードコード（Jazzy 非依存の設定作業）

`open3d_loc/CMakeLists.txt` に開発者のホームディレクトリが直書きされている:

```cmake
set(Open3D_DIR "/home/sax/open3d141/lib/cmake/Open3D")
```

これは README にも「自分の展開先に書き換えること」と明記された想定通りの作業であり、Jazzy 固有の問題ではない。今回は `/opt/open3d/lib/cmake/Open3D`（コンテナ内に配置した Open3D devel パッケージのパス）に `sed` で書き換えた。

### 3. ビルド時 warning（エラーではない、参考情報）

- CMake Policy 系の deprecation warning（`CMP0148` FindPythonLibs 廃止予定、`CMP0144` / `CMP0074` の `_ROOT` 変数関連、Open3D 側の `CMP0072` OLD 挙動）。いずれも "Warning (dev)" レベルでビルドは継続され、失敗しない。ROS2 の標準的な `ament_cmake` パッケージで一般的に見られる警告であり、Jazzy 特有ではない。
- `boost/bind.hpp` の "Bind placeholders in the global namespace is deprecated" というコンパイル時メッセージ（IKFoM_toolkit 経由）。警告のみで実害なし。
- `open3d_conversions.cpp` 内の `size_t` と `long` の符号比較 warning、未使用変数 warning多数。いずれも実行に影響しない静的解析レベルの指摘。
- Livox-SDK2 のサードパーティ同梱ライブラリ（spdlog, rapidjson）が C++20 の `char8_t` 予約語やコンパイラ固有 pragma に関する warning を出すが、これも Livox-SDK2 自体の3rdparty同梱コードの話で ROS2/Jazzy とは無関係。

## ランタイム確認（実データなしでのクラッシュチェック）

ビルド成功後、実際にノードを起動して即座にクラッシュしないかを確認した。

### `fastlio_mapping`（FAST_LIO 本体）

正しい launch ファイル・config（`ros2 launch fast_lio mapping.launch.py rviz:=false`、`mid360.yaml` 使用）で起動したところ、

```
[INFO] [laser_mapping]: p_pre->lidar_type 1
Multi thread started
[INFO] [laser_mapping]: Node init finished.
```

まで到達し、クラッシュせず LiDAR トピックの購読待ち状態で安定した。
（注: 必須パラメータを渡さず実行ファイルを直接叩いた場合はセグフォルトしたが、これは「configファイルを与えずに起動した」呼び出し側のミスであり、正規の launch 経由では再現しない。）

### `global_localization_node`（open3d_loc 本体）

同梱のサンプル地図 `data/map.ply` を使い、正しいパラメータを与えて起動したところ、

```
TRACE ... after ReadPointCloud points=221330
...
[WARN] [global_loc_node]: initialize finished
[INFO] [global_loc_node]: wait for Odometry_loc
[INFO] [global_loc_node]: Waiting for Odometry_loc...
```

まで到達し、地図読み込み・ダウンサンプリング・法線推定・Open3D↔ROSメッセージ変換すべてが正常に動作したうえで、オドメトリ入力待ちの定常状態に入ることを確認した。

**ただし調査の過程で、このノード固有の（Jazzy とは無関係の）既存バグを2件発見した:**

1. **同梱の `config/loc_param_g1.yaml` の ROS ノード名不一致**: YAML のトップレベルキーが `global_localization_node:` だが、実際のノードは `this->Node("global_loc_node")`（コード内、`global_localization.cpp:244`）で登録されている。そのため `--params-file loc_param_g1.yaml` を渡してもパラメータが一切適用されない。
2. **上記1の結果、`initialpose` パラメータがコード側のデフォルト値（空の `std::vector<double>`）のまま使われ、`initialpose_[3], initialpose_[4], initialpose_[5]` への範囲外アクセスが発生し、コンストラクタ内で SIGSEGV（起動即クラッシュ）する。**
   - 該当箇所: `open3d_loc/src/global_localization.cpp:369`
   - 該当宣言: `this->declare_parameter<std::vector<double>>("initialpose", std::vector<double>());`（`global_localization.cpp:327`、デフォルトが空配列）
   - これは GDB でスタックトレースを取得し、`initialpose` を `-p` で直接（YAMLファイル経由ではなく）与えることで回避できることを実機確認して原因を特定した。**Humble でも同一の構成（同じ config ファイル）で組み合わせれば同じ理由でクラッシュするはずで、Jazzy 固有の問題ではない。**
   - また `open3d_loc_g1.launch.py` 内の地図パスも `/home/sax/GO2_Localization_ROS2/...` という開発者ローカルパスがハードコードされており、これも実運用前に書き換えが必要（README記載の既知の作業）。
   - さらにノードを `SIGINT` で終了させた際に `terminate called without an active exception` が出た（別スレッドの後始末に起因すると思われる、シャットダウン時の軽微な不具合）。起動直後のクラッシュではないため、今回のビルド可否判定には影響しない。

**結論として、`humble` ブランチはまだ実運用向けに十分にテストされていない「できたてのポート」であり、パス直書きやconfig不整合などの粗さが残っている。** これは Jazzy 対応可否とは別軸の「アプリケーションとしての成熟度」の課題として、Nav2 本統合前に別途手直しが必要になる。

## 修正内容の差分サマリ

| ファイル | 修正内容 | 分類 |
|---|---|---|
| `open3d_loc/CMakeLists.txt` | `Open3D_DIR` を `/opt/open3d/lib/cmake/Open3D` に変更 | ローカル環境設定（README記載の想定作業、Jazzy非依存） |
| `open3d_loc/src/open3d_registration/open3d_registration.cpp` | `RegistrationRANSACBasedOnFeatureMatching()` 呼び出しから末尾の `seed_` 引数を削除 | Open3D バージョン差分対応（0.14.1→0.18.0）、Jazzy非依存 |

**Jazzy / ROS2 API 起因のコード修正は0件。**

## 今後の推奨

1. **Jazzy 継続方針は維持して問題ない。** `humble` ブランチのコードベース自体は Jazzy でも同一の修正（Open3D APIの1行差分吸収）だけで動く。Humble に切り替えても Open3D周りの差分は同様に発生するため、「Jazzy だから追加コストが生じる」要素は無い。
2. **Open3D の調達方法を確定させる必要がある。** 今回は 0.18.0 の公式 devel パッケージを使ったが、以下のいずれかを選ぶ必要がある:
   - (a) 公式 devel パッケージ（0.17〜最新）を使い、上記1行差分を当てて使う（**今回の検証で動作確認済み、推奨**）。
   - (b) README指定の 0.14.1 をソースからビルドする（数時間規模、Baidu Netdiskからの手動取得が前提でCI化しづらい）。
   - (a) を採用する場合、Open3D側の将来のマイナーバージョンアップでさらに API 差分が出る可能性があるため、devel パッケージのバージョンを固定（pin）してDockerfileに明記しておくこと（本検証のDockerfileは 0.18.0 に固定済み）。
3. **`global_localization_node` 側の既知バグ（config ノード名不一致、`initialpose` 範囲外アクセスによるクラッシュ）は Nav2 統合前に upstream への修正提案 or フォークでの独自修正が必要。** 現状の `config/loc_param_g1.yaml` をそのまま使うと確実に起動時クラッシュする。
4. **地図パスなどのハードコードされた開発者ローカルパス**（`open3d_loc_g1.launch.py` の `map_file`、`CMakeLists.txt` の `Open3D_DIR`）は運用時に必ず上書きが必要。launch引数化するなど、自分たちで統合する際に併せて改善するのが望ましい。
5. **livox_ros_driver2 は upstream (`Livox-SDK/livox_ros_driver2`) から素直に取得すれば Jazzy を正式サポートしている**ため、追加の心配は不要。ただし `FAST_LIO_LOCALIZATION_HUMANOID` の `main`（ROS1）ブランチに含まれる「G1向けにIMU外部パラメータを反転できるよう改造した」独自版 `livox_ros_driver2` は `humble` ブランチには同梱されていない。G1のLiDAR上下逆さま問題への対応（README 0.1節）が `humble` ブランチでどう扱われているか（同じ改造が反映されているか）は、今回の検証スコープ外なので別途確認が必要。
6. **Humble専用コンテナでの分離実行は不要と判断する。** ビルド・起動まで確認できた今回の結果から、Jazzy環境に一本化してこのパッケージを組み込む方針で進めて問題ない。

## 参考: 検証に用いたビルド手順の要点

- ベースイメージ: `osrf/ros:jazzy-desktop`
- Livox-SDK2: `git clone https://github.com/Livox-SDK/Livox-SDK2` → 素の CMake ビルド → `make install`
- livox_ros_driver2: `git clone https://github.com/Livox-SDK/livox_ros_driver2` → `package_ROS2.xml` を `package.xml` に、`launch_ROS2/` を `launch/` にコピー（upstream `build.sh` と同じ処理を手動再現）→ ワークスペースの一部として colcon ビルド
- Open3D: 公式 `open3d-devel-linux-x86_64-cxx11-abi-0.18.0.tar.xz` を展開して `/opt/open3d` に配置
- FAST_LIO_LOCALIZATION_HUMANOID: `humble` ブランチを clone、`FAST_LIO` と `open3d_loc` をワークスペースにコピー
- 前述の2箇所を `sed` で修正
- `rosdep install --from-paths src --ignore-src -r -y` で残りの依存を解決
- `colcon build --symlink-install --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=jazzy`

再現用の完全な Dockerfile は同ディレクトリの `fastlio_jazzy.Dockerfile` を参照。
