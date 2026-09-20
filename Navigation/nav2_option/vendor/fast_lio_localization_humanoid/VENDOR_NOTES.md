# Vendoring notes — FAST_LIO_LOCALIZATION_HUMANOID

Planning.md の Q7決定（2026-09-09）に基づき、upstream リポジトリのソースを丸ごと
このディレクトリに社内取り込み(vendoring)した。GitHub fork + PR の運用ではなく、
**upstream への追従は行わない**。

## 取り込み元

- リポジトリ: https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID
- ブランチ: **`humble`**（`main` は ROS1/catkin_make 版。README の「Ubuntu22.04/Humble」表記は
  実態と食い違っており、実際の ROS2 実装は `humble` ブランチにある）
- 取り込み時点のコミット: `df4772ec4797172430e7efe990711d09a529f4ad`（2025-12-01）
- 検証: [../../findings/fastlio_jazzy_build.md](../../findings/fastlio_jazzy_build.md)（Phase A-6、2026-09-08）

## 取り込んだもの・除外したもの

| 対象 | 扱い | 理由 |
|---|---|---|
| `FAST_LIO/`（`doc/`除く） | 取り込み | ビルド・実行に必要なソース一式。`doc/`は128MBの動画・GIFでビルドに不要 |
| `open3d_loc/` | 取り込み | 同上 |
| `data/map.ply`（6.8MB） | 取り込み | 動作確認用のサンプル地図 |
| `doc/`（リポジトリ全体、256MB） | **除外** | READMEの説明用動画・GIFのみ。ビルド・実行に無関係 |
| `livox_ros_driver2` | **取り込まない** | upstream(`Livox-SDK/livox_ros_driver2`)がJazzyを正式サポート済み。改変不要なので通常の外部依存として都度取得する（`Dockerfile`参照） |
| `.git`（382MB） | 除外 | 履歴は不要。vendoringなのでこのリポジトリ自身のgit管理下に置く |

サイズ: 取り込み後の合計 約7.7MB。

## ライセンスに関する注記（要確認事項）

- `FAST_LIO/LICENSE` は **GPLv2**。FAST-LIO本家(HKU-Mars)由来のコードにこのライセンスが付与されている
- `open3d_loc/` には明示的なライセンスファイルが見当たらなかった
- **社内の検証・ハッカソン用途では問題にならない想定だが、外部への配布や商用化を検討する段階では
  GPLv2のコピーレフト条項（リンクした成果物全体への波及可能性）を法務確認すること**

## 適用したパッチ

### 1. Open3D API バージョン差分の吸収

**ファイル**: `open3d_loc/src/open3d_registration/open3d_registration.cpp`

Open3D 0.14.1（README指定、Baidu Netdisk限定配布で入手困難）ではなく、公式devel配布の
**0.18.0**を使う（[Dockerfile](Dockerfile)参照）。0.16以降で
`RegistrationRANSACBasedOnFeatureMatching()`の末尾`seed`引数が削除されているため、
呼び出しから該当引数を削除。`seed_`パラメータ自体はAPI互換性のため残し、`(void)seed_`で
未使用警告のみ抑制。

### 2. `Open3D_DIR`のCACHE変数化

**ファイル**: `open3d_loc/CMakeLists.txt`

開発者ローカルパス`/home/sax/open3d141/lib/cmake/Open3D`のハードコードを
`set(... CACHE PATH ...)`に変え、`-DOpen3D_DIR=...`で上書きできるようにした。
既定値は[Dockerfile](Dockerfile)がOpen3D develパッケージを展開する先`/opt/open3d`に合わせてある。

### 3. `initialpose`パラメータのサイズ検証を追加（Q7の本題）

**ファイル**: `open3d_loc/src/global_localization.cpp`

**発見された不具合**: `global_localization_node`実行ファイルを、`launch/open3d_loc_g1.launch.py`
経由ではなく直接（ノード名オーバーライド無しで）起動すると、`initialpose`パラメータが
コンストラクタのデフォルト値（空vector）のまま`initialpose_[3]`等へ範囲外アクセスし、SIGSEGVする。

**重要な訂正（当初の見立ては誤りだった）**: 最初は「`config/loc_param_g1.yaml`のトップレベルキー
`global_localization_node:`がC++コンストラクタのデフォルトノード名`global_loc_node`と
不一致だから」という理由でYAML側のキー名を書き換えようとした。しかし
`launch/open3d_loc_g1.launch.py`を確認したところ、**正規の起動経路ではlaunchファイルが
`Node(name='global_localization_node', ...)`でノード名を明示的に上書きしており、
元のYAMLのキーと一致していた**。つまりlaunchファイル経由の正規の使い方では、
そもそも不整合は無かった。YAMLを書き換えていたら、正規経路の方を壊すところだった
（一度適用して気づき、revertした）。

**採用した修正**: config/launchの名前解決には手を付けず、**C++コード側に
`initialpose_.size() != 6`のガードを追加**し、不正な場合は`RCLCPP_FATAL`で原因
（ノード名がparamsファイルのキーと一致しているか確認せよ、という具体的なメッセージ）を
ログに出したうえで`std::runtime_error`を送出するようにした。この修正は起動経路（launch経由か
直接実行か）によらず安全に効く。

**検証結果**（2026-09-09、Docker上で実施）:

| シナリオ | 修正前 | 修正後 |
|---|---|---|
| ノード名オーバーライド無しで直接実行（`initialpose`が空vectorのまま） | SIGSEGV（GDBでのスタックトレース取得が必要） | `RCLCPP_FATAL`で原因を明示 → `std::runtime_error`で制御されたabort |
| `-r __node:=global_localization_node`でノード名を一致させた場合（launch経由と同等） | 正常（`initialpose`が正しく`[0,0,0,0,0,0]`として読み込まれる） | 変化なし。同じく正常（回帰なし） |

## 未修正（今回のスコープ外・既知の課題として記録）

- `open3d_loc_g1.launch.py`の`map_file`が開発者ローカルパス
  (`/home/sax/GO2_Localization_ROS2/...`)にハードコードされている。運用時に書き換えが必要
  （Planning.md A-6残作業に記載済み）
- `kf_baselink2map`パラメータの読み込みに疑義あり。コードは
  `declare_parameter("kf_baselink2map/x", ...)`のようにスラッシュ区切りで宣言しているが、
  ROS 2のYAMLパラメータファイルはネストしたマップを**ドット区切り**でフラット化する規約のため
  （例: `kf_baselink2map.x`）、YAML側の`kf_baselink2map: {x: [...], ...}`という書き方では
  一致せず、常にデフォルト値`[0.0, 0.0]`が使われている可能性がある。実際、上記の検証ログでも
  `kf_x: [0.000000, 0.000000], size: 2`とデフォルトのまま出力されていた。**クラッシュはしないため
  Q7の対象（SIGSEGV修正）には含めなかったが、Kalman filterのパラメータが常に既定値になっている
  疑いがあり、Phase 2aで定位精度を評価する際に併せて確認すべき**
- ノード終了時（SIGINT）に`terminate called without an active exception`が出る
  （別スレッド後始末起因と推測。起動直後のクラッシュではないため今回は対応していない）

## ビルド確認

```bash
cd nav2_option/vendor/fast_lio_localization_humanoid
docker build -t g1-fastlio-jazzy-vendored .
```

2026-09-09、上記コマンドで`livox_ros_driver2` / `fast_lio` / `open3d_loc`の3パッケージが
ビルド成功することを確認済み（`Summary: 3 packages finished`、エラー0件）。
