# Unitree SDK2 高レベル移動API 調査結果 (G1 / LocoClient)

調査日: 2026-09-08
調査対象:
- `unitree_sdk2` (C++): https://github.com/unitreerobotics/unitree_sdk2.git
  commit `9754cd153af3da471b0fe5f3aa535e426fb11db3` (2026-08-20)
- `unitree_sdk2_python`: https://github.com/unitreerobotics/unitree_sdk2_python.git
  commit `65691c8a8bc53b98d3976dba4dbf9d5d20b2e7f5` (2026-07-21)

クローン先（ローカル一時領域、本リポジトリには含めない）:
`/tmp/research_sdk2`, `/tmp/research_sdk2_python`

---

## 1. `Move()` の確定シグネチャ

### C++ (`unitree_sdk2`)
ファイル: `include/unitree/robot/g1/loco/g1_loco_client.hpp`

```cpp
// L232-236
int32_t Move(float vx, float vy, float vyaw, bool continous_move) {
  return SetVelocity(vx, vy, vyaw, continous_move ? 864000.f : 1.f);
}

int32_t Move(float vx, float vy, float vyaw) { return Move(vx, vy, vyaw, continous_move_); }
```

- オーバーロードが2つ存在する。
  - 3引数版: `Move(vx, vy, vyaw)` — 内部メンバ `continous_move_`（デフォルト `false`）を使う。
  - 4引数版: `Move(vx, vy, vyaw, continous_move)` — `bool continous_move` を明示指定できる。
- `continous_move_` メンバのデフォルト値（L308）: `bool continous_move_ = false;`
- `continous_move` フラグは `SwitchMoveMode(bool flag)`（L242-245）で恒久的に切り替えられる。

### Python (`unitree_sdk2_python`)
ファイル: `unitree_sdk2py/g1/loco/g1_loco_client.py`

```python
# L151-153
def Move(self, vx: float, vy: float, vyaw: float, continous_move: bool = False):
    duration = 864000.0 if continous_move else 1
    self.SetVelocity(vx, vy, vyaw, duration)
```

- 3引数 + キーワード引数1つ（`continous_move: bool = False`）の合計4引数。C++版と意味は同一。

### 結論
`Move()` は `vx, vy, omega(vyaw)` の3引数が基本だが、それに加えて **`continous_move`（bool）という4番目の引数が実在する**（C++はオーバーロード、Pythonはデフォルト引数）。`duration` という名前の引数は `Move()` 自体には存在しないが、内部で呼び出す `SetVelocity()` の `duration` 引数に変換されて渡される。

---

## 2. `duration` 相当引数の有無 → **結論: 「あり」（`SetVelocity()` に実在。`Move()` からは `continous_move` 経由で間接的に制御）**

`SetVelocity()` の定義（C++ヘッダ L164-174、Python L71-78）:

```cpp
// C++: include/unitree/robot/g1/loco/g1_loco_client.hpp L164
int32_t SetVelocity(float vx, float vy, float omega, float duration = 1.f) {
  std::string parameter, data;
  JsonizeVelocityCommand json;
  std::vector<float> velocity = {vx, vy, omega};
  json.velocity = velocity;
  json.duration = duration;
  parameter = common::ToJsonString(json);
  return Call(ROBOT_API_ID_LOCO_SET_VELOCITY, parameter, data);
}
```

```python
# Python: unitree_sdk2py/g1/loco/g1_loco_client.py L71
def SetVelocity(self, vx: float, vy: float, omega: float, duration: float = 1.0):
    p = {}
    velocity = [vx, vy, omega]
    p["velocity"] = velocity
    p["duration"] = duration
    parameter = json.dumps(p)
    code, data = self._Call(ROBOT_API_ID_LOCO_SET_VELOCITY, parameter)
    return code
```

- `duration` のデフォルト値は **1.0秒**。
- `Move(vx, vy, vyaw, continous_move=True)` を呼ぶと `duration=864000.0`（= 10日 ≒ 事実上「無期限」）に設定される。
- `continous_move=False`（デフォルト）の場合は `duration=1.0` 秒しか速度指令が有効でない、と読み取れる。
- ワイヤプロトコル上の型定義 `JsonizeVelocityCommand`（`g1_loco_api.hpp` L57-74）に `velocity: vector<float>[3]` と `duration: float` の2フィールドがあり、JSON形式で `{"velocity":[vx,vy,omega], "duration":d}` としてサーバ（G1本体のsportサービス、API ID `7105` = `ROBOT_API_ID_LOCO_SET_VELOCITY`）に送られる。
- C++サンプル (`example/g1/high_level/g1_loco_client_example.cpp` L128-146) のCLIオプション `--set_velocity="vx vy omega [duration]"` でも、4番目の引数として `duration` を直接渡せることを確認（未指定時はデフォルト1.0秒）。

---

## 3. `SetVelocity` の実在有無 → **結論: 実在する（仮説「存在しない可能性が高い」は誤り）**

- C++: `include/unitree/robot/g1/loco/g1_loco_client.hpp` L164 に `int32_t SetVelocity(float vx, float vy, float omega, float duration = 1.f)` として実装。
- Python: `unitree_sdk2py/g1/loco/g1_loco_client.py` L71 に `def SetVelocity(self, vx, vy, omega, duration=1.0)` として実装。
- `Move()` は `SetVelocity()` の薄いラッパーであり、`StopMove()`（`SetVelocity(0,0,0)`、C++ L226 / Python L140-141）もこれを利用している。
- API ID: `ROBOT_API_ID_LOCO_SET_VELOCITY = 7105`（`g1_loco_api.hpp` L29）、サービス名 `sport`（`LOCO_SERVICE_NAME = "sport"`, L11）。

---

## 4. 移動系メソッド一覧（`LocoClient` / G1）

C++ (`include/unitree/robot/g1/loco/g1_loco_client.hpp`) と Python (`unitree_sdk2py/g1/loco/g1_loco_client.py`) を突き合わせ。両言語でシグネチャがほぼ一致するが、細部の差異は注記した。

| メソッド | C++シグネチャ | Python シグネチャ | 内部実装 / 備考 |
|---|---|---|---|
| `Init()` | `void Init()` | `def Init(self)` | APIバージョン設定・全APIの登録 |
| `GetFsmId` | `int32_t GetFsmId(int& fsm_id)` | `def GetFsmId(self) -> (code, fsm_id)` | API 7001 |
| `GetFsmMode` | `int32_t GetFsmMode(int& fsm_mode)` | (C++のみ確認) | API 7002 |
| `GetBalanceMode` | `int32_t GetBalanceMode(int& balance_mode)` | (C++のみ) | API 7003 |
| `GetSwingHeight` | `int32_t GetSwingHeight(float&)` | (C++のみ) | API 7004 |
| `GetStandHeight` | `int32_t GetStandHeight(float&)` | (C++のみ) | API 7005 |
| `GetPhase` (deprecated) | `int32_t GetPhase(std::vector<float>&)` | (C++のみ) | API 7006, deprecated |
| `SetFsmId(int)` | 同左 | `def SetFsmId(self, fsm_id: int)` | API 7101 |
| `SetBalanceMode(int)` | 同左 | `def SetBalanceMode(self, balance_mode: int)` | API 7102 |
| `SetSwingHeight(float)` | 同左 | (C++のみ) | API 7103 |
| `SetStandHeight(float)` | 同左 | `def SetStandHeight(self, stand_height: float)` | API 7104 |
| `SetVelocity(vx,vy,omega,duration=1.0)` | 同左 | 同左 | API 7105（本調査の中核。§2, §3参照） |
| `SetTaskId(int)` | 同左 | `def SetTaskId(self, task_id: float)` | API 7106（腕タスク） |
| `SetSpeedMode(int)` | 同左 | `def SetSpeedMode(self, speed_mode: int)` | API 7107 |
| `SwitchToUserCtrl()` | 同左 | 同左 | API 7110 |
| `SwitchToInternalCtrl(InternalFsmMode)` | 同左 | 同左 | API 7111 |
| `Damp()` | `int32_t Damp() { return SetFsmId(1); }` | `def Damp(self): self.SetFsmId(1)` | fsm_id=1（減衰/脱力に近い安全停止モード） |
| `Start()` | `SetFsmId(500)` | `SetFsmId(500)` | メインFSM開始 |
| `Squat()` | `SetFsmId(2)` (C++のみ) | Python側は代わりに `Squat2StandUp()`/`StandUp2Squat()` が `SetFsmId(706)` | **C++とPythonでメソッド名・fsm_idマッピングに差異あり（要確認・バージョン差の可能性）** |
| `Sit()` | `SetFsmId(3)` | `def Sit(self): self.SetFsmId(3)` | |
| `StandUp()` | `SetFsmId(4)` (C++のみ) | Python側は `Lie2StandUp()`=`SetFsmId(702)`, `Squat2StandUp()`/`StandUp2Squat()`=`SetFsmId(706)` として提供 | 同上、名称差異に注意 |
| `ZeroTorque()` | `SetFsmId(0)` | 同左 | |
| `StopMove()` | `SetVelocity(0.f,0.f,0.f)` | `self.SetVelocity(0., 0., 0.)` | **duration省略＝デフォルト1.0秒**。1秒だけ速度ゼロを送るのみで「恒久的停止」を明示的に保証するAPIではない点に注意 |
| `HighStand()` | `SetStandHeight(UINT32_MAX)` | 同左 | |
| `LowStand()` | `SetStandHeight(UINT32_MIN)` (=0) | 同左 | |
| `Move(vx,vy,vyaw[,continous_move])` | 上記§1参照 | 上記§1参照 | 本調査の中核 |
| `BalanceStand()` / `BalanceStand(balance_mode)` | C++: `int32_t BalanceStand() { return SetBalanceMode(0); }`（引数なし） | Python: `def BalanceStand(self, balance_mode: int): self.SetBalanceMode(balance_mode)`（引数あり） | **シグネチャ差異あり（C++は固定0、Pythonは任意値を指定可能）** |
| `ContinuousGait(bool)` | `SetBalanceMode(flag ? 1 : 0)` | (Pythonのソースには見当たらず) | C++のみ確認 |
| `SwitchMoveMode(bool)` | `continous_move_` を更新 | (Pythonのソースには見当たらず) | 3引数版`Move()`のデフォルト挙動を変える。C++のみ確認 |
| `WaveHand(turn_flag=false)` | `SetTaskId(turn_flag?1:0)` | 同左 | |
| `ShakeHand(stage=-1)` | 同左 | 同左 | |
| `GetMimicMotion(data)` | `_fsm_api(...)` 経由でmimicモーション一覧取得 | (Pythonのソースには見当たらず) | fsm_id=550系。ロボットが安定姿勢（立位等）であることが前提、とコメントに明記 |

備考: C++版とPython版でAPI名・fsm_idマッピングに差異が見つかった（`Squat/StandUp` 系、`BalanceStand`の引数）。これはSDKのバージョン間の同期漏れの可能性があるため、**実機・実バージョンで動作確認すること**を推奨。

---

## 5. 指令が送られ続けない場合のG1の挙動（コメント・サンプルからの手がかり）

直接的に「高レベルMove/SetVelocityを送り続けないとどうなるか」を明記したコメントはSDK内に見当たらなかったが、以下の間接的な手がかりが得られた。

1. **`duration` パラメータの存在自体が「時限式」であることを示す**。`SetVelocity()` のデフォルト `duration=1.0`秒であり、`Move()` の `continous_move=False`（デフォルト）時も同じく1.0秒。これは指令が**永続的にlatchされるのではなく、指定時間だけ有効**という設計を強く示唆する。`continous_move=True` の場合のみ `duration=864000.0`（10日）となり、事実上「明示的に止めるまで動き続ける」動作になる。
2. `example/g1/high_level/g1_loco_client_example.py` のインタラクティブサンプルでは、`Move(vx,vy,omega)`（=1秒デュレーション）を1回呼んだ後 `time.sleep(1)` して次のユーザー入力を待つループになっており、**継続移動のために再送信するという運用は前提にされていない**（1回のコマンドで完結するデモ）。
3. C++側の低レベル制御に関する `include/unitree/robot/g1/common/terminations.hpp` の `lost_connection()` 関数（L73-86）のコメント:
   > "When using a wired connection to the robot, a loose network cable may cause the connection to be interrupted. **If the program continues to run at this time, it will send a step signal to the motors, causing violent movement.**"
   これは低レベル制御（LowCmd/LowState、モータ直接制御）における通信断のリスクに関する注意であり、高レベルLocoClientそのものの挙動ではないが、「通信が途切れた状態で指令を送り続ける／古い状態を使い続けることの危険性」についてUnitree自身が明示的に警告している点は設計上重要。
4. 高レベルAPI側（`sport`サービス）に同種の「connection lost検知」ヘルパーは見当たらなかった。つまり、**Bridgeプロセット側が能動的にwatchdog/タイムアウトを実装する必要がある**（SDKが自動的にやってくれる保証はない）。

**結論**: `duration` 引数の挙動から、G1本体側（sportサービス実行FSM）は受け取った速度指令を `duration` 秒後に自動的に無効化する（おそらく速度ゼロに戻す）と推測される。ただし、その後の具体的挙動（完全停止か、直前速度を保持し続けるか等）を明記したドキュメントはリポジトリ内で見つからなかったため、**実機検証が必要**（下記「設計上の示唆」参照）。

---

## 6. State型のフィールド一覧

G1（ヒューマノイド、`unitree_hg` 名前空間）は、Go2（四足、`unitree_go` 名前空間）とは別のIDL定義を使っている点に注意。

### 6.1 `unitree_hg::msg::dds_::SportModeState_`
ファイル: `include/unitree/idl/hg/SportModeState_.hpp`

```cpp
uint32_t fsm_id_ = 0;
uint32_t fsm_mode_ = 0;
uint32_t task_id_ = 0;
float task_time_ = 0.0f;
```

- フィールドは **`fsm_id`, `fsm_mode`, `task_id`, `task_time` の4つのみ**。
- **位置(position)・速度(velocity)・odometry相当のフィールドは一切含まれない。**

### 6.2 （参考・対比用）`unitree_go::msg::dds_::SportModeState_`（Go2/四足用、G1では使われない）
ファイル: `unitree_sdk2py/idl/unitree_go/msg/dds_/_SportModeState_.py`

```python
stamp: TimeSpec_
error_code: uint32
imu_state: IMUState_
mode: uint8
progress: float32
gait_type: uint8
foot_raise_height: float32
position: float32[3]        # ← 位置情報あり
body_height: float32
velocity: float32[3]        # ← 速度情報あり
yaw_speed: float32
range_obstacle: float32[4]
foot_force: int16[4]
foot_position_body: float32[12]
foot_speed_body: float32[12]
path_point: PathPoint_[10]
```

→ Go2用の `SportModeState_` には `position`, `velocity`, `body_height`, `imu_state` 等が含まれ、Nav2用odom相当のデータをそのまま得られるが、**G1用の `SportModeState_`（hg名前空間）には同種のフィールドが存在しない**。これはSDK2バージョン間の設計差であり、G1では高レベルAPIから直接world座標系の位置・速度は取得できないことを意味する。

### 6.3 `unitree_hg::msg::dds_::LowState_`
ファイル: `include/unitree/idl/hg/LowState_.hpp`

```cpp
std::array<uint32_t, 2> version_;
uint8_t mode_pr_;
uint8_t mode_machine_;
uint32_t tick_;
unitree_hg::msg::dds_::IMUState_ imu_state_;
std::array<unitree_hg::msg::dds_::MotorState_, 35> motor_state_;  // 全35関節
std::array<uint8_t, 40> wireless_remote_;
std::array<uint32_t, 4> reserve_;
uint32_t crc_;
```

- `imu_state_`: 姿勢・角速度・加速度（下記6.4）
- `motor_state_`: 35関節分のモータ状態（下記6.5）
- 座標系や単位に関する明示コメントはヘッダ内になし（IDL自動生成ファイルのため）。

### 6.4 `unitree_hg::msg::dds_::IMUState_`
ファイル: `include/unitree/idl/hg/IMUState_.hpp`

```cpp
std::array<float, 4> quaternion_;      // [w,x,y,z] 想定（順序の明記なし）
std::array<float, 3> gyroscope_;       // 角速度 (rad/s 想定、単位明記なし)
std::array<float, 3> accelerometer_;   // 加速度 (m/s^2 想定、単位明記なし)
std::array<float, 3> rpy_;             // roll/pitch/yaw (rad 想定)
int16_t temperature_;
```

- `terminations.hpp` の `bad_orientation()` 関数内で `quaternion()[0..3]` を `Eigen::Quaternionf(w,x,y,z)` の順で構築していることから、**`quaternion` の要素順は `[w, x, y, z]`** であると推測できる（コード上の使用例からの間接確認。IDL自体に順序の明記なし）。

### 6.5 `unitree_hg::msg::dds_::MotorState_`
ファイル: `unitree_sdk2py/idl/unitree_hg/msg/dds_/_MotorState_.py`（Python dataclassの方が読みやすいため引用）

```python
mode: uint8
q: float32          # 関節角度
dq: float32          # 関節角速度
ddq: float32         # 関節角加速度
tau_est: float32     # 推定トルク
temperature: int16[2]  # [winding, casing] （terminations.hppでの使用箇所から推測）
vol: float32
sensor: uint32[2]
motorstate: uint32
reserve: uint32[4]
```

- 単位・座標系についての明記はIDL内になし。`terminations.hpp` から `dq`（関節角速度、rad/s想定）に対する安全上限チェック（デフォルト10.0）が行われていることが分かる。

### 6.6 座標系・単位に関する所見のまとめ
- IDLファイル自体には単位・座標系のコメントは一切ない（自動生成ファイルのため）。
- 唯一の手がかりは `terminations.hpp` のデフォルト閾値（`joint_vel_out_of_limit` の `limit_vel=10.0`、`ang_vel_out_of_limit` の `limit_vel=6.0` など）で、これらはrad/s換算と推測されるが明記はされていない。
- **G1のhg状態メッセージにはworld座標系での位置・速度（odometry）フィールドが存在しない** ため、Nav2が要求する `odom` の情報源はSDK2の高レベル/低レベルstateには含まれておらず、別途（外部SLAM、LiDARオドメトリ、あるいはUnitree公式ROS2ブリッジ等）から供給する必要がある。

---

## 7. 設計上の示唆（Bridge実装への影響）

1. **速度指令はデフォルトで1秒しか有効でない「時限式」である可能性が高い**（`SetVelocity`/`Move` の `duration` デフォルト=1.0秒）。したがって:
   - Nav2からの`cmd_vel`をBridgeが中継する設計では、**`continous_move=False`（デフォルト挙動）のまま、Nav2の制御周期（通常20Hz前後）に合わせて`Move()`を継続的に呼び続ける**運用が、通信断時のフェイルセーフとして機能する可能性が高い（Bridgeプロセスが死ねば1秒以内に指令が失効し、ロボットは（推測だが）自動的に停止するはず）。
   - 逆に `continous_move=True`（`duration=864000`秒 ≒ 10日）を使う設計は、**Bridgeプロセスがクラッシュ・ハング・通信断した場合に、ロボットが指令された速度で歩き続け続けるリスク**が理論上ある。ユーザー仮説どおり、この設定は避けるか、必ず外部watchdogとセットで運用すべき。
   - ただし、`duration`経過後の正確な挙動（自動停止か、それとも直前速度を保持し続けるか）はSDK内に明記がないため、**実機での検証が必須**。安全のためBridge側では独自にwatchdogタイマーを持ち、一定時間`cmd_vel`が来なければ明示的に`StopMove()`または`Damp()`を呼ぶ設計を推奨する。
2. **`SetVelocity`は実在し、`Move`の下請け関数である**。Bridge実装では`Move(vx, vy, omega)`（3引数、continous_move=False）をそのまま周期送信する方式が最もシンプルかつSDKのデフォルト安全マージン（1秒）を活かせる。`SetVelocity`を直接叩いて`duration`を細かく制御する（例：Nav2周期の2〜3倍程度に設定する）選択肢も検討可能。
3. **C++版とPython版でメソッド名・シグネチャに差異がある**（`Squat`/`StandUp`/`BalanceStand`等）。Bridgeをどちらの言語で実装するかで使えるAPIが微妙に異なるため、実装言語決定後に該当バージョンのAPIで再確認すること。
4. **Nav2用のodometryはSDK2の高レベル/低レベルstateメッセージからは得られない**。`unitree_hg::SportModeState_`は`fsm_id/fsm_mode/task_id/task_time`のみで、Go2用`SportModeState_`にある`position`/`velocity`が存在しない。したがって:
   - Bridge（またはNav2スタック側）は、別途SLAM（LiDAR/カメラベース）やUnitree公式ROS2パッケージが提供するodometry相当のトピックから`nav_msgs/Odometry`を合成する必要がある。
   - IMUデータ（`imu_state`: quaternion, gyroscope, accelerometer, rpy）は取得できるため、姿勢（orientation）はSDK2から得られるが、位置(x, y)は別ソースが必須。
5. **低レベル制御における通信断時の危険性についてUnitree自身が明記**（`terminations.hpp`のコメント: 通信が切れた状態で古い指令を送り続けるとモータに「step信号」が入り激しい動きを引き起こす）。これは直接には低レベルLowCmd制御に関する注意だが、Bridgeの設計思想（「通信が切れたら安全側に倒す」）を補強する根拠として引用できる。
6. Unitree公式ヘルパー `lost_connection()`（`terminations.hpp`）は低レベルLowState購読用であり、高レベルLocoClient/sportサービス用には同等のヘルパーが提供されていない。**Bridge側で自前のタイムアウト監視ロジックを実装する必要がある**（例: 最後に`cmd_vel`を受信してから200ms以上経過したら`StopMove()`を送る、など）。
