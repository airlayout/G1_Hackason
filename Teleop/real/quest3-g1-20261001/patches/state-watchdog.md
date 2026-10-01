# パッチ: state-watchdog

**2026-09-30 の事故（再接続時に全29関節が飽和トルクで跳ねた）への対策。**

`xr_teleoperate` に当てる。適用先は `~/xr_teleoperate/teleop/robot_control/robot_arm.py`。

> **このパッチが当たっていない環境で実機を動かさないこと。**

## 何を直すか

素の `xr_teleoperate` の送信ループ `_ctrl_motor_state()` には、
**ロボットからの状態報告（`rt/lowstate`）の鮮度チェックが無い。**

```python
while True:
    ...
    self.msg.crc = self.crc.Crc(self.msg)
    self.lowcmd_publisher.Write(self.msg)   # ← 届いているか確認せず送り続ける
```

そのため通信が切れても 250Hz で送信を続け、**復帰した瞬間に「途絶前に凍結した目標」へ
全関節を引き戻そうとして飽和する**（脚 kp=300 / 腕 kp=80）。

腕の速度制限 `clip_arm_q_target()` も効かない。比較対象の `get_current_dual_arm_q()` が
**途絶時点で凍結した実測値**を返すため、「偏差ほぼゼロ＝制限不要」と誤判定する。

## どう直すか

1. 受信時刻を記録する（`_subscribe_motor_state()` 内）
2. 送信の直前に鮮度を見る。**0.2秒** を超えていたら送信せず、**プロセスを強制終了する**

**再接続による自動復帰はさせない（意図的な fail-closed）。** 復帰させると同じ事故が
起きるため、人が状態を確認してから起動し直す形にする。

子プロセスも終了させる。残るとポート8012を掴んだままになり、次回の起動を妨げる
（実際にこれで1度つまずいた）。

## 当てる場所

G1 の3クラス（`safe-startup` パッチと同じ範囲。H1/H2/R1 は対象外）。

- `G1_29_ArmController`
- `G1_29_Arm_Internal_Dex1_Controller`
- `G1_23_ArmController`

### 受信側（各クラスの `_subscribe_motor_state`）

```python
                self.lowstate_sub_ready = True
                # PATCH(state-watchdog): 受信時刻。送信側の鮮度判定に使う。
                self.lowstate_last_time = time.time()
            time.sleep(0.002)
```

### 送信側（各クラスの `_ctrl_motor_state`、`Write` の直前）

```python
            # PATCH(state-watchdog): lowstate 途絶中は送信しない。
            _wd_last = getattr(self, "lowstate_last_time", None)
            if _wd_last is not None:
                _wd_age = time.time() - _wd_last
                if _wd_age > STATE_WATCHDOG_TIMEOUT_S:
                    _state_watchdog_fail_closed(_wd_age)

            self.msg.crc = self.crc.Crc(self.msg)  # PATCH(state-watchdog)
            self.lowcmd_publisher.Write(self.msg)
```

### モジュール直下

`STATE_WATCHDOG_TIMEOUT_S = 0.2` と `_state_watchdog_fail_closed()` を追加。
`os` / `signal` / `sys` の import も足す（素の `robot_arm.py` には無い）。

## 当て方

`g1-starter-kit` の `setup/apply_patches.py` に `state-watchdog` として登録してある。
**冪等（二重適用しない）・`--revert` で完全に戻せる。**

```bash
cd ~/g1-starter-kit
python3 setup/apply_patches.py --dry-run              # 当たるか確認
python3 setup/apply_patches.py --only state-watchdog  # 当てる
python3 setup/apply_patches.py --only state-watchdog --revert   # 戻す
```

## 検証

### 1. 自己テスト（実機不要）

`tools/state_watchdog_selftest.py` が、パッチ後の `_ctrl_motor_state()` を
スタブの publisher で実際に動かし、鮮度だけを人工的に古くして挙動を見る。

```
✅ パッチが適用されている
✅ 鮮度が新しい間は送信される（誤発火しない）   ← 0.15秒で73回送信
✅ 途絶で強制終了する（終了コード 1）
✅ 途絶を知らせるメッセージが出る
✅ 送信を止めてから落ちている
結果: PASS
```

### 2. 冪等性と可逆性

- 2回目の適用で全8件が「既に適用済み」
- `--revert` → 再適用で **sha256 がバイト単位で一致**
- 既存パッチ（`safe-startup` 等）への影響なし

### 3. 実機での発火

**未検証。** 自己テストはスタブに対するもので、実機・実DDS相手での発火は
確認していない。次に実機を触るとき、テレオペ中に意図的に有線を抜いて
「0.2秒で落ちること」「再接続しても無反応なこと」を確認する。
**パッチがある状態なら、この試験自体は安全に行える。**

## 開発の過程で作り込んだ不具合（記録）

最初の実装では `before`（置換前の文字列）が適用後も残る書き方をしていたため、
**二重適用されるバグ**を作り込んだ。2回目の実行で「既に適用済み」にならず
「applied」と出たことで検出した。

`apply_patches.py` 自身のコメントがこの罠を警告していた。

> 挿入型なので before に「置換後には存在しなくなる範囲」を含める。
> アンカー行だけにすると置換後もそれが残り、未適用と誤判定して二重挿入される。

**冪等性は「2回流して結果が変わらないこと」を実際に確かめるまで信用しない。**
