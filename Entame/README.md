# Entame（エンタメ系の動作）

G1にエンタメ系の動作（ダンス・ジェスチャー・パフォーマンス等）をさせる機能。

## 構成

- `sim/` — シミュレーション（`G1_HuggingFace/`と同じMuJoCo/lerobotスタック）上での
  動作の検証
- `real/` — 実機G1での実行

## 環境

`G1_HuggingFace/venv/`（操作PC側）・G1本体側のPython 3.12 conda環境（`lerobot`）を
共通で使う想定。ネットワーク接続・疎通確認は`Common/network/`を参照。
**Dockerは使わない**（→ ルート`README.md`の「開発環境の使い分け」）。

## 実現方法の選択（着手前に決めること）

G1には制御方式が2つあり、**同時には使えない**（片方がロボットの高レベル制御を占有する）。
どちらを使うかで実装がまったく変わる。

| | SDK方式（`sport_mode`） | lerobot方式 |
|---|---|---|
| 中身 | Unitree製の歩行コントローラ（`LocoClient`） | 独自ONNXポリシーによる低レベル関節制御 |
| 動作の自由度 | **Unitreeの固定動作のみ**。独自モーションは載せられない | **独自モーションを作れる** |
| 導入コスト | 低い。G1本体側に何も置かなくてよい | 高い。50Hzの関節指令を流す必要がある |

手を振るなどUnitreeが用意している動作で足りるならSDK方式で済む。
独自の振り付けが要るならlerobot方式になる。

`Navigation/`の巡回はSDK方式を使うため、**巡回中にエンタメ動作を挟む場合はモード切替が
発生する**。切替時に制御主体が入れ替わるため転倒リスクがある点は、Step2/3の
VLA動作と共通の課題。動作の事後条件は直立姿勢へ戻すこと。

## 進め方

1. `sim/`でロジックを作り、シミュレーション上で動作確認
2. `real/`で実機G1に接続し、同じロジックが実機でも動くか確認

失敗した内容は`FAILURES.md`に記録する。

## 開発ルール

- 依存は`Common/`配下のみ。他チームのフォルダ（`Mapping/`, `Navigation/`,
  `Perception/`等）は import しない
- 共通で使うもの（地図作成・自己位置推定・巡回・安全停止）は`Common/`に置く
- エンタメ側に閉じるもの（ゲームのルール判定・参加者向けUI・演出）は
  `Entame/`配下に閉じる

### セットアップ手順

```bash
git clone <repo-url>
cd G1_Hackason
git checkout Dev/Entame

# 作業ブランチを作成（Dev/Entame という ref が既に存在するため
# `Dev/Entame/<名前>` は git の仕様上作成できない。別名にする）
git checkout -b feature/entame-<自分の作業名>

# 作業ツリーを Common/ と Entame/ だけに絞る（他チームのフォルダを誤って
# 編集しないための保険。git管理・履歴には影響しない、ローカル表示のみの設定）
git sparse-checkout init --cone
git sparse-checkout set Common Entame
```

作業が終わったら`feature/entame-<名前>`ブランチから`Dev/Entame`へPRを出す。

## 状態

未着手。実現方法（SDK方式 / lerobot方式）も未決定。
