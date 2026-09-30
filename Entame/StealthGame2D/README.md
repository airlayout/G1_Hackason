# Stealth Game PoC

`Entame/docs/win-conditions.md` の「①盗む」パターンをブラウザで素早く試すMVP。
「警備員1人の視野を避けてターゲットを取り、STARTへ戻る」体験が面白いかを検証する。

## 起動方法

ES Modules を使っているため `file://` では動かない。簡易HTTPサーバで配信する。

```bash
cd Entame/StealthGame2D
python3 devserver.py
```

`python3 -m http.server 8000` でも閲覧・プレイ自体は可能だが、エディタの
「💾 JSONを保存」がサーバーへの直接保存に対応していないため、その場合は
ブラウザへのダウンロードにフォールバックする（後述）。`devserver.py` を
使うと `stages/` フォルダへ自動保存される。

ブラウザで `http://localhost:8000` を開く。

## スケール（実世界換算）

実機G1・人の歩行速度に合わせ、1m = 80px で換算している。

| 項目 | 値 |
|---|---|
| 部屋サイズ | 5m〜25m四方の範囲でステージごとに調整可能（デフォルト10m×10m = 800px×800px） |
| プレイヤー(人)の歩行速度 | 1.0 m/s（= 80px/sec、`player.js`） |
| 警備員(G1)の歩行速度 | 0.5 m/s（= 40px/sec、各`stages/*.json`の`guard.moveSpeed`） |

視野角・視認距離の初期値の根拠（水平画角70°のWebカメラ想定など）は
`Entame/docs/g1-detection-spec.md`を正とする。数値を変更する場合は両方を
更新すること。

## 操作方法

- 矢印キー（↑↓←→）のみで移動
- ターゲットに触れると自動取得
- 取得後にSTART地点へ戻るとCLEAR
- 警備員の視野（扇形）に、遮蔽物なしで入ると即GAME OVER（一発アウト）

## ステージデータ（stages/*.json）

ステージデータは `stages.js` ではなく、`stages/` 配下の**ステージ1つ＝JSONファイル1つ**
という構成で管理する。

```
stages/
  manifest.json    # 読み込むJSONファイル名の一覧（配列）
  stage-1.json
  stage-2.json
  stage-3.json
```

`main.js` / `editor.js` は起動時に `stagesLoader.js` の `loadStages()` を呼び、
`manifest.json` に載っているファイルを順にfetchして読み込む。

`devserver.py` で起動している場合、エディタから保存すると `POST /api/save-stage`
経由で `stages/stage-N.json` の作成・上書きと、`stages/manifest.json` への
ファイル名追記（新規ステージの場合のみ）が自動で行われる。
`python3 -m http.server` など保存APIを持たないサーバーで開いている場合は、
ブラウザへのJSONファイルダウンロードにフォールバックするので、手動で
`stages/` に置いて `manifest.json` を編集する必要がある。

## ステージエディタ

`editor.html` でSTART/TARGET位置、警備員の初期位置・巡回ルート、壁の配置を
GUIで調整できる。LEVEL SELECT画面の「ステージエディタを開く」リンクから開く。

- Width(m) / Height(m) でエリアサイズを5m〜25mの範囲で調整できる（範囲外の値は自動でクランプされる）
- Move Speed(m/s) / Vision Distance(m) は実世界の単位で入力する（保存されるJSONは1m=80pxでpx単位に変換される）
- モードボタンで配置対象を切り替え、Canvas上をクリック（壁はドラッグ）で配置
- 巡回ルートの各区間が壁と衝突する場合、赤線＋警告メッセージで即座に警告する
  （「警備員が壁にスタックする」バグを作った時点で気づけるようにするため）
- 既存のSTAGE 1〜3を読み込んで編集することも、Newで新規作成することも可能
- **▶ テストプレイ**: 保存前でも、編集中のステージ設定でその場にゲーム本体と
  同じロジック（移動・巡回・視野判定・CLEAR/GAME OVER）を動かして確認できる。
  矢印キーで操作し、「■ 編集に戻る」でエディタに戻る
- **💾 JSONを保存**: 編集中のステージ1件を `stage-<ID>.json` として保存する。
  `devserver.py` 起動時は `stages/` に直接書き込まれ、`manifest.json` も
  自動更新される。保存APIがない場合はブラウザへのダウンロードにフォールバック
  し、手動で `stages/` に配置する手順をダイアログで案内する
- 「コードをテキスト表示」で同じ内容をテキストエリアにも表示する（コピー用）

## ファイル構成

- `devserver.py` — 静的ファイル配信 + ステージ保存API（`POST /api/save-stage`）を持つ簡易サーバー
- `index.html` / `style.css` — ゲーム画面とスタイル
- `main.js` — 状態遷移（LEVEL_SELECT → PLAYING → CLEAR/GAME_OVER）とゲームループ
- `stagesLoader.js` — `stages/manifest.json` 経由でステージJSON群を読み込む
- `stages/` — ステージデータ本体（JSON、1ファイル1ステージ）
- `player.js` — プレイヤーの移動・壁衝突・ターゲット取得判定
- `guard.js` — 警備員の巡回移動と「距離＋視野角＋遮蔽物」の発見判定
- `collision.js` — 円と壁の当たり判定、レイと壁のスラブ法交差判定（player/guard/editor共通）
- `render.js` — ゲーム画面のCanvas描画
- `editor.html` / `editor.css` / `editor.js` — ステージエディタの画面・スタイル・DOM配線
- `editorState.js` / `editorValidate.js` / `editorRender.js` / `editorExport.js` / `editorPlay.js`
  — エディタの状態・検証・描画・JSON出力・テストプレイ

## MVPで未実装のもの

複数警備員、複雑なAI、攻撃・戦闘、アイテム・インベントリ、音声、
オンライン機能、ランキング、複雑なアニメーション、高品質グラフィック。
