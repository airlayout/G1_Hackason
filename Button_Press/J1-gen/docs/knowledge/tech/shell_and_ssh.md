# シェル・SSH の注意点

## `pkill -f` が、実行したシェル自身を止める

- **日付**: 2026-10-22
- **プロジェクト**: G1 ボタン押し（Button_Press/J1-gen）
- **前提バージョン**: Ubuntu 24.04、procps-ng 4.0.4（pkill）、bash 5.2
- **状況**: バックグラウンドで起動したプロセス（模擬ロボット）を、同じコマンドの最後で `pkill -f "sim_robot_server"` や
  `pkill -f "sim_robot_serve[r].py"` で止めようとした。3 回とも、コマンド全体が終了コード 144 で止まった
- **原因**: `pkill -f` はコマンドライン全体に対して照合する。`bash -c "...起動... ; pkill -f パターン"` のように、
  同じシェルのコマンドラインに起動したコマンドの文字列が入っていると、そのシェル自身も一致して止められる。
  `[r]` で自分の pkill の行を避ける定番の方法も、起動の部分（`sim/sim_robot_server.py`）が同じ行にあると効かない
- **対処**: 起動と停止を別のコマンドに分ける。止めるときは PID（`$!` を保存する、`pgrep` で確かめてから `kill <PID>`）か、
  バックグラウンドのタスクを止める仕組みを使う。止めたあとは `pgrep -af ... | grep -v pgrep` で残っていないかを確かめる
- **教訓**: `pkill -f` のパターンは、それを書いたシェルのコマンドラインにも一致しうる。同じコマンドの中で起動と pkill をしない

## `~/.ssh/config` の `PubkeyAuthentication no` で、登録した鍵が使われない

- **日付**: 2026-10-21
- **プロジェクト**: G1 ボタン押し（ラボ PC → G1 の PC2）
- **前提バージョン**: OpenSSH 9.6p1（Ubuntu 24.04）
- **状況**: `ssh-copy-id g1` で鍵を登録できた（「Number of key(s) added: 1」）のに、`ssh -o BatchMode=yes g1` は
  `Permission denied (publickey,password)` になった
- **原因**: `~/.ssh/config` の `Host g1` に `PreferredAuthentications password` と `PubkeyAuthentication no` が書いてあった。
  ssh-copy-id は鍵を相手に置くだけで、自分の設定は変えない
- **対処**: 設定ファイルを変えずに、コマンドで上書きする:
  `ssh -o PubkeyAuthentication=yes -o PreferredAuthentications=publickey -i ~/.ssh/<鍵> g1`
  （コマンドの `-o` は設定ファイルより優先される）。`ssh -v` の `identity file` と `Authentications that can continue` を見ると分かる
- **教訓**: 鍵を登録しても使われないときは、まず `~/.ssh/config` の該当の Host を見る

## パスワードを聞く ssh / scp は、Claude Code の `!` では動かない

- **日付**: 2026-10-21
- **プロジェクト**: G1 ボタン押し
- **前提バージョン**: OpenSSH 9.6p1、Claude Code の `!`（シェル実行）
- **状況**: `! scp g1:... ...` が `ssh_askpass: exec(/usr/bin/ssh-askpass): No such file or directory` → `Permission denied` で失敗した
- **原因**: `!` の実行には端末が無く、ssh はパスワードを ssh-askpass で聞こうとするが、入っていない
- **対処**: パスワードが要るコマンドは、ふつうのターミナルで実行してもらう。何度も要るなら鍵で入れるようにする（上の項目）
- **教訓**: 対話の入力が要るコマンド（パスワード・Enter 待ち）は、`!` や Claude のシェルではなく、ふつうのターミナルで

## Claude が G1 の PC2 に ssh する手順（鍵の登録・オプション・起動したプロセスの扱い）

- **日付**: 2026-10-31
- **プロジェクト**: G1 ボタン押し（ラボ PC → G1 の PC2。有線 192.168.123.164、ユーザー `unitree`）
- **前提バージョン**: OpenSSH 9.6p1（Ubuntu 24.04）、Claude Code の Bash ツール
- **状況**: Claude に PC2 のサーバ（`rgbd_server`）の起動・停止・ログ確認をさせたかったが、ラボ PC の鍵は PC2 に登録されておらず、
  `ssh -i ~/.ssh/id_ed25519_humanoid_expo unitree@192.168.123.164` は `Permission denied (publickey,password)` だった
- **対処（鍵の登録）**:
  1. ラボ PC で公開鍵を出す: `cat ~/.ssh/id_ed25519_humanoid_expo.pub`（1 行）
  2. パスワードで入れる別のターミナルで、PC2 の `~/.ssh/authorized_keys` にその 1 行を足す:
     `echo '<1 行そのまま>' >> ~/.ssh/authorized_keys`。`ssh-copy-id` は、パスワードを入力する端末が要るので `!` では動かない（上の項目）
  3. 登録後も、`~/.ssh/config` の `Host g1 / g1-jetson / 192.168.123.164` に `PubkeyAuthentication no` があるので、次のオプションを毎回付ける:
     ```bash
     ssh -o BatchMode=yes -o ConnectTimeout=8 -o PreferredAuthentications=publickey -o PubkeyAuthentication=yes \
       -i ~/.ssh/id_ed25519_humanoid_expo unitree@192.168.123.164 '<コマンド>'
     ```
     （`BatchMode=yes` を付けると、鍵が通らないときにパスワードを待たずに失敗するので、Claude が固まらない）
- **はまった点**:
  - **公開鍵を `…` で省略した例を書いて、そのまま実行された。** `authorized_keys` に `ssh-ed25519 …` という無効な行が入った
    （害はない。`sed -i '/…$/d'` で消した）。コマンドに貼ってもらう文字列は、省略せずに完全な形で出す
  - `ssh host 'cmd &'` で起動したプロセスの出力を ssh が掴み続けて、ssh が返らない（Bash ツールの 120 秒のタイムアウトで、
    バックグラウンドに回された）。プロセス自体は PC2 で動き続ける。起動は
    `( timeout 20 ssh ... 'cd ~/dir && setsid nohup <コマンド> > log 2>&1 < /dev/null &' ; true )` とし、動いているかは**別の ssh** で
    `pgrep -af <名前> | grep -v pgrep` と `tail log` で確かめる
  - `ssh host 'pgrep -af 名前'` は、自分の ssh の `bash -c ...` の行も表示する（`grep -v pgrep` で除く）。`pkill -f` を同じ行に書かない（上の項目）
- **教訓**: Claude から PC2 を操作するには、鍵の登録（人がパスワードで 1 回）と、設定の上書きオプションが要る。
  起動したプロセスは、標準入出力を切り離して（`setsid nohup ... < /dev/null`）、確認は別の接続で行う

## Claude には、ユーザーのターミナルの出力が見えない

- **日付**: 2026-10-31
- **プロジェクト**: G1 ボタン押し
- **状況**: ユーザーが別のターミナルで `auto_press.py --execute` を実行した結果を、Claude は直接見られない
- **対処**: 実行の記録はディスクに残る（`_local/button_press/auto/*.json`、`runs/<日時>_*/events.jsonl`・`body_motion.csv`・`lowstate.csv`）。
  Claude はそこから結果（終了コード・進んだ距離・中止の理由・体の動き）を読める。ターミナルの文面そのものが要るときは、
  貼ってもらい、`_local/button_press/terminal_logs/<日付>_*.log` に保存する（その日の全体を 1 つのファイルに追記する）
- **教訓**: 実機日は「何を見たか」を人が口頭で言うので、Claude の側では、保存されたログで裏を取ってから答える
