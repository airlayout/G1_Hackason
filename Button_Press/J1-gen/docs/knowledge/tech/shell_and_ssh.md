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
