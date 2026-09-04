# USBメモリへの Ubuntu フルインストール 手順書

実際に作業して得た知見と失敗をまとめたもの。次回はこの順番でやれば手戻りが少ない。

対象: **どのPC（Windows機含む）に挿しても起動する、データが保存される持ち運び用 Ubuntu**

---

## 0. 前提: 3つの方式の違いを最初に決める

| 方式 | 保存 | 作り方 | 所要 | 向き |
|---|---|---|---|---|
| **Live USB** | ❌ 再起動で全消去 | ISOを `dd` で書くだけ | 約30分 | インストーラ、レスキュー、お試し |
| **Live + 永続化** | △ 一部のみ | mkusb / Ventoy | 中 | 軽い持ち運び |
| **フルインストール** | ✅ 完全 | debootstrap または実機インストーラ | 約50分 | 本気の持ち運び環境 |

「USBを挿せばどのPCでもUbuntuが起動する」という要望は Live USB でも満たせるが、
**「作ったファイルが次回も残る」を期待しているなら Live USB では要件を満たさない。**
ここを最初に確認しないと作り直しになる。

---

## 1. 事前確認（ここを飛ばすと後で泣く）

### 1-1. 対象デバイスをシリアルで特定する

`/dev/sdb` のようなデバイス名は**挿し直すと変わる**。名前ではなくシリアルで縛る。

```bash
# 全ディスクの一覧（モデル・シリアル付き）
lsblk -o NAME,SIZE,TYPE,FSTYPE,LABEL,MOUNTPOINT,TRAN,MODEL,SERIAL

# 個別に確認
udevadm info -q property -n /dev/sdX | grep -E "ID_MODEL=|ID_SERIAL_SHORT="

# by-id から辿る（確実）
readlink -f /dev/disk/by-id/*<シリアル>*
```

⚠️ **システムディスクがUSB接続のこともある。** 今回の作業環境は起動ディスク自体がUSB
（`TRAN=usb`）だった。「USBだから外部メモリ」という判定は危険。
`findmnt -no SOURCE /` と `findmnt -no SOURCE /boot/efi` で稼働中のディスクを必ず除外する。

### 1-2. USBのリンク速度を確認する ★重要

```bash
for f in /sys/bus/usb/devices/*/speed; do
  d=$(dirname $f); p=$(cat $d/product 2>/dev/null)
  [ -n "$p" ] && printf "%-6s %6s Mbps  %s\n" "$(basename $d)" "$(cat $f)" "$p"
done
```

- `5000` / `10000` Mbps → USB 3.x。快適
- `480` Mbps → **USB 2.0**。フルインストールは数時間コース、日常利用も重い

**USB 3.0対応品でもUSB 2.0ポートに挿すと480 Mbpsになる。**
本体に「3.2 Gen1」と書いてあるのに480 Mbpsなら、まずポートを疑うこと
（コネクタ内部が青い / `SS` マークのあるポートへ挿し替える）。
今回これで 480 → 5000 Mbps になり、作業時間が大幅に短縮された。

補足: `ID_USB_INTERFACES` に `080662`（UAS）があるかも速度の目安になる。
`080650`（Bulk-Only）のみだと低速なことが多い。

### 1-3. 書き込み先の中身を必ず見る

```bash
lsblk -f /dev/sdX
ls -la /media/$USER/<ラベル>/     # マウントされていれば中身を確認
du -sh /media/$USER/<ラベル>/
```

---

## 2. 作業環境の準備

### 2-1. 非TTY環境での sudo

エディタやIDE経由の自動実行ではTTYがなく `sudo` がパスワードを読めない。
zenity をパスワード入力ヘルパーにすると解決する。

```bash
cat > ~/askpass.sh <<'EOF'
#!/bin/sh
exec zenity --password --title="sudo認証" --width=400 2>/dev/null
EOF
chmod 700 ~/askpass.sh

export SUDO_ASKPASS=~/askpass.sh DISPLAY=:1
sudo -A <コマンド>
```

- `/usr/libexec/gcr-ssh-askpass` は単体実行できないので使えない
- **長時間の処理は「1回の `sudo bash スクリプト` にまとめる」** のが最善。
  処理の途中でsudoのタイムスタンプ（既定15分）が切れると、
  バックグラウンド実行中に見えないダイアログが出て固まる
- どうしても細切れに実行するなら一時的に NOPASSWD を入れる。**作業後に必ず消すこと**

```bash
# 一時付与
sudo tee /etc/sudoers.d/99-temp >/dev/null <<'EOF'
<ユーザー名> ALL=(ALL) NOPASSWD: ALL
EOF
sudo chmod 440 /etc/sudoers.d/99-temp
sudo visudo -c                       # 構文チェック必須

# 作業後
sudo rm -f /etc/sudoers.d/99-temp
sudo -K && sudo -n true; echo $?     # 0以外なら解除できている
```

### 2-2. 必要パッケージ

```bash
sudo apt-get install -y debootstrap gdisk dosfstools rsync
sudo apt-get install -y qemu-system-x86 ovmf     # 検証用（後述、必須級）
```

### 2-3. アンマウントできないとき

ファイルマネージャがマウントを掴んでいることが多い。

```bash
lsof /media/$USER/<ラベル>          # 犯人を特定（大抵 nautilus）
fuser -vm /media/$USER/<ラベル>
sudo umount -l /dev/sdX1            # lazy umount で強制的に外せる
sudo udisksctl power-off -b /dev/sdX
```

---

## 3. 作成手順（debootstrap方式）

現在動いているUbuntuから直接構築する。ISOのダウンロードは不要。

### 3-1. パーティション（UEFI + Legacy BIOS 両対応のGPT）

```bash
DEV=/dev/sdX
sudo wipefs -a $DEV
sudo sgdisk --zap-all $DEV
sudo sgdisk -n1:0:+2M -t1:EF02 -c1:"BIOS boot"   $DEV   # 古いBIOS機用
sudo sgdisk -n2:0:+1G -t2:EF00 -c2:"EFI System"  $DEV   # UEFI用
sudo sgdisk -n3:0:0   -t3:8300 -c3:"Ubuntu root" $DEV
sudo partprobe $DEV; sudo udevadm settle

sudo mkfs.vfat -F32 -n UBUNTUEFI ${DEV}2
sudo mkfs.ext4 -F -L ubuntu-usb -m 1 ${DEV}3
```

`-m 1` で予約ブロックを1%に（既定5%だと数GB無駄になる）。

### 3-2. debootstrap

```bash
M=/mnt/ubuntu-usb
sudo mkdir -p $M
sudo mount -o noatime ${DEV}3 $M
sudo mkdir -p $M/boot/efi && sudo mount ${DEV}2 $M/boot/efi

sudo debootstrap --arch=amd64 \
     --components=main,restricted,universe,multiverse \
     noble $M http://jp.archive.ubuntu.com/ubuntu
```

`--components` を指定しないと universe が有効にならず `ubuntu-desktop` が入らない。

### 3-3. chroot 準備

```bash
for d in dev dev/pts proc sys run; do sudo mount --bind /$d $M/$d; done
echo "nameserver 1.1.1.1" | sudo tee $M/etc/resolv.conf

# chroot内でサービスが起動するのを防ぐ
printf '#!/bin/sh\nexit 101\n' | sudo tee $M/usr/sbin/policy-rc.d
sudo chmod +x $M/usr/sbin/policy-rc.d
```

### 3-4. chroot内の設定

`/etc/apt/sources.list.d/ubuntu.sources`（deb822形式）:

```
Types: deb
URIs: http://jp.archive.ubuntu.com/ubuntu
Suites: noble noble-updates noble-backports
Components: main restricted universe multiverse
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg

Types: deb
URIs: http://security.ubuntu.com/ubuntu
Suites: noble-security
Components: main restricted universe multiverse
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg
```

パッケージ導入:

```bash
apt-get update
apt-get install -y eatmydata          # ★ dpkgのfsyncを抑止。フラッシュメモリでは劇的に速くなる
E=eatmydata

$E apt-get install -y locales tzdata console-setup keyboard-configuration
ln -sf /usr/share/zoneinfo/Asia/Tokyo /etc/localtime
sed -i 's/^# *\(ja_JP.UTF-8\|en_US.UTF-8\)/\1/' /etc/locale.gen
locale-gen && update-locale LANG=ja_JP.UTF-8

$E apt-get install -y linux-generic initramfs-tools ubuntu-standard sudo
$E apt-get install -y grub-efi-amd64-signed shim-signed grub-pc-bin grub2-common
$E apt-get install -y ubuntu-desktop network-manager
$E apt-get install -y language-pack-ja language-pack-gnome-ja fonts-noto-cjk ibus-mozc
```

`grub-pc-bin` は `grub-efi` と共存できる（`grub-pc` 本体は競合するので入れない）。
これでUEFIとLegacy BIOSの両方に書き込める。

### 3-5. 持ち運びのための設定 ★ここが肝

```bash
# 全ハードウェアのドライバをinitramfsに含める（別のPCで起動するために必須）
sed -i 's/^MODULES=.*/MODULES=most/' /etc/initramfs-tools/initramfs.conf

# 存在しないswapへのレジューム待ちで30秒固まるのを防ぐ
echo "RESUME=none" > /etc/initramfs-tools/conf.d/resume

update-initramfs -c -k all
```

### 3-6. GRUB ★最大の落とし穴

```bash
# UEFI: EFI/BOOT/BOOTX64.EFI （リムーバブルメディア用のパス）
grub-install --target=x86_64-efi --efi-directory=/boot/efi \
             --bootloader-id=ubuntu --removable --no-nvram --recheck

# UEFI: EFI/ubuntu/ ← これも必須（理由は「4-1」参照）
grub-install --target=x86_64-efi --efi-directory=/boot/efi \
             --bootloader-id=ubuntu --no-nvram --recheck

# Legacy BIOS
grub-install --target=i386-pc --recheck /dev/sdX

update-grub
```

- `--removable` … PC側のNVRAMに登録せずUSB単体で起動できるようにする
- `--no-nvram` … **作業に使っているPCの起動設定を書き換えない**。これを忘れると自分のPCのブートエントリが汚れる

### 3-7. ユーザーとfstab

```bash
useradd -m -s /bin/bash <ユーザー名>
echo "<ユーザー名>:<パスワード>" | chpasswd
usermod -aG sudo,adm,dip,plugdev,lpadmin,cdrom <ユーザー名>
passwd -l root
```

fstabは**必ずUUIDで**書く（デバイス名は挿す場所で変わる）:

```bash
ROOT_UUID=$(blkid -s UUID -o value ${DEV}3)
EFI_UUID=$(blkid -s UUID -o value ${DEV}2)
cat > $M/etc/fstab <<FST
UUID=$ROOT_UUID  /          ext4  defaults,noatime,errors=remount-ro  0 1
UUID=$EFI_UUID   /boot/efi  vfat  umask=0077,noatime                  0 1
/swapfile        none       swap  sw                                  0 0
FST
```

### 3-8. 後始末

```bash
rm -f $M/usr/sbin/policy-rc.d
ln -sf ../run/systemd/resolve/stub-resolv.conf $M/etc/resolv.conf
for d in run sys proc dev/pts dev; do sudo umount -l $M/$d; done
sync; sudo umount $M/boot/efi; sudo umount $M; sync
```

---

## 4. ハマった点と対策（今回の反省）

### 4-1. ★ GRUBがレスキュープロンプトに落ちる（最重要）

**症状**: 起動すると `grub>` プロンプトだけが出てメニューが表示されない。

**原因**: Ubuntuの**署名済み** `grubx64.efi` には設定ファイルの参照先が
`/EFI/ubuntu` として**焼き込まれており、署名の都合で変更できない**。
一方 `grub-install --removable` は `/EFI/BOOT/` にしか `grub.cfg` を置かない。
結果、GRUBは設定を見つけられずフォールバックし、最終的にプロンプトへ落ちる。

**切り分け方**: `grub>` で `set` を実行して変数を見る。

```
grub> set
prefix='(hd0,gpt2)/boot/grub'    ← gpt2はESP。本来はgpt3(root)を指すべき
root='hd0,gpt2'                  ← search.fs_uuid が効いていない
```

`search.fs_uuid <UUID> rr` を手で叩いて `echo RESULT=$rr` が正しく返るなら、
ext4は読めており **ESP側のgrub.cfgが実行されていない** と確定できる。

**対策**: `--removable` あり・なしの **両方** で `grub-install` を実行し、
`EFI/BOOT/` と `EFI/ubuntu/` の両方に `grub.cfg` を配置する。

### 4-2. os-prober が作業PCの情報を埋め込む

**症状**: 生成された `grub.cfg` に作業に使ったPCのWindowsやUbuntuのメニュー項目が入る。
他のPCで起動すると存在しない項目が並び、選ぶとエラーになる。

**原因（自分のミス）**: `/etc/default/grub` には最初から
`#GRUB_DISABLE_OS_PROBER=false` という**コメント行**が存在する。そのため

```bash
grep -q GRUB_DISABLE_OS_PROBER /etc/default/grub || echo '...=true' >> /etc/default/grub
sed -i 's/^GRUB_DISABLE_OS_PROBER=.*/GRUB_DISABLE_OS_PROBER=true/' /etc/default/grub
```

は **grepがコメント行にマッチして追記されず、sedは行頭 `#` にマッチせず**、
どちらも空振りする。

**対策**: 削除してから追記する。パッケージごと消すのが確実。

```bash
sed -i "/GRUB_DISABLE_OS_PROBER/d" /etc/default/grub
echo "GRUB_DISABLE_OS_PROBER=true" >> /etc/default/grub
dpkg --purge --force-depends os-prober
update-grub
```

**検証**: 作業PCのUUIDが混入していないか必ず数える。

```bash
for u in $(lsblk -no UUID /dev/sda1 /dev/sda2 /dev/nvme0n1p1); do
  echo "$u → $(sudo grep -c "$u" $M/boot/grub/grub.cfg)件"     # 全て0であること
done
```

### 4-3. ★ ファイル検査だけでは不十分。必ず実起動テストをする

今回、ファイル構成の確認では 4-2 しか見つからず、**4-1（起動しない致命傷）は
QEMUで実際に起動させて初めて発覚した。**
「必要なファイルが正しい場所にある」ことと「起動する」ことは別物。

### 4-4. `pkill -f` が自分自身を殺す

`pkill -f "qemu-system-x86_64 -enable-kvm"` は、そのコマンドを実行している
シェル自身のコマンドラインにも文字列が含まれるためマッチして自滅する。
プロセス名で殺すか（`pkill -x`、ただし15文字までの制限あり）、
QEMUなら monitor に `quit` を送るのが確実。

### 4-5. その他

- ISO配布の国内ミラー（jaist/riken 等）は環境によっては到達できない。`releases.ubuntu.com` に素直にフォールバックする
- `debootstrap` 方式ならISOは**そもそも不要**。方式決定前にダウンロードを始めると無駄になる

---

## 5. 検証（QEMUで実起動テスト）★必須

`snapshot=on` を付けると書き込みが一時ファイルに逃げるので**USBの中身は変更されない**。

```bash
SP=/tmp/usbtest; mkdir -p $SP
cp /usr/share/OVMF/OVMF_VARS_4M.fd $SP/vars.fd

# (A) UEFI 通常起動
sudo qemu-system-x86_64 -enable-kvm -m 4096 -smp 2 -machine q35 \
  -drive if=pflash,format=raw,unit=0,readonly=on,file=/usr/share/OVMF/OVMF_CODE_4M.fd \
  -drive if=pflash,format=raw,unit=1,file=$SP/vars.fd \
  -drive file=/dev/sdX,format=raw,if=none,id=u,snapshot=on \
  -device qemu-xhci,id=xhci -device usb-storage,bus=xhci.0,drive=u \
  -display none -vga virtio -monitor unix:$SP/mon.sock,server,nowait -daemonize
```

**(B) Secure Boot**: firmware を `OVMF_CODE_4M.ms.fd` / `OVMF_VARS_4M.ms.fd` に変え、
`-machine q35,smm=on` と `-global driver=cfi.pflash01,property=secure,value=on` を追加。

**(C) Legacy BIOS**: `-drive if=pflash` の2行を丸ごと外す（SeaBIOSになる）。
`usb-storage` に `bootindex=0` を付ける。

### スクリーンショットの撮り方（ヘッドレス）

```bash
python3 - <<'EOF'
import socket, time
s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM); s.connect('/tmp/usbtest/mon.sock')
time.sleep(0.6); s.recv(65536)
s.sendall(b"screendump /tmp/usbtest/shot.png -f png\n"); time.sleep(2.5)
EOF
```

同じ要領で `sendkey` を送ればGRUBプロンプトを対話操作できる（`$` は `shift-4` など）。

**合格ライン**: (A)(B)(C) の3つすべてでログイン画面まで到達すること。

---

## 6. 2本目以降を複製する

再インストール（約50分＋ダウンロード数GB）より、rsync複製（約15分）が速い。

```bash
SRC=/dev/sdX; DST=/dev/sdY

# 複製先を同じレイアウトで作り直す（UUIDは自動で新規採番される）
sudo wipefs -a $DST && sudo sgdisk --zap-all $DST
sudo sgdisk -n1:0:+2M -t1:EF02 -n2:0:+1G -t2:EF00 -n3:0:0 -t3:8300 $DST
sudo partprobe $DST; sudo udevadm settle
sudo mkfs.vfat -F32 -n UBUNTUEFI ${DST}2
sudo mkfs.ext4 -F -L ubuntu-usb -m 1 ${DST}3

sudo mount -o ro ${SRC}3 /mnt/src        # ★複製元は読み取り専用で保護
sudo mount ${DST}3 /mnt/dst
sudo mkdir -p /mnt/dst/boot/efi && sudo mount ${DST}2 /mnt/dst/boot/efi

sudo rsync -aHAXx --numeric-ids --info=progress2 \
     --exclude='/swapfile' /mnt/src/ /mnt/dst/
```

`-x` でファイルシステム境界を越えないため、ESPは複製されない（後でgrub-installが作る）。

**複製後に必ず個体差を作ること:**

```bash
# 1. fstab を新しいUUIDに書き換え（これを忘れると起動しない）
# 2. machine-id をリセット（同一だとDHCPのアドレス割当が衝突する）
sudo truncate -s 0 /mnt/dst/etc/machine-id
sudo rm -f /mnt/dst/var/lib/dbus/machine-id
sudo ln -sf /etc/machine-id /mnt/dst/var/lib/dbus/machine-id
# 3. SSHホスト鍵を削除（初回起動時に再生成される）
sudo rm -f /mnt/dst/etc/ssh/ssh_host_*
# 4. swapfile を作り直す
# 5. chroot して update-initramfs -u -k all と grub-install 3種 と update-grub
```

UUIDを同じままにすると、2本同時に挿したときGRUBがどちらを掴むか不定になる。

---

## 7. 実行前チェックリスト

- [ ] 方式（Live / フルインストール）を利用者と合意した
- [ ] 対象デバイスを**シリアル**で特定した
- [ ] `findmnt /` と `findmnt /boot/efi` で稼働ディスクを除外した
- [ ] リンク速度が 5000 Mbps 以上（480ならポートを変える）
- [ ] 書き込み先の中身を確認し、消えて困るデータがないと確認した
- [ ] 長時間処理を1回のsudoにまとめた
- [ ] `MODULES=most` を設定した
- [ ] `RESUME=none` を設定した
- [ ] `grub-install` を `--removable` あり・なし・i386-pc の**3回**実行した
- [ ] `--no-nvram` を付けた（作業PCのブート設定を守る）
- [ ] os-proberを削除し、grub.cfgに作業PCのUUIDが**0件**であることを確認した
- [ ] fstabがUUID表記になっている
- [ ] **QEMUで (A)UEFI (B)SecureBoot (C)LegacyBIOS の3パターンとも起動を確認した**
- [ ] 一時的に入れたNOPASSWD設定を削除した
- [ ] アンマウントと `sync` を完了した

---

## 8. 利用時のメモ

- 起動は電源投入直後に `F12`（DELL/Lenovo）、`F9`（HP）、`ESC`、`F2` などでブートメニューを開いてUSBを選ぶ
- **BIOS設定そのものを書き換えると、BitLockerで暗号化されたWindows機では次回起動時に回復キーを要求されることがある。** ブートメニューからの一時選択なら通常は安全
- Ubuntu公式ISO/shimは署名済みなのでSecure Bootを無効化する必要はない
- USB 2.0ポートに挿すと目に見えて重い。可能な限りUSB 3.0ポートを使う
