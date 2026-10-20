"""固定した公式SDKのIDL・CRCを読み込む。ベンダーコードは複製しない。"""

from functools import lru_cache
import json
from pathlib import Path
import socket
import subprocess
import sys
from types import SimpleNamespace
from xml.sax.saxutils import quoteattr


def initialize(domain, interface):
    if not 0 <= domain <= 232:
        raise ValueError("DDS domainは0..232です")
    if interface not in [name for _, name in socket.if_nameindex()]:
        raise ValueError(f"network interfaceがありません: {interface}")
    from unitree_sdk2py.core import channel

    # SDKの公開設定テンプレートを、プロセス内の接続条件に合わせる。
    # loではunicast discoveryを明示し、OSのmulticast設定に依存させない。
    peers = (
        "<Discovery><ParticipantIndex>auto</ParticipantIndex>"
        "<MaxAutoParticipantIndex>20</MaxAutoParticipantIndex>"
        '<Peers><Peer Address="127.0.0.1"/></Peers></Discovery>'
        if interface == "lo"
        else ""
    )
    multicast = "<AllowMulticast>false</AllowMulticast>" if interface == "lo" else ""
    channel.ChannelConfigHasInterface = (
        '<CycloneDDS><Domain Id="any"><General><Interfaces>'
        f"<NetworkInterface name={quoteattr(interface)}/>"
        f"</Interfaces>{multicast}</General>{peers}</Domain></CycloneDDS>"
    )
    channel.ChannelFactoryInitialize(domain, interface)


@lru_cache
def load_sdk():
    root = Path(__file__).resolve().parents[3]
    source = root / "assets/cache/dds/sdk"
    lock = json.loads((root / "configs/dds.lock.json").read_text())
    if not (source / "unitree_sdk2py").is_dir():
        raise RuntimeError("先に bash EndToEnd/setup.sh --with-demo --with-dds を実行してください")
    commit = subprocess.check_output(["git", "-C", str(source), "rev-parse", "HEAD"], text=True).strip()
    if commit != lock["sdk_commit"]:
        raise RuntimeError(f"SDKの版がdds.lock.jsonと異なります: {commit}")
    sys.path.insert(0, str(source))
    from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
    from unitree_sdk2py.idl.default import (
        unitree_hg_msg_dds__LowCmd_,
        unitree_hg_msg_dds__LowState_,
        unitree_hg_msg_dds__IMUState_,
    )
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_, IMUState_
    from unitree_sdk2py.utils.crc import CRC

    return SimpleNamespace(
        initialize=initialize,
        Publisher=ChannelPublisher,
        Subscriber=ChannelSubscriber,
        LowCmd=LowCmd_,
        LowState=LowState_,
        IMUState=IMUState_,
        new_command=unitree_hg_msg_dds__LowCmd_,
        new_state=unitree_hg_msg_dds__LowState_,
        new_imu=unitree_hg_msg_dds__IMUState_,
        crc=CRC(),
        commit=commit,
    )
