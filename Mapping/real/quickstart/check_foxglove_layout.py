"""Foxglove のレイアウトが参照するトピックが、実際に advertise されているか突き合わせる。

ブラウザで開いて「パネルが空」になる原因はほぼこれ（存在しないトピック名）なので、
読み込ませる前にここで潰す。ブラウザも機体の移動も要らない。

    python3 check_foxglove_layout.py foxglove/g1_nav.json --host 192.168.123.164

⚠️ **アクションのトピックは既定では出ない。** `/navigate_to_pose/_action/feedback`
は名前に "_action" を含むため ROS 2 では hidden 扱いで、foxglove_bridge の
`include_hidden:=true` が要る。さらに **PC2 側に nav2_msgs が入っていないと**
schema を作れず、include_hidden を立てても広告されない（2026-09-13 に両方踏んだ）。
`recoveries` はこのトピックにしか出ないので、合否 3-3 を見るには両方が要る。

出力する側のトピック（poseTopic / pointTopic）は、購読者が居なくても
publish できるので「実在しない」に数えない。
"""
import argparse, json, pathlib, sys

from foxglove_ws import FoxgloveClient


def wanted_topics(layout: dict) -> tuple[dict, set]:
    """{topic: 要求元パネル} と、出力専用トピックの集合を返す。"""
    wanted, publish_only = {}, set()
    for panel_id, cfg in layout.get("configById", {}).items():
        for topic in cfg.get("topics", {}):
            wanted.setdefault(topic, panel_id)
        publish = cfg.get("publish", {})
        for key in ("poseTopic", "pointTopic", "poseEstimateTopic"):
            topic = publish.get(key)
            if topic:
                wanted.setdefault(topic, "%s(publish)" % panel_id)
                publish_only.add(topic)
        for path in cfg.get("paths", []):
            # Plot は "/topic.field.sub" 形式。最初のドットまでがトピック名
            wanted.setdefault("/" + path["value"].lstrip("/").split(".")[0], panel_id)
        if cfg.get("topicPath"):
            wanted.setdefault(cfg["topicPath"].split(".")[0], panel_id)
        if panel_id.startswith("Log!"):
            wanted.setdefault("/rosout", panel_id)
    return wanted, publish_only


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("layout", help="Foxglove のレイアウト JSON")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8765)
    args = ap.parse_args()

    layout = json.loads(pathlib.Path(args.layout).read_text())
    wanted, publish_only = wanted_topics(layout)

    client = FoxgloveClient(args.host, args.port)
    channels = client.collect_channels()
    client.close()

    live, dead, outputs = [], [], []
    for topic, panel in sorted(wanted.items()):
        if topic in channels:
            live.append((topic, panel))
        elif topic in publish_only:
            outputs.append((topic, panel))
        else:
            dead.append((topic, panel))

    print("レイアウト %s / bridge のチャンネル %d 件\n"
          % (pathlib.Path(args.layout).name, len(channels)))
    print("■ 届く (%d)" % len(live))
    for topic, _ in live:
        print("    OK   %-46s %s" % (topic, channels[topic].get("schemaName", "")))
    if outputs:
        print("\n■ 出力専用 (%d) — 購読者が居なくても publish できるので問題ない" % len(outputs))
        for topic, panel in outputs:
            print("    --   %-46s %s" % (topic, panel))
    print("\n■ 届かない (%d) — このパネルは空になる" % len(dead))
    for topic, panel in dead:
        print("    NG   %-46s 要求元 %s" % (topic, panel))
    return 1 if dead else 0


if __name__ == "__main__":
    sys.exit(main())
