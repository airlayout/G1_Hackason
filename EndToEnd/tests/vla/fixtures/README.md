π0.5の互換性検証に使う公開checkpointの設定・正規化統計とデータセット仕様。
モデル本体の重みは含まない。設定・統計は変更せず保存する。
`weights_header.json`は本体safetensorsのヘッダーから抽出した813テンソルの名前・形状・dtypeで、重みの値は含まない。

- モデル: https://huggingface.co/xiaopeng-wu/pi05_unitree_g1/tree/580238372153c51ad564b894d8fea736ab4cdeb8
- データセット: https://huggingface.co/datasets/nepyope/unitree_box_move_blue_full/tree/7cbc4619475e7607701b2ddf37f695ef85f69ec3
- ライセンス: 各配布元のApache-2.0。モデルカードとデータセットカードを参照。
