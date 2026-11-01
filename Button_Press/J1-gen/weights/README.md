# エレベーターボタンYOLO final8

`button_final8.pt` は Jigen の `Dev/ButtonPress` から取り込んだ最終版。
元と同じ配置で、実機とシミュレーションのボタン用YOLOから共用する。

- [取得元](https://github.com/airlayout/G1_Hackason/blob/8bcbbd7275cd4545d8abd14279226b3fd91b78fc/Button_Press/J1-gen/weights/button_final8.pt)
- 元コミット: `8bcbbd7275cd4545d8abd14279226b3fd91b78fc`
- Git blob: `7d5eeab147a6fa786078a741e62d4e4ba81730c3`
- SHA256: `7795e946b5331f17160ea688ef7ad2d8ea10cd59c93ef420f18bd75ca62eec41`
- サイズ: 5,370,181 bytes、クラス: `up` / `down`
- 標準推論設定: 640画素、信頼度0.4

ラボのエレベーターを撮影した8本の動画から726枚と、シミュレーション80枚で学習した重み
（2026-10-31、11エポック目）。学習データと途中の重みは同梱しない。

[元の検証記録](https://github.com/airlayout/G1_Hackason/blob/8bcbbd7275cd4545d8abd14279226b3fd91b78fc/Button_Press/J1-gen/weights/README.md)
では、Jigenのエージェントによるbasic・realistic・模擬DDSの練習20試行がそれぞれ20/20。
この成績はManipulation側の制御を検証したものではない。G1頭カメラの実画像による精度と
実機での押下成功は別途確認が必要。ここには指定された `.pt` 版を収録する。
