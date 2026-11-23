"""MASt3R の推論を fp16 で回す差し込み（生の速さを上げる。上流の MASt3R-SLAM・MASt3R の中身は書き換えない）。

上流は読み取り部・復号部を fp32（行列は TF32）、頭（DPT と記述子の MLP）を autocast を切って fp32 で回す。
11-17 に OMEN（RTX 5060 Ti）で 1 こまの追跡を測ると、頭を含む復号が 47.7 ms で一番重かった（tests/bench_mast3r.py）。

ここでは:
- 読み取り部（_encode_image）と復号部（_decoder）を fp16 の autocast で回し、読み取りの出力は fp32 に戻す
  （キーフレームの箱は fp32。後ろの照合の CUDA の部品も fp32 を待つ）
- 頭は DPT と MLP だけを fp16 で回し、指数を取る後処理（点・確からしさ・記述子の正規化）は fp32 に戻してから行う
  （確からしさは 1+exp(x) なので fp16 のままだと大きい値で溢れうる）
- 畳み込みは cudnn.benchmark で、その大きさに一番速い解き方を選ばせる

差し込みはクラスに入れる。spawn で起こした子のプロセス（後ろの最適化）はクラスを読み直すので、子でも enable() を呼ぶ。
"""

from __future__ import annotations

import torch
import torch.nn.functional as F

_ENABLED = False


def enable() -> bool:
    """差し込む（何度呼んでもよい）。差し込んだら True"""
    global _ENABLED
    if _ENABLED:
        return True
    from mast3r.catmlp_dpt_head import Cat_MLP_LocalFeatures_DPT_Pts3d as Head
    from mast3r.model import AsymmetricMASt3R

    torch.backends.cudnn.benchmark = True
    orig_encode = AsymmetricMASt3R._encode_image
    orig_decoder = AsymmetricMASt3R._decoder

    def _encode_image(self, image, true_shape):
        with torch.autocast("cuda", dtype=torch.float16):
            x, pos, other = orig_encode(self, image, true_shape)
        return x.float(), pos, other

    def _decoder(self, f1, pos1, f2, pos2):
        with torch.autocast("cuda", dtype=torch.float16):
            return orig_decoder(self, f1, pos1, f2, pos2)

    def head_forward(self, decout, img_shape):
        # 上流の Cat_MLP_LocalFeatures_DPT_Pts3d.forward と同じ手順。DPT と MLP だけ fp16
        H, W = img_shape
        with torch.autocast("cuda", dtype=torch.float16):
            pts3d = self.dpt(decout, image_size=(H, W))
            cat_output = torch.cat([decout[0], decout[-1]], dim=-1)
            local_features = self.head_local_features(cat_output)
        pts3d, local_features = pts3d.float(), local_features.float()
        B = cat_output.shape[0]
        local_features = local_features.transpose(-1, -2).view(B, -1, H // self.patch_size, W // self.patch_size)
        local_features = F.pixel_shuffle(local_features, self.patch_size)
        out = torch.cat([pts3d, local_features], dim=1)
        if self.postprocess:
            out = self.postprocess(
                out,
                depth_mode=self.depth_mode,
                conf_mode=self.conf_mode,
                desc_dim=self.local_feat_dim,
                desc_mode=self.desc_mode,
                two_confs=self.two_confs,
                desc_conf_mode=self.desc_conf_mode,
            )
        return out

    AsymmetricMASt3R._encode_image = _encode_image
    AsymmetricMASt3R._decoder = _decoder
    Head.forward = head_forward
    _ENABLED = True
    return True


def allow_any_image_size() -> None:
    """MASt3R に渡す画の長い辺を 512 以外（例 384）にもできるようにする（上流の resize_img は 224 と 512 しか通さない）。

    224・512 は上流のまま。それ以外は 512 と同じ手順（長い辺を size に合わせ、真ん中を 16 の倍数に切る）。
    MASt3R の位置の埋め込みは相対（RoPE）なので、512 より小さい画でも重みはそのまま使える。
    画素が減ると、推論・頭・対応づけ・姿勢の解き・後ろの照合のどれもその分軽くなる。
    create_frame は frame.py が読み込んだ時の resize_img を使うので、両方の名前を差し替える。
    """
    import mast3r_slam.frame as frame_mod
    import mast3r_slam.mast3r_utils as mu
    import numpy as np
    import PIL.Image

    if getattr(mu.resize_img, "_any_size", False):
        return
    orig = mu.resize_img

    def resize_img(img, size, square_ok=False, return_transformation=False):
        if size in (224, 512):
            return orig(img, size, square_ok=square_ok, return_transformation=return_transformation)
        pil = PIL.Image.fromarray(np.uint8(img * 255))
        W1, H1 = pil.size
        pil = mu._resize_pil_image(pil, size)
        W, H = pil.size
        cx, cy = W // 2, H // 2
        halfw, halfh = ((2 * cx) // 16) * 8, ((2 * cy) // 16) * 8
        if not square_ok and W == H:
            halfh = 3 * halfw / 4
        pil = pil.crop((cx - halfw, cy - halfh, cx + halfw, cy + halfh))
        res = dict(img=mu.ImgNorm(pil)[None], true_shape=np.int32([pil.size[::-1]]), unnormalized_img=np.asarray(pil))
        if return_transformation:
            return res, (W1 / W, H1 / H, (W - pil.size[0]) / 2, (H - pil.size[1]) / 2)
        return res

    resize_img._any_size = True
    mu.resize_img = resize_img
    frame_mod.resize_img = resize_img
