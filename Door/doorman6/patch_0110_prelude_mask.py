"""0110: 前座の方策が動かした歩（引き継ぐ前）を、方策の損失・エントロピー・CAPS・KL から外す。価値の損失には残す。
  python patch_0110.py <ppo_trainer_homie_api.py>
"""
import sys

p = sys.argv[1]
s = open(p).read()
if "0110" in s:
    sys.exit("もう当たっている")


def rep(old: str, new: str) -> None:
    global s
    if s.count(old) != 1:
        sys.exit(f"当てる所が 1 つでない（{s.count(old)}）: {old[:60]}")
    s = s.replace(old, new)


rep("        prelude = self._load_prelude_policy()  # 0109\n",
    "        prelude = self._load_prelude_policy()  # 0109\n"
    "        self._prelude_mask_steps = []  # 0110: 歩ごとの「引き継ぐ前」の印\n")
rep("                    actor_state[\"actions\"] = self._prelude_swap(prelude, obs_dict, policy_state_dict[\"actions\"], cur_dones=dones)\n",
    "                    actor_state[\"actions\"] = self._prelude_swap(prelude, obs_dict, policy_state_dict[\"actions\"], cur_dones=dones)\n"
    "                    self._prelude_mask_steps.append(self._prelude_pre_now.clone())  # 0110\n")
rep("        pre = ~self.env.arm_handover_buf.to(actions.device)\n",
    "        pre = ~self.env.arm_handover_buf.to(actions.device)\n"
    "        self._prelude_pre_now = pre  # 0110\n")
rep("            padding_mask = torch.zeros_like(dones)\n            padding_mask_p1 = torch.zeros_like(dones)\n",
    "            padding_mask = torch.zeros_like(dones)\n            padding_mask_p1 = torch.zeros_like(dones)\n"
    "        # 0110: 前座が動かした歩は、学ぶ方策の行動と結果が結び付かないので方策の損失から外す（価値は学ぶ）\n"
    "        pm_steps = getattr(self, \"_prelude_mask_steps\", None)\n"
    "        if pm_steps and self.config.get(\"prelude_mask_loss\", True):\n"
    "            pm = torch.stack(pm_steps, dim=1).to(device)\n"
    "            if pm.shape != padding_mask.shape:\n"
    "                raise RuntimeError(f\"0110: 印の形 {tuple(pm.shape)} が覆いの形 {tuple(padding_mask.shape)} と違う\")\n"
    "            padding_mask = padding_mask.bool() | pm\n"
    "            if not getattr(self, \"_prelude_mask_said\", False):\n"
    "                self._prelude_mask_said = True\n"
    "                print(f\"[prelude] 0110 引き継ぐ前の歩を方策の損失から外す {100.0 * pm.float().mean().item():.1f}%\", flush=True)\n")
rep("            local_kl_mean = torch.mean(kl)\n",
    "            if getattr(self, \"_prelude_mask_steps\", None) and kl.shape == padding_mask.shape and padding_mask.dtype == torch.bool:\n"
    "                local_kl_mean = masked_mean(kl, ~padding_mask)  # 0110\n"
    "            else:\n"
    "                local_kl_mean = torch.mean(kl)\n")
open(p, "w").write(s)
print("当てた", p)
