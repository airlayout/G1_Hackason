"""Fine-tune COCO bottle YOLO for the synthetic fixed-bottle fixture only."""
from __future__ import annotations

import argparse
import json
import hashlib
from pathlib import Path
import numpy as np
import yaml

from .runtime import enable_yada
from .settings import ROOT,load_settings


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--images",type=int,default=40)
    ap.add_argument("--epochs",type=int,default=15)
    args=ap.parse_args()
    enable_yada()
    import mujoco
    from PIL import Image
    from common.config import load_config
    from contest.task import make_trial
    from .simulation import make_mujoco_robot
    output=ROOT/"_local/wrist_press/bottle_yolo"
    cfg=load_settings(mode="bottle",use_wrist=False)
    signature=hashlib.sha256(json.dumps(cfg,sort_keys=True).encode()+Path(__file__).read_bytes()).hexdigest()[:10]
    dataset=output/f"dataset-{args.images}-{signature}"
    training=f"training-{args.images}-{signature}"
    cc=load_config("contest.yaml")
    for index in range(args.images):
        seed=10000+index
        trial=make_trial(seed,cc)
        trial.scene_cfg["wall"]["front_x"]=.65
        robot=make_mujoco_robot(trial.scene_cfg,cc,cfg)
        renderer=mujoco.Renderer(robot.model,height=480,width=640)
        renderer.enable_segmentation_rendering()
        try:
            rng=np.random.default_rng(seed)
            body=robot.model.body("press_fixed_bottle").id
            robot.model.body_pos[body,:2]+=rng.uniform([-.035,-.035],[.035,.035])
            if index%3:
                home=robot.data.qpos[robot._qadr].copy()
                near=np.array([-.224,-.251,.837,.485,.144,.292,-1.249])
                home[22:]+=rng.uniform(.4,1.05)*(near-home[22:])
                robot.data.qpos[robot._qadr]=home
                robot.q_des=home.copy()
            mujoco.mj_forward(robot.model,robot.data)
            obs=robot.observe(0)
            renderer.update_scene(robot.data,camera="head_camera")
            ids=renderer.render()
            gids=[g for g in range(robot.model.ngeom)
                  if robot.model.geom(g).name.startswith("press_bottle_") and robot.model.geom(g).name != "press_bottle_fixture"]
            mask=np.isin(ids[...,0],gids)&(ids[...,1]==mujoco.mjtObj.mjOBJ_GEOM)
            y,x=np.where(mask)
            if len(x)<100:
                continue
            split="val" if index%5==0 else "train"
            image=dataset/"images"/split/f"bottle_{seed}.png"
            label=dataset/"labels"/split/f"bottle_{seed}.txt"
            image.parent.mkdir(parents=True,exist_ok=True)
            label.parent.mkdir(parents=True,exist_ok=True)
            Image.fromarray(obs.rgb).save(image)
            label.write_text(f"0 {(x.min()+x.max())/2/640:.7f} {(y.min()+y.max())/2/480:.7f} {(x.max()-x.min()+1)/640:.7f} {(y.max()-y.min()+1)/480:.7f}\n")
        finally:
            renderer.close()
            robot.close()
    data=dataset/"data.yaml"
    data.write_text(yaml.safe_dump({"path":str(dataset),"train":"images/train","val":"images/val","names":{0:"bottle"}}))
    from ultralytics import YOLO
    model=YOLO(str(ROOT/"yolo26n.pt"))
    model.train(data=str(data),epochs=args.epochs,imgsz=320,batch=8,device="cpu",workers=0,
                project=str(output),name=training,exist_ok=True,plots=False,verbose=False)
    import shutil
    weights=output/"weights/best.pt"
    weights.parent.mkdir(parents=True,exist_ok=True)
    shutil.copyfile(output/training/"weights/best.pt",weights)
    (output/"training.json").write_text(json.dumps({"sim_only":True,"start_seed":10000,"images":args.images,
             "epochs":args.epochs,"generator_config_signature":signature,
             "weights_sha256":hashlib.sha256(weights.read_bytes()).hexdigest()},indent=2))
    print(weights)


if __name__=="__main__":
    main()
