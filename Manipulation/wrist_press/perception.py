"""RGB[-D] target localization. Simulator labels are never inputs."""
from __future__ import annotations

from collections import deque
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from time import perf_counter
import numpy as np

from .runtime import j1_module, source_root
from .settings import resolve_path
from .types import Target, unit


class NotFound(RuntimeError):
    pass


@dataclass
class Detection:
    name: str
    score: float
    bbox: tuple


class Yolo:
    def __init__(self, weights, names, confidence=0.4, device="cpu"):
        path = resolve_path(weights)
        if not path.is_file():
            raise FileNotFoundError(f"YOLO weights missing: {path}. Use the weights prepare command; no detector fallback is enabled.")
        from ultralytics import YOLO
        self.model = YOLO(str(path))
        self.confidence, self.device = confidence, device
        self.names = set(names)
        available = set(self.model.names.values())
        if not self.names <= available:
            raise ValueError(f"weights classes {available} do not contain {self.names}")
        self.model.predict(np.zeros((320, 320, 3), np.uint8), device=device, verbose=False)

    def detect(self, rgb):
        # Ultralytics numpy input is BGR. Our public camera contract is RGB.
        pred = self.model.predict(rgb[..., ::-1].copy(), conf=self.confidence,
                                  device=self.device, verbose=False)[0]
        return [Detection(self.model.names[int(cls)], float(score), tuple(map(float, box)))
                for box, cls, score in zip(pred.boxes.xyxy.cpu().numpy(), pred.boxes.cls.cpu().numpy(),
                                          pred.boxes.conf.cpu().numpy())
                if self.model.names[int(cls)] in self.names]


def point_cloud(frame):
    if frame.depth is None:
        raise NotFound("no depth")
    z = np.asarray(frame.depth, float)
    v, u = np.indices(z.shape)
    K, T = frame.K, frame.T_pelvis_optical
    xyz = np.stack(((u-K[0, 2])*z/K[0, 0], (v-K[1, 2])*z/K[1, 1], z), axis=-1)
    return xyz @ T[:3, :3].T + T[:3, 3]


def refine_button(rgb,bbox):
    """Use the detected button's visible rim for its center, rather than box bias."""
    import cv2
    h,w=rgb.shape[:2]
    x1,y1,x2,y2=map(float,bbox)
    padx,pady=.15*(x2-x1),.15*(y2-y1)
    left,top=max(0,int(x1-padx)),max(0,int(y1-pady))
    right,bottom=min(w,int(x2+padx)+1),min(h,int(y2+pady)+1)
    gray=cv2.cvtColor(rgb[top:bottom,left:right],cv2.COLOR_RGB2GRAY)
    if min(gray.shape,default=0)<8:
        return None
    border=np.r_[gray[0],gray[-1],gray[:,0],gray[:,-1]]
    background=float(np.median(border))
    bright=float(np.quantile(gray,.8))
    if bright-background<12:
        return None
    mask=(gray>background+.5*(bright-background)).astype(np.uint8)*255
    contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    candidates=[]
    for contour in contours:
        if len(contour)<12 or cv2.contourArea(contour)<.25*(x2-x1)*(y2-y1):
            continue
        ellipse=cv2.fitEllipse(contour)
        (cx,cy),(a,b),angle=ellipse
        if not .55*(x2-x1)<max(a,b)<1.5*max(x2-x1,y2-y1):
            continue
        if cv2.contourArea(contour)<.65*np.pi*a*b/4:
            continue
        center=np.array([cx+left,cy+top])
        if np.linalg.norm(center-[(x1+x2)/2,(y1+y2)/2])>.3*max(x2-x1,y2-y1):
            continue
        full=np.zeros((h,w),np.uint8)
        shifted=((cx+left,cy+top),(a,b),angle)
        cv2.ellipse(full,shifted,1,-1)
        points=cv2.boxPoints(shifted)
        lo,hi=points.min(axis=0),points.max(axis=0)
        candidates.append((float(cv2.contourArea(contour)),tuple([*lo,*hi]),full.astype(bool)))
    if not candidates:
        return None
    _,box,mask=max(candidates,key=lambda x:x[0])
    return box,mask


class RefinedButtons:
    def __init__(self,detector,module):
        self.detector,self.module=detector,module
    def detect(self,rgb):
        out=[]
        for candidate in self.detector.detect(rgb):
            refined=refine_button(rgb,candidate.bbox)
            if refined is not None:
                box,mask=refined
                candidate=self.module.Candidate2D(tuple(map(int,box)),mask,candidate.arrow,candidate.score,candidate.source)
            out.append(candidate)
        return out


class _FrozenTransform:
    def __init__(self, T):
        self.T = T
    def pelvis_from_optical(self, q):
        return self.T[:3, :3], self.T[:3, 3]


class Localizer:
    def __init__(self, cfg):
        self.cfg = cfg
        self.buttons = j1_module("elevator_buttons")
        self.button_cfg = j1_module("config").load_config(source_root() / "Button_Press/J1-gen/configs/elevator.yaml")
        self.button_cfg["detector"] = cfg["detector"]
        self.button_cfg["yolo"].update(weights=str(resolve_path(cfg["button_weights"])),
                                        confidence=cfg["confidence"], device=cfg["device"])
        if cfg["mode"] == "button":
            if cfg["detector"] == "yolo" and not resolve_path(cfg["button_weights"]).is_file():
                raise FileNotFoundError("Button YOLO weights missing; run python -m Manipulation.wrist_press.weights --help")
            self.detector = self.buttons.make_candidate_detector(self.button_cfg)
            if cfg["detector"]=="yolo" and cfg["refine_button_centers"]:
                self.detector=RefinedButtons(self.detector,self.buttons)
        else:
            self.detector = Yolo(cfg["bottle_weights"], ["bottle"], cfg["confidence"], cfg["device"])
        self.previous = None
        self.timings = []
        self.tracks = {}

    def locate(self, frame, name, previous=None):
        started = perf_counter()
        try:
            return self._button(frame, name, previous) if self.cfg["mode"] == "button" else self._bottle(frame,previous)
        finally:
            self.timings.append(1000 * (perf_counter()-started))

    def _button(self, frame, name, previous):
        # Whole head image must identify the general-use up/down pair, not wheelchair buttons.
        if frame.camera == "head" and previous is None:
            if frame.depth is None:
                raise NotFound("head RGB-D is required for initial localization")
            finder = self.buttons.ButtonFinder(_FrozenTransform(frame.T_pelvis_optical), self.detector, self.button_cfg)
            try:
                found = finder.find(frame.rgb, frame.depth, frame.K, frame.q[12:15])
            except self.buttons.ButtonNotFound as exc:
                raise NotFound(str(exc)) from exc
            b = found.button(name)
            P=point_cloud(frame)
            valid=np.isfinite(frame.depth)&(frame.depth>.05)&(frame.depth<2)
            mid=.5*(found.up.center+found.down.center)
            near=valid&(np.abs(P[...,1]-mid[1])<.06)&(np.abs(P[...,2]-mid[2])<.15)
            near&=np.abs(P[...,0]-mid[0])<.03
            for candidate in finder.last_candidates:
                near&=~candidate.mask
            if near.sum()<100:
                raise NotFound("insufficient sensed panel points")
            _,plane_center,_=self.buttons.fit_plane(P[near])
            wall=b.center+found.normal*float((plane_center-b.center)@found.normal)
            # Sensed local panel plane; conservative extension over the arm workspace.
            return self._target(frame, name, b.center, found.normal, b.diameter_m/2, b.bbox,
                                wall)
        if previous is None:
            raise NotFound("wrist camera cannot select a button without head association")
        matches = []
        candidates = self._tracked_candidates(frame)
        for c in candidates:
            if c.arrow is not None and c.arrow != name:
                continue
            refined=refine_button(frame.rgb,c.bbox) if self.cfg["refine_button_centers"] else None
            if self.cfg["refine_button_centers"] and frame.camera=="wrist" and refined is None:
                continue
            if refined is not None:
                c.bbox=refined[0]
            u, v = (c.bbox[0]+c.bbox[2])/2, (c.bbox[1]+c.bbox[3])/2
            ray = frame.T_pelvis_optical[:3, :3] @ np.linalg.solve(frame.K, [u, v, 1.0])
            origin = frame.T_pelvis_optical[:3, 3]
            denom = float(ray @ previous.normal)
            if denom <= 0.05:
                continue
            distance = float((previous.point-origin) @ previous.normal / denom)
            if not 0.03 < distance < 2:
                continue
            p = origin + distance*ray
            if frame.depth is not None:
                x, y = int(u), int(v)
                roi = frame.depth[max(0,y-2):y+3, max(0,x-2):x+3]
                valid = roi[np.isfinite(roi) & (roi > 0.03)]
                if len(valid) >= 6:
                    depth = float(np.median(valid))
                    # Reject foreground finger and background rather than changing targets.
                    if abs(depth-distance) > 0.02:
                        continue
                    p = origin + depth*ray
            if np.linalg.norm(p-previous.point) > 0.025:
                continue
            matches.append(self._target(frame, name, p, previous.normal, previous.radius, c.bbox,
                                         previous.wall_point))
        if len(matches) != 1:
            raise NotFound(f"wrist target is ambiguous/missing ({len(matches)})")
        return matches[0]

    def _tracked_candidates(self, frame):
        """Track the locked patch between detections; re-detect on loss or every .5 s."""
        import cv2
        old = self.tracks.get(frame.camera)
        gray = cv2.cvtColor(frame.rgb, cv2.COLOR_RGB2GRAY)
        if old is not None and 0 < frame.captured_at-old["detected_at"] < .5:
            mask = np.zeros(old["gray"].shape, np.uint8)
            x1,y1,x2,y2 = map(int,old["bbox"])
            mask[max(0,y1):y2,max(0,x1):x2] = 255
            points = cv2.goodFeaturesToTrack(old["gray"], 40, .01, 2, mask=mask)
            if points is not None and len(points) >= 4:
                moved, ok, _ = cv2.calcOpticalFlowPyrLK(old["gray"],gray,points,None)
                if moved is not None:
                    back, back_ok, _ = cv2.calcOpticalFlowPyrLK(gray,old["gray"],moved,None)
                    keep = ok.ravel().astype(bool) & back_ok.ravel().astype(bool)
                    keep &= np.linalg.norm(back-points,axis=(1,2)) < 1.5
                    if keep.sum() >= 4:
                        shift = np.median((moved-points)[keep,0],axis=0)
                        if np.linalg.norm(shift) < 40:
                            box = np.asarray(old["bbox"])+np.tile(shift,2)
                            h,w = gray.shape
                            if 0 <= box[0] < box[2] < w and 0 <= box[1] < box[3] < h:
                                from types import SimpleNamespace
                                return [SimpleNamespace(bbox=box,arrow=old["name"])]
        self.tracks.pop(frame.camera, None)
        return self.detector.detect(frame.rgb)

    def _target(self, frame, name, p, n, radius, bbox, wall=None):
        x1, y1, x2, y2 = [int(v) for v in bbox]
        patch = frame.rgb[max(0,y1):y2, max(0,x1):x2]
        brightness = float(np.median(patch.mean(axis=-1))) if patch.size else None
        import cv2
        old = self.tracks.get(frame.camera)
        detected_at = old["detected_at"] if old else frame.captured_at
        self.tracks[frame.camera] = {"gray": cv2.cvtColor(frame.rgb,cv2.COLOR_RGB2GRAY),
                                      "bbox": bbox, "name": name, "detected_at": detected_at}
        return Target("button", name, p, n, radius, frame.captured_at, camera=frame.camera,
                      bbox=bbox, brightness=brightness, wall_point=wall)

    def _bottle(self, frame, previous=None):
        if frame.depth is None:
            raise NotFound("bottle localization needs valid depth on the label")
        detections = self._tracked_candidates(frame)
        if len(detections) != 1:
            raise NotFound(f"expected one bottle, found {len(detections)}")
        d = detections[0]
        x1, y1, x2, y2 = d.bbox
        ax, ay = self.cfg["bottle"]["anchor"]
        u, v = x1 + ax*(x2-x1), y1 + ay*(y2-y1)
        if previous is not None:
            T=frame.T_pelvis_optical
            optical=T[:3,:3].T @ (previous.point-T[:3,3])
            if optical[2]<=.03:
                raise NotFound("locked label point is behind the camera")
            pixel=frame.K @ optical
            u,v=pixel[:2]/pixel[2]
            if not x1-5<=u<=x2+5 or not y1-5<=v<=y2+5:
                raise NotFound("bottle detection does not contain the locked label point")
        dx, dy = max(2, int(0.08*(x2-x1))), max(2, int(0.04*(y2-y1)))
        x, y = int(u), int(v)
        roi = frame.depth[max(0,y-dy):y+dy+1, max(0,x-dx):x+dx+1]
        valid = roi[np.isfinite(roi) & (roi > 0.08) & (roi < 2)]
        if len(valid) < 12 or len(valid) < roi.size*0.65:
            raise NotFound("insufficient bottle label depth; transparent bottle/background rejected")
        if np.quantile(valid, .9)-np.quantile(valid, .1) > self.cfg["bottle"]["depth_spread_m"]:
            raise NotFound("mixed foreground/background depth on bottle")
        z = float(np.median(valid))
        if previous is not None and abs(z-optical[2])>.015:
            raise NotFound("bottle label is occluded or inconsistent with the locked surface")
        T = frame.T_pelvis_optical
        p = T[:3, :3] @ (np.linalg.solve(frame.K, [u,v,1])*z) + T[:3, 3]
        n = p-T[:3, 3]
        n[2] = 0  # Horizontal radial direction of the upright cylinder.
        n = unit(n)
        if previous is not None:
            n=previous.normal.copy()
        import cv2
        old=self.tracks.get(frame.camera)
        detected_at=old["detected_at"] if old else frame.captured_at
        self.tracks[frame.camera]={"gray":cv2.cvtColor(frame.rgb,cv2.COLOR_RGB2GRAY),"bbox":d.bbox,
                                   "name":"bottle","detected_at":detected_at}
        return Target("bottle", "bottle", p, n, self.cfg["bottle"]["radius_m"], frame.captured_at,
                      confidence=getattr(d,"score",.9), camera=frame.camera, bbox=d.bbox)


class LatestPerception:
    """One in-flight inference and one replaceable pending frame; no stale queue."""
    def __init__(self, localizer, asynchronous=False):
        self.localizer = localizer
        self.pool = ThreadPoolExecutor(max_workers=1) if asynchronous else None
        self.future = None
        self.pending = None
        self.result = None
        self.error = ""
        self.last_keys = {}

    def update(self, frame, name, previous=None):
        if frame is None:
            return None
        key = (frame.camera, frame.captured_at, frame.sequence)
        if self.future is not None and self.future.done():
            self._collect(self.future)
            self.future = None
        if key != self.last_keys.get(frame.camera):
            self.pending = (frame, name, previous)
            self.last_keys[frame.camera] = key
        if self.pending and self.future is None:
            args, self.pending = self.pending, None
            if self.pool:
                self.future = self.pool.submit(self.localizer.locate, *args)
            else:
                self.result = None
                try:
                    self.result = self.localizer.locate(*args)
                    self.error = ""
                except NotFound as exc:
                    self.error = str(exc)
        result, self.result = self.result, None
        return result

    def _collect(self, f):
        self.result = None
        try:
            self.result = f.result()
            self.error = ""
        except NotFound as exc:
            self.error = str(exc)

    def close(self):
        if self.pool:
            self.pool.shutdown(wait=True, cancel_futures=True)


class StableTarget:
    def __init__(self, count=3, tolerance=0.005):
        self.items = deque(maxlen=count)
        self.tolerance = tolerance
        self.last = None

    def add(self, target):
        key = (target.camera, target.captured_at)
        if key == self.last:
            return None
        self.last = key
        self.items.append(target)
        if len(self.items) != self.items.maxlen:
            return None
        points = np.array([t.point for t in self.items])
        if np.max(np.linalg.norm(points-np.median(points, axis=0), axis=1)) > self.tolerance:
            return None
        if any(t.name != target.name or t.normal @ target.normal < np.cos(np.radians(5)) for t in self.items):
            return None
        target.point = np.median(points, axis=0)
        return target
