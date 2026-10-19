"""Bounded RRT-Connect transit and checked Cartesian contact/retreat paths."""
from __future__ import annotations

import time
import numpy as np

from .model import ARM


class PlanningError(RuntimeError):
    pass


class Planner:
    def __init__(self, robot, collision, cfg):
        self.robot, self.collision, self.cfg = robot, collision, cfg
        self.paths = []

    def remember(self,path):
        self.paths = (self.paths+[path])[-8:]
        return path

    def transit(self, start, goal, target):
        pad=self.cfg.get("planning_padding_m",0)
        def edge(a,b,t):
            return self.collision.edge(a,b,t,padding_m=pad)
        if not self.collision.valid(start, target,padding_m=pad) or not self.collision.valid(goal, target,padding_m=pad):
            raise PlanningError(self.collision.last_reason)
        if edge(start, goal, target):
            return self.remember([start.copy(), goal.copy()])
        # Reuse a known corridor only after checking every edge in the CURRENT scene.
        for prior in reversed(self.paths):
            for middle in (list(reversed(prior)),prior):
                candidate = [start.copy()]+[q.copy() for q in middle]+[goal.copy()]
                if all(edge(a,b,target) for a,b in zip(candidate,candidate[1:])):
                    return self.remember(self.shortcut(candidate,target))
        # Fast, target-relative elbow-out candidates before general joint-space search.
        # These are hints only: both segments still require full collision checks.
        right = np.cross(target.normal, [0,0,1.])
        right /= np.linalg.norm(right)
        axis = self.robot.kin.fk(goal)[1]
        for back, side in ((.12,.08),(.12,.15),(.18,.15)):
            point = target.point-(back+self.cfg["tip_radius_m"])*target.normal+side*right
            point[2] = max(.15, point[2])
            try:
                via = self.robot.solve(point, goal, axis)
            except ValueError:
                continue
            if edge(start, via, target) and edge(via, goal, target):
                return self.remember([start.copy(), via, goal.copy()])
        rng = np.random.default_rng(17)
        trees = [([start.copy()], [-1]), ([goal.copy()], [-1])]
        deadline = time.perf_counter() + self.cfg["planning_timeout_s"]
        def extend(tree, sample):
            nodes, parents = tree
            i = int(np.argmin([np.linalg.norm(n[ARM] - sample[ARM]) for n in nodes]))
            delta = sample[ARM] - nodes[i][ARM]
            distance = np.linalg.norm(delta)
            q = nodes[i].copy()
            q[ARM] += delta * min(1.0, 0.2 / max(distance, 1e-9))
            if not edge(nodes[i], q, target):
                return None, False
            nodes.append(q)
            parents.append(i)
            return len(nodes)-1, distance <= 0.2
        def branch(tree, index):
            out = []
            while index >= 0:
                out.append(tree[0][index])
                index = tree[1][index]
            return list(reversed(out))
        turn = 0
        while time.perf_counter() < deadline:
            sample = start.copy()
            sample[ARM] = rng.uniform(self.robot.lower, self.robot.upper)
            a, b = trees[turn], trees[1-turn]
            ia, _ = extend(a, sample)
            if ia is not None:
                while time.perf_counter() < deadline:
                    ib, reached = extend(b, a[0][ia])
                    if ib is None:
                        break
                    if reached:
                        left, right = branch(a, ia), branch(b, ib)
                        path = left + list(reversed(right))
                        if turn:
                            path.reverse()
                        return self.remember(self.shortcut(path, target))
            turn = 1-turn
        raise PlanningError("no collision-free transit within planning budget")

    def shortcut(self, path, target):
        out, i = [path[0]], 0
        while i < len(path)-1:
            j = len(path)-1
            while j > i+1 and not self.collision.edge(path[i], path[j], target,padding_m=self.cfg.get("planning_padding_m",0)):
                j -= 1
            out.append(path[j])
            i = j
        return out

    def cartesian(self, start, point, axis, target, contact=False):
        origin = self.robot.tip(start)
        count = max(1, int(np.ceil(np.linalg.norm(point-origin) / 0.002)))
        q, path = start.copy(), [start.copy()]
        for s in np.linspace(0, 1, count+1)[1:]:
            goal = self.robot.solve(origin + s*(point-origin), q, axis)
            if not self.collision.edge(q, goal, target, contact):
                raise PlanningError(self.collision.last_reason)
            path.append(goal)
            q = goal
        # Avoid stopping at every 2 mm IK knot. Keep a longer segment only when
        # its actual FK stays on the straight line and its swept path is valid.
        compact, i = [path[0]], 0
        while i < len(path)-1:
            j = len(path)-1
            while j > i+1:
                a, b = self.robot.tip(path[i]), self.robot.tip(path[j])
                straight = all(np.linalg.norm(self.robot.tip(path[i]+s*(path[j]-path[i]))-(a+s*(b-a))) < .0006
                               for s in np.linspace(0,1,9))
                if straight and self.collision.edge(path[i],path[j],target,contact):
                    break
                j -= 1
            compact.append(path[j])
            i = j
        return compact


class Trajectory:
    """Seven-phase jerk-limited S curves along the checked joint segments."""
    def __init__(self, path, cfg):
        self.path = np.asarray(path, float)
        if self.path.ndim != 2 or self.path.shape[1] != 29 or not np.isfinite(self.path).all():
            raise ValueError("invalid joint path")
        delta = np.max(np.abs(np.diff(self.path[:, ARM], axis=0)), axis=1)
        self.curves = [self._curve(d,cfg) for d in delta]
        self.durations = np.array([sum(t for t,j in curve) for curve in self.curves])
        self.ends = np.cumsum(self.durations)
        self.duration = float(self.ends[-1]) if len(self.ends) else 0.0

    def at(self, elapsed):
        if elapsed >= self.duration or not len(self.ends):
            return self.path[-1].copy()
        i = min(int(np.searchsorted(self.ends, max(0, elapsed), side="right")), len(self.durations)-1)
        prev = self.ends[i-1] if i else 0
        remaining = max(0,elapsed-prev)
        position = velocity = acceleration = 0.0
        for duration,jerk in self.curves[i]:
            t = min(remaining,duration)
            position += velocity*t + .5*acceleration*t*t + jerk*t*t*t/6
            velocity += acceleration*t + .5*jerk*t*t
            acceleration += jerk*t
            remaining -= t
            if remaining <= 1e-12:
                break
        distance = np.max(np.abs(self.path[i+1,ARM]-self.path[i,ARM]))
        s = np.clip(position/max(distance,1e-12),0,1)
        return self.path[i] + s * (self.path[i+1]-self.path[i])

    @staticmethod
    def _curve(distance,cfg):
        if distance < 1e-12:
            return [(.02,0)]
        V,A,J = cfg["joint_speed_rad_s"],cfg["joint_acceleration_rad_s2"],cfg["joint_jerk_rad_s3"]
        tj = min(A/J,np.sqrt(V/J))
        ta = max(0,V/(J*tj)-tj)
        noncruise = V*(ta+2*tj)
        tv = max(0,(distance-noncruise)/V)
        if distance < noncruise:
            ta = (-tj+np.sqrt(tj*tj+4*distance/(J*tj)))/2-tj
            if ta < 0:
                tj,ta = np.cbrt(distance/(2*J)),0
        return list(zip([tj,ta,tj,tv,tj,ta,tj],[J,0,-J,0,-J,0,J]))
