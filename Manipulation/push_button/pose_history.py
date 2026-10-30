"""受信状態の履歴から、画像の撮影時刻に対応する関節角とIMUを補間する。"""

from collections import deque
import threading

import numpy as np


class PoseHistory:
    def __init__(self):
        self.samples=deque(maxlen=200)
        self.lock=threading.Lock()

    def append(self, configuration):
        q,rpy,stamp,*_=configuration
        with self.lock:
            if not self.samples or stamp>self.samples[-1][2]:
                self.samples.append((np.array(q),np.array(rpy),float(stamp)))

    def at(self, stamp, fallback):
        with self.lock:
            rows=list(self.samples)
        if not rows or stamp<rows[0][2] or stamp>rows[-1][2]:
            q,rpy,*_=min(rows,key=lambda r:abs(r[2]-stamp)) if rows else fallback
            return np.array(q),np.array(rpy)
        for a,b in zip(rows,rows[1:]):
            if a[2]<=stamp<=b[2]:
                ratio=(stamp-a[2])/(b[2]-a[2])
                angles=np.arctan2(np.sin(b[1]-a[1]),np.cos(b[1]-a[1]))
                return a[0]+ratio*(b[0]-a[0]),a[1]+ratio*angles
        return rows[-1][0].copy(),rows[-1][1].copy()
