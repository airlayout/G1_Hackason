// 壁を避けた最短経路の探索（ミニロボット用）。
// 見通しが通るなら直線。通らなければ格子上の A* で壁を回り込む経路を求め、
// 見通しの通る点だけに間引いて「できるだけ直線に近い」折れ線にする。

import { segmentBlockedByWall } from "./collision.js";

const CELL_PX = 10; // 格子の1マスの大きさ
const ENDPOINT_REACH_CELLS = 2; // 出発点・到着点から格子へつなぐ範囲（±マス）
const NEIGHBORS = [
  [1, 0], [-1, 0], [0, 1], [0, -1],
  [1, 1], [1, -1], [-1, 1], [-1, -1],
];

// a→b を半径 radius の円が壁に当たらず通れるか。
export function isLineClear(a, b, walls, radius) {
  return !walls.some((wall) => segmentBlockedByWall(a, b, wall, radius));
}

// 半径 radius の円の中心が p に置けるか（エリア内で、壁に触れない）。
export function isPointFree(p, stage, radius) {
  if (p.x < radius || p.y < radius || p.x > stage.width - radius || p.y > stage.height - radius) return false;
  return !stage.walls.some(
    (w) => p.x > w.x - radius && p.x < w.x + w.w + radius && p.y > w.y - radius && p.y < w.y + w.h + radius
  );
}

// from → to の経路を、通過する点の列（from は含まず、最後は to）で返す。通れる経路が無ければ null。
export function findPath(from, to, stage, radius) {
  if (isLineClear(from, to, stage.walls, radius)) return [to];

  const grid = createGrid(stage, radius);
  const route = searchRoute(from, to, grid, stage, radius);
  return route ? smoothRoute(route, stage.walls, radius).slice(1) : null;
}

function createGrid(stage, radius) {
  const cols = Math.ceil(stage.width / CELL_PX);
  const rows = Math.ceil(stage.height / CELL_PX);
  const free = new Uint8Array(cols * rows);
  for (let r = 0; r < rows; r++) {
    for (let c = 0; c < cols; c++) {
      free[r * cols + c] = isPointFree(cellCenter(c, r), stage, radius) ? 1 : 0;
    }
  }
  return { cols, rows, free };
}

function cellCenter(c, r) {
  return { x: (c + 0.5) * CELL_PX, y: (r + 0.5) * CELL_PX };
}

function cellOf(p, grid) {
  return {
    c: Math.max(0, Math.min(grid.cols - 1, Math.floor(p.x / CELL_PX))),
    r: Math.max(0, Math.min(grid.rows - 1, Math.floor(p.y / CELL_PX))),
  };
}

// 出発点・格子のマス・到着点をノードにした A*。経路を点の列（from, …, to）で返す。
function searchRoute(from, to, grid, stage, radius) {
  const { cols, rows, free } = grid;
  const count = cols * rows;
  const START = count;
  const GOAL = count + 1;
  const goalCell = cellOf(to, grid);
  const fromCell = cellOf(from, grid);

  const posOf = (id) => (id === START ? from : id === GOAL ? to : cellCenter(id % cols, Math.floor(id / cols)));
  const dist = (a, b) => Math.hypot(a.x - b.x, a.y - b.y);
  const clear = (a, b) => isLineClear(a, b, stage.walls, radius);

  const neighborsOf = (id) => {
    const result = [];
    if (id === START) {
      forEachCellAround(fromCell, grid, (nid) => free[nid] && clear(from, posOf(nid)) && result.push(nid));
      return result;
    }
    const c = id % cols;
    const r = Math.floor(id / cols);
    const here = posOf(id);
    for (const [dc, dr] of NEIGHBORS) {
      const nc = c + dc;
      const nr = r + dr;
      if (nc < 0 || nr < 0 || nc >= cols || nr >= rows) continue;
      const nid = nr * cols + nc;
      if (free[nid] && clear(here, posOf(nid))) result.push(nid);
    }
    const nearGoal = Math.abs(c - goalCell.c) <= ENDPOINT_REACH_CELLS && Math.abs(r - goalCell.r) <= ENDPOINT_REACH_CELLS;
    if (nearGoal && clear(here, to)) result.push(GOAL);
    return result;
  };

  const best = new Float64Array(count + 2).fill(Infinity);
  const parent = new Int32Array(count + 2).fill(-1);
  const heap = createHeap();
  best[START] = 0;
  heap.push(START, dist(from, to));

  while (heap.size() > 0) {
    const id = heap.pop();
    if (id === GOAL) return rebuildRoute(parent, GOAL, posOf);
    for (const nid of neighborsOf(id)) {
      const cost = best[id] + dist(posOf(id), posOf(nid));
      if (cost >= best[nid]) continue;
      best[nid] = cost;
      parent[nid] = id;
      heap.push(nid, cost + dist(posOf(nid), to));
    }
  }
  return null;
}

function forEachCellAround(cell, grid, fn) {
  for (let dr = -ENDPOINT_REACH_CELLS; dr <= ENDPOINT_REACH_CELLS; dr++) {
    for (let dc = -ENDPOINT_REACH_CELLS; dc <= ENDPOINT_REACH_CELLS; dc++) {
      const c = cell.c + dc;
      const r = cell.r + dr;
      if (c >= 0 && r >= 0 && c < grid.cols && r < grid.rows) fn(r * grid.cols + c);
    }
  }
}

function rebuildRoute(parent, goalId, posOf) {
  const route = [];
  for (let id = goalId; id !== -1; id = parent[id]) route.push(posOf(id));
  return route.reverse();
}

// 見通しの通る一番遠い点へ直接つなぎ直して、格子のジグザグを取り除く。
function smoothRoute(route, walls, radius) {
  const smoothed = [route[0]];
  let anchor = 0;
  while (anchor < route.length - 1) {
    let farthest = anchor + 1;
    for (let j = route.length - 1; j > anchor + 1; j--) {
      if (isLineClear(route[anchor], route[j], walls, radius)) {
        farthest = j;
        break;
      }
    }
    smoothed.push(route[farthest]);
    anchor = farthest;
  }
  return smoothed;
}

// 小さい優先度が先に出る二分ヒープ。
function createHeap() {
  const items = [];
  const swap = (i, j) => ([items[i], items[j]] = [items[j], items[i]]);
  return {
    size: () => items.length,
    push(id, priority) {
      items.push({ id, priority });
      let i = items.length - 1;
      while (i > 0) {
        const p = (i - 1) >> 1;
        if (items[p].priority <= items[i].priority) break;
        swap(i, p);
        i = p;
      }
    },
    pop() {
      const top = items[0];
      const last = items.pop();
      if (items.length > 0) {
        items[0] = last;
        let i = 0;
        for (;;) {
          const l = 2 * i + 1;
          const r = l + 1;
          let m = i;
          if (l < items.length && items[l].priority < items[m].priority) m = l;
          if (r < items.length && items[r].priority < items[m].priority) m = r;
          if (m === i) break;
          swap(i, m);
          i = m;
        }
      }
      return top.id;
    },
  };
}
