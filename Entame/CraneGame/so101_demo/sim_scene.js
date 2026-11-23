// /sim の 3D 表示（three.js）。座標は MuJoCo と同じ Z が上・X が前・Y が左。表示だけで、物理はサーバー側。
import * as THREE from "/assets/three.module.min.js";

const TARGET = new THREE.Vector3(0.25, 0.05, 0.05);
const VIEWS = {            // 方位 az・仰角 el（rad）・距離 dist（m）。top は真上（画面の上が +X）
  oblique: { az: Math.PI + 0.6, el: 0.5, dist: 0.95 },
  top: { az: Math.PI, el: Math.PI / 2 - 0.001, dist: 0.8, up: [1, 0, 0] },
  side: { az: -Math.PI / 2, el: 0.12, dist: 0.95 },
  front: { az: 0, el: 0.25, dist: 0.95 },
};
const EL_RANGE = [0.05, 1.5], DIST_RANGE = [0.35, 2.0];
const COLORS = { desk: 0x39404d, floor: 0x20242c, hole: 0xffc83d, holeDone: 0x3ddc84, snack: 0xf09628, grabbable: 0xffe94a, held: 0x3ddc84, tip: 0xff4d4d };

function parseMesh(buffer) {
  const [vertCount, faceCount] = new Uint32Array(buffer, 0, 2);
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(buffer, 8, vertCount * 3), 3));
  geometry.setIndex(new THREE.BufferAttribute(new Uint32Array(buffer, 8 + vertCount * 12, faceCount * 3), 1));
  geometry.computeVertexNormals();
  return geometry;
}

function setPose(object, pose, offset = 0) {      // pose = [px,py,pz, 3x3 行列を行順に 9 個]
  const [px, py, pz, ...m] = pose.slice(offset);
  object.matrix.set(m[0], m[1], m[2], px, m[3], m[4], m[5], py, m[6], m[7], m[8], pz, 0, 0, 0, 1);
  object.matrixWorldNeedsUpdate = true;
}

function pivot(mesh) {
  mesh.matrixAutoUpdate = false;
  mesh.castShadow = mesh.receiveShadow = true;
  return mesh;
}

export async function createScene(canvas, model, fetchMesh) {
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.shadowMap.enabled = true;
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x15181e);
  const camera = new THREE.PerspectiveCamera(40, 1, 0.01, 10);
  camera.up.set(0, 0, 1);

  scene.add(new THREE.HemisphereLight(0xdfe8ff, 0x303540, 1.3));
  const sun = new THREE.DirectionalLight(0xffffff, 2.2);
  sun.position.set(0.2, -0.4, 1.0);
  sun.target.position.copy(TARGET);
  sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  Object.assign(sun.shadow.camera, { left: -0.6, right: 0.6, top: 0.6, bottom: -0.6, near: 0.1, far: 3 });
  scene.add(sun, sun.target);

  // 机の天板は穴を囲む箱の集まり（サーバーの物理と同じ）。上面の高さは全部同じ
  const top = model.hole.center[2];
  const deskMaterial = new THREE.MeshStandardMaterial({ color: COLORS.desk, roughness: 0.95 });
  for (const { center, half } of model.desk) {
    const piece = new THREE.Mesh(new THREE.BoxGeometry(...half.map((v) => v * 2)), deskMaterial);
    piece.position.set(...center);
    piece.receiveShadow = piece.castShadow = true;
    scene.add(piece);
  }
  const floor = new THREE.Mesh(new THREE.PlaneGeometry(6, 6), new THREE.MeshStandardMaterial({ color: COLORS.floor, roughness: 1 }));
  floor.position.z = model.floor_z;      // 穴の下の床。落ちたお菓子はここに残る
  floor.receiveShadow = true;
  scene.add(floor);

  // 穴の縁の目印（細い枠）。穴の中は机の影で暗いので、縁を色で縁取る
  const [hx, hy] = model.hole.center, hh = model.hole.half, edge = 0.004;
  const holeMaterial = new THREE.MeshBasicMaterial({ color: COLORS.hole });
  for (const [cx, cy, sx, sy] of [[hx, hy - hh, hh * 2 + edge, edge], [hx, hy + hh, hh * 2 + edge, edge],
                                  [hx - hh, hy, edge, hh * 2], [hx + hh, hy, edge, hh * 2]]) {
    const bar = new THREE.Mesh(new THREE.PlaneGeometry(sx, sy), holeMaterial);
    bar.position.set(cx, cy, top + 0.0004);
    scene.add(bar);
  }

  const parts = [];
  const geometries = new Map();
  for (const part of model.parts) {
    if (!geometries.has(part.mesh)) geometries.set(part.mesh, parseMesh(await fetchMesh(part.mesh)));
    const [r, g, b] = part.rgba;
    const mesh = pivot(new THREE.Mesh(geometries.get(part.mesh), new THREE.MeshStandardMaterial({ color: new THREE.Color(r, g, b), roughness: 0.6 })));
    scene.add(mesh);
    parts.push(mesh);
  }
  const snackMaterial = new THREE.MeshStandardMaterial({ color: COLORS.snack, roughness: 0.7 });
  const snack = pivot(new THREE.Mesh(new THREE.BoxGeometry(...model.snack.half.map((v) => v * 2)), snackMaterial));
  scene.add(snack);

  // 先端の目印: 球と、机の面までの縦線と、机の上の輪（高さと位置が見分けにくいので）
  const tipDot = new THREE.Mesh(new THREE.SphereGeometry(0.005, 16, 12), new THREE.MeshBasicMaterial({ color: COLORS.tip }));
  const stem = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]), new THREE.LineBasicMaterial({ color: COLORS.tip }));
  const ring = new THREE.Mesh(new THREE.RingGeometry(0.008, 0.011, 24), new THREE.MeshBasicMaterial({ color: COLORS.tip, side: THREE.DoubleSide }));
  scene.add(tipDot, stem, ring);

  const view = { ...VIEWS.oblique };
  function aim() {
    camera.up.set(...(view.up ?? [0, 0, 1]));
    camera.position.set(
      TARGET.x + view.dist * Math.cos(view.el) * Math.cos(view.az),
      TARGET.y + view.dist * Math.cos(view.el) * Math.sin(view.az),
      TARGET.z + view.dist * Math.sin(view.el));
    camera.lookAt(TARGET);
  }
  aim();

  let drag = null;
  canvas.addEventListener("pointerdown", (e) => { drag = { x: e.clientX, y: e.clientY }; canvas.setPointerCapture(e.pointerId); });
  canvas.addEventListener("pointerup", () => { drag = null; });
  canvas.addEventListener("pointermove", (e) => {
    if (!drag) return;
    delete view.up;
    view.az -= (e.clientX - drag.x) * 0.008;
    view.el = Math.min(EL_RANGE[1], Math.max(EL_RANGE[0], view.el + (e.clientY - drag.y) * 0.006));
    drag = { x: e.clientX, y: e.clientY };
    aim();
  });
  canvas.addEventListener("wheel", (e) => {
    e.preventDefault();
    view.dist = Math.min(DIST_RANGE[1], Math.max(DIST_RANGE[0], view.dist * Math.exp(e.deltaY * 0.001)));
    aim();
  }, { passive: false });

  function resize() {
    const { clientWidth: w, clientHeight: h } = canvas;
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }
  new ResizeObserver(resize).observe(canvas);
  resize();

  return {
    setView(name) { Object.assign(view, { up: undefined }, VIEWS[name]); aim(); },
    update(state) {
      state.geoms.forEach((pose, i) => setPose(parts[i], pose));
      setPose(snack, [...state.snack.pos, ...state.snack.mat]);
      const [x, y, z] = state.tip;
      tipDot.position.set(x, y, z);
      ring.position.set(x, y, top + 0.0006);
      stem.geometry.setFromPoints([new THREE.Vector3(x, y, top + 0.0006), new THREE.Vector3(x, y, z)]);
      holeMaterial.color.setHex(state.task.got ? COLORS.holeDone : COLORS.hole);
      // アシスト: つかめる距離なら黄色く、つかんでいる間は緑に光らせる
      const glow = state.held ? COLORS.held : state.grabbable ? COLORS.grabbable : 0x000000;
      snackMaterial.emissive.setHex(glow);
      snackMaterial.emissiveIntensity = glow ? 0.6 : 0;
    },
    render() { renderer.render(scene, camera); },
  };
}
