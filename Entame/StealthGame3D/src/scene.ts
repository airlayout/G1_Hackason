import * as THREE from "three";
import { DEFAULT_WALL_HEIGHT_M, PLAYER_RADIUS_PX, PX_TO_M } from "./constants";
import type { Stage } from "./types";

export interface StageScene {
  scene: THREE.Scene;
  fpCamera: THREE.PerspectiveCamera;
  topCamera: THREE.PerspectiveCamera;
  playerMesh: THREE.Mesh;
  footRing: THREE.Mesh;
  guardMesh: THREE.Mesh;
  visionMesh: THREE.Mesh;
  wallBoxes: THREE.Box3[];
  wallMeshes: THREE.Mesh[];
}

// 2D座標(px)をXZ平面のメートル座標に変換する（yは高さ）。
export function toWorld(xPx: number, yPx: number): THREE.Vector2 {
  return new THREE.Vector2(xPx * PX_TO_M, yPx * PX_TO_M);
}

export function buildStageScene(stage: Stage): StageScene {
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x10141a);

  const widthM = stage.width * PX_TO_M;
  const depthM = stage.height * PX_TO_M;

  const floorGeo = new THREE.PlaneGeometry(widthM, depthM);
  const floorMat = new THREE.MeshStandardMaterial({ color: 0x2a2f38 });
  const floor = new THREE.Mesh(floorGeo, floorMat);
  floor.rotation.x = -Math.PI / 2;
  floor.position.set(widthM / 2, 0, depthM / 2);
  scene.add(floor);

  scene.add(new THREE.AmbientLight(0xffffff, 0.6));
  const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
  dirLight.position.set(widthM / 2, 10, depthM / 2 - 5);
  scene.add(dirLight);

  const wallMat = new THREE.MeshStandardMaterial({ color: 0x5a6b85 });
  const wallBoxes: THREE.Box3[] = [];
  const wallMeshes: THREE.Mesh[] = [];
  for (const wall of stage.walls) {
    const wM = wall.w * PX_TO_M;
    const dM = wall.h * PX_TO_M;
    const heightM = DEFAULT_WALL_HEIGHT_M;
    const geo = new THREE.BoxGeometry(wM, heightM, dM);
    const mesh = new THREE.Mesh(geo, wallMat);
    mesh.position.set(wall.x * PX_TO_M + wM / 2, heightM / 2, wall.y * PX_TO_M + dM / 2);
    scene.add(mesh);
    wallMeshes.push(mesh);
    wallBoxes.push(new THREE.Box3().setFromObject(mesh));
  }

  addMarkerWithBeacon(scene, toWorld(stage.start.x, stage.start.y), 0x4caf50);
  addMarkerWithBeacon(scene, toWorld(stage.target.x, stage.target.y), 0xffc107);

  const playerMesh = new THREE.Mesh(
    new THREE.CapsuleGeometry(0.12, 0.4, 4, 8),
    new THREE.MeshStandardMaterial({ color: 0x4fc3f7 })
  );
  playerMesh.position.y = 0.35;
  playerMesh.layers.set(1); // 一人称カメラには映さない(俯瞰カメラのみ layer1 を見る)
  scene.add(playerMesh);

  // 足元リング：プレイヤーの当たり判定範囲(半径)を床に描画し、
  // 一人称視点でも自分の立ち位置(足元)がわかるようにする。
  const footRing = new THREE.Mesh(
    new THREE.RingGeometry(PLAYER_RADIUS_PX * PX_TO_M * 0.7, PLAYER_RADIUS_PX * PX_TO_M, 24),
    new THREE.MeshBasicMaterial({ color: 0x4fc3f7, transparent: true, opacity: 0.8, side: THREE.DoubleSide })
  );
  footRing.rotation.x = -Math.PI / 2;
  footRing.position.y = 0.015;
  scene.add(footRing);

  const guardMesh = new THREE.Mesh(new THREE.BoxGeometry(0.3, 0.7, 0.3), new THREE.MeshStandardMaterial({ color: 0xef5350 }));
  guardMesh.position.y = 0.35;
  scene.add(guardMesh);

  const visionMesh = new THREE.Mesh(
    new THREE.BufferGeometry(),
    new THREE.MeshBasicMaterial({ color: 0xffee58, transparent: true, opacity: 0.25, side: THREE.DoubleSide })
  );
  visionMesh.position.y = 0.02;
  scene.add(visionMesh);

  // fovは初期値。実際の値は main.ts の resizeRenderer で水平FOV基準に再計算する。
  const fpCamera = new THREE.PerspectiveCamera(90, 1, 0.05, 200);
  // ヨー(左右)を先に、ピッチ(上下)をローカルXで適用する順序でロール発生を防ぐ。
  fpCamera.rotation.order = "YXZ";

  const topCamera = new THREE.PerspectiveCamera(55, 1, 0.1, 200);
  const centerX = widthM / 2;
  const centerZ = depthM / 2;
  topCamera.position.set(centerX, Math.max(widthM, depthM) * 0.9, centerZ + Math.max(widthM, depthM) * 0.5);
  topCamera.lookAt(centerX, 0, centerZ);
  topCamera.layers.enable(1);

  return { scene, fpCamera, topCamera, playerMesh, footRing, guardMesh, visionMesh, wallBoxes, wallMeshes };
}

// STARTやTARGETの位置を、床の円+視認しやすい発光ビーコン(半透明の柱)で示す。
function addMarkerWithBeacon(scene: THREE.Scene, worldPos: THREE.Vector2, color: number): void {
  const disc = new THREE.Mesh(new THREE.CircleGeometry(0.3, 24), new THREE.MeshBasicMaterial({ color }));
  disc.rotation.x = -Math.PI / 2;
  disc.position.set(worldPos.x, 0.012, worldPos.y);
  scene.add(disc);

  const beaconHeight = 1.6;
  const beacon = new THREE.Mesh(
    new THREE.CylinderGeometry(0.05, 0.05, beaconHeight, 12),
    new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.5 })
  );
  beacon.position.set(worldPos.x, beaconHeight / 2, worldPos.y);
  scene.add(beacon);
}

// 視野ポリゴン(2D px座標の点列)からXZ平面上のfanメッシュを再構築する。
export function updateVisionMesh(visionMesh: THREE.Mesh, points: { x: number; y: number }[]): void {
  const vertices: number[] = [];
  for (let i = 1; i < points.length - 1; i++) {
    const p0 = toWorld(points[0].x, points[0].y);
    const p1 = toWorld(points[i].x, points[i].y);
    const p2 = toWorld(points[i + 1].x, points[i + 1].y);
    vertices.push(p0.x, 0, p0.y, p1.x, 0, p1.y, p2.x, 0, p2.y);
  }
  const geometry = visionMesh.geometry as THREE.BufferGeometry;
  geometry.setAttribute("position", new THREE.Float32BufferAttribute(vertices, 3));
  geometry.computeVertexNormals();
}
