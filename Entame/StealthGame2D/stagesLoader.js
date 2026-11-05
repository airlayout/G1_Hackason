// ステージデータを stages/manifest.json 経由でJSONファイル群から読み込む。
// 新しいステージを追加するときは stages/*.json を置いて manifest.json に追記するだけでよい。

// 旧形式（securityCamera: 1台 or 無し）を securityCameras 配列に揃える。
export function normalizeStage(stage) {
  const { securityCamera, ...rest } = stage;
  const cameras = Array.isArray(rest.securityCameras) ? rest.securityCameras : securityCamera ? [securityCamera] : [];
  return { ...rest, securityCameras: cameras };
}

export async function loadStages() {
  // 保存直後のリロードで古いJSONが出ないよう、ブラウザのキャッシュを使わない。
  // （devserver は Cache-Control を付けないため、更新日時が古いファイルは長く再利用されうる）
  const manifestResponse = await fetch("stages/manifest.json", { cache: "no-store" });
  if (!manifestResponse.ok) {
    throw new Error(`stages/manifest.json の読み込みに失敗しました (status: ${manifestResponse.status})`);
  }
  const fileNames = await manifestResponse.json();

  const stages = await Promise.all(
    fileNames.map(async (fileName) => {
      const response = await fetch(`stages/${fileName}`, { cache: "no-store" });
      if (!response.ok) {
        throw new Error(`stages/${fileName} の読み込みに失敗しました (status: ${response.status})`);
      }
      return normalizeStage(await response.json());
    })
  );

  return stages.sort((a, b) => a.id - b.id);
}
