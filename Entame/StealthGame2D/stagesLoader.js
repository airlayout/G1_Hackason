// ステージデータを stages/manifest.json 経由でJSONファイル群から読み込む。
// 新しいステージを追加するときは stages/*.json を置いて manifest.json に追記するだけでよい。

export async function loadStages() {
  const manifestResponse = await fetch("stages/manifest.json");
  if (!manifestResponse.ok) {
    throw new Error(`stages/manifest.json の読み込みに失敗しました (status: ${manifestResponse.status})`);
  }
  const fileNames = await manifestResponse.json();

  const stages = await Promise.all(
    fileNames.map(async (fileName) => {
      const response = await fetch(`stages/${fileName}`);
      if (!response.ok) {
        throw new Error(`stages/${fileName} の読み込みに失敗しました (status: ${response.status})`);
      }
      return response.json();
    })
  );

  return stages.sort((a, b) => a.id - b.id);
}
