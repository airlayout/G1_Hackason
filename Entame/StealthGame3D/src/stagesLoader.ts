import type { Stage } from "./types";

// 2D版 (StealthGame2D/stages) を public/stages のシンボリックリンク経由でそのまま読み込む。
export async function loadStages(): Promise<Stage[]> {
  const manifestResponse = await fetch("stages/manifest.json");
  if (!manifestResponse.ok) {
    throw new Error(`stages/manifest.json の読み込みに失敗しました (status: ${manifestResponse.status})`);
  }
  const fileNames: string[] = await manifestResponse.json();

  const stages = await Promise.all(
    fileNames.map(async (fileName) => {
      const response = await fetch(`stages/${fileName}`);
      if (!response.ok) {
        throw new Error(`stages/${fileName} の読み込みに失敗しました (status: ${response.status})`);
      }
      return (await response.json()) as Stage;
    })
  );

  return stages.sort((a, b) => a.id - b.id);
}
