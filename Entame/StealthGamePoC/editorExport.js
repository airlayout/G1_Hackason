// 編集中のステージ1件を、stages/*.json にそのまま保存できる形のJSON文字列に変換する。

export function exportStageJson(stage) {
  return JSON.stringify(stage, null, 2) + "\n";
}
