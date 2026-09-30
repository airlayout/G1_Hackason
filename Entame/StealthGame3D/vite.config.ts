import { defineConfig } from "vite";

// 2D版のステージJSONをそのまま public/ 経由で配信する
export default defineConfig({
  publicDir: "public",
  server: {
    fs: {
      allow: [".."],
    },
  },
});
