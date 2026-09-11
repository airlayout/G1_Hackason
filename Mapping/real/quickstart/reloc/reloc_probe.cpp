// mola_relocalization の SE(2) 尤度探索を、ROS 抜きで 1 回だけ回す試作。
//
// ## なぜ C++ なのか
//
// mola_relocalization は **C++ ライブラリしか無い**（ノードもサービスも Python
// バインディングも無い。2026-09-11 に apt と dpkg -L で確認）。
// 稼働中の MOLA が出している /relocalize_near_pose は **探索をしない**
// （LidarOdometry_Relocalization.cpp は渡された姿勢を FixedPose で置くだけ）。
// 探索はこちらで組む必要がある。
//
// ## 何をするか
//
//   map.mm（mp2p_icp::metric_map_t）＋ base_link 系の 1 スキャン
//     -> ROI を SE(2) の格子に切り、各点にロボットを置いて観測尤度を評価
//     -> 上位の姿勢を尤度順に印字（ICP は回さない）
//
// 真値を渡せば誤差も出す。記録から 1 枚出す側は export_scan.py。
//
// 使い方:
//   # 探索だけ
//   reloc_probe --map <map.mm> --scan <scan.xyz> --center X Y
//               [--roi 3.0] [--res-xy 0.5] [--res-phi 30] [--top 5] [--truth X Y YAW_DEG]
//
//   # 残差の分布だけ（ゲートの較正に使う）
//   reloc_probe --map <map.mm> --scan <scan.xyz> --residuals X Y YAW_DEG
//
//   # 較正 -> 探索 -> ゲートを通す（本番の形）
//   reloc_probe --map <map.mm> --scan <scan.xyz> --relocalize --center X Y \
//               --trusted-pose TX TY TYAW   (または --r0 0.070)
//               [--band-lo 1.3] [--gate-n 2.0] [--min-match 0.50]
//
// ⚠️ **探索は全点、ゲートは壁の帯（z > band-lo）。使う点群が逆になる**（段 6 実測）。
//    帯で探索すると 1.6〜2.9 m 外し、全点でゲートすると 0.5 m の誤りが 1.45 倍にしか
//    ならず弾けない。--relocalize はこれを中で固定するので、外から取り違えられない。

#include <mola_relocalization/relocalization.h>
#include <mp2p_icp/metricmap.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservationPointCloud.h>
// ⚠️ map.mm の層は mola::HashedVoxelPointCloud（CPointsMap ではない）。
// リンクしないと load_from_file が「class not registered」で落ちる
#include <mola_metric_maps/HashedVoxelPointCloud.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <chrono>
#include <numeric>
#include <vector>
#include <cstdlib>
#include <string>

namespace
{
struct Args
{
  std::string map, scan;
  double cx = 0, cy = 0, roi = 3.0;
  double resXY = 0.5, resPhiDeg = 30.0;
  int top = 5;
  bool hasTruth = false;
  double tx = 0, ty = 0, tyawDeg = 0;
  // --residuals: 探索せず、与えた姿勢での「点 -> 地図の最近傍距離」の分布を出す。
  // sigma_dist（＝期待される残差の標準偏差）を**測って決める**ため
  bool residualsOnly = false;
  double rx = 0, ry = 0, ryawDeg = 0;
  // --relocalize: 較正（帯）-> 探索（全点）-> ゲート（帯）の 3 段を通す。
  // ⚠️ 探索とゲートで使う点群が逆なのは実測（段 6）。中で固定し、外から取り違えられなくする
  bool relocalize = false;
  double bandLo = 1.3;          // ゲートで使う帯の下限 [m]（base_link 系の z）
  // r > gateN * r0 なら「自信なし」。
  // ⚠️ **安全側に寄せる。**間違った姿勢を採用すると MOLA が黙って違う場所で再開し、
  //    しかも間違った場所の ICP 品質は平常より高く出る（既知の型）。落とす方が安い。
  // 2026-09-11 実測: 正例 1.08x / 部屋の中の誤り 2.45x / 部屋の別の場所 8.62x。
  //    1.5 なら正例に 39%・誤りに 63% の余裕。2.0 だと誤り側が 23% しかない
  double gateN = 1.5;
  double r0 = -1;               // 較正済みの基準値。渡さなければ --trusted-pose から測る
  bool hasTrusted = false;
  double ux = 0, uy = 0, uyawDeg = 0;
  double minMatchRate = 0.50;   // これを割ったら「地図の外」
  // 詰め: 格子の 1 セルぶんを細かく走り、**帯の残差が最小**になる姿勢を探す。
  // ⚠️ これが無いと、格子の yaw 量子化（30 度の半分）だけで残差が 2 倍になり、
  //    正解をゲートが弾く（2026-09-11 実測: 真値ちょうどでも 0.070 -> 0.142）
  double refineXY = 0.10;       // 詰めの刻み [m]。0 で詰めない
  double refinePhiDeg = 5.0;
  std::string jsonOut;      // 結果を機械可読で落とす（段 8 の駆動スクリプトが読む）
  std::string gridOut;      // 尤度格子（phi 方向の最大）を落とす。動画のヒートマップ用
  // 観測尤度のつまみ。**参照地図の層に載せる**（relocalization.h の注記どおり）。
  // ⚠️ 既定は「触らない」。層の型ごとに妥当な既定が違う
  //    （CPointsMap は sigma_dist 0.0025、HashedVoxelPointCloud は 0.5）ので、
  //    こちらの都合で黙って上書きしない。
  double sigma = -1, maxCorr = -1;
  int decimation = -1;
};

[[noreturn]] void die(const std::string& msg)
{
  std::fprintf(stderr, "[reloc_probe] %s\n", msg.c_str());
  std::exit(2);
}

Args parse(int argc, char** argv)
{
  Args a;
  for (int i = 1; i < argc; i++)
  {
    const std::string k = argv[i];
    auto next = [&](int n) -> const char* {
      if (i + n >= argc) die("引数が足りない: " + k);
      return argv[i + n];
    };
    if (k == "--map") { a.map = next(1); i += 1; }
    else if (k == "--scan") { a.scan = next(1); i += 1; }
    else if (k == "--center") { a.cx = atof(next(1)); a.cy = atof(next(2)); i += 2; }
    else if (k == "--roi") { a.roi = atof(next(1)); i += 1; }
    else if (k == "--res-xy") { a.resXY = atof(next(1)); i += 1; }
    else if (k == "--res-phi") { a.resPhiDeg = atof(next(1)); i += 1; }
    else if (k == "--top") { a.top = atoi(next(1)); i += 1; }
    else if (k == "--sigma") { a.sigma = atof(next(1)); i += 1; }
    else if (k == "--max-corr") { a.maxCorr = atof(next(1)); i += 1; }
    else if (k == "--decim") { a.decimation = atoi(next(1)); i += 1; }
    else if (k == "--relocalize") { a.relocalize = true; }
    else if (k == "--band-lo") { a.bandLo = atof(next(1)); i += 1; }
    else if (k == "--gate-n") { a.gateN = atof(next(1)); i += 1; }
    else if (k == "--r0") { a.r0 = atof(next(1)); i += 1; }
    else if (k == "--min-match") { a.minMatchRate = atof(next(1)); i += 1; }
    else if (k == "--refine-xy") { a.refineXY = atof(next(1)); i += 1; }
    else if (k == "--refine-phi") { a.refinePhiDeg = atof(next(1)); i += 1; }
    else if (k == "--json") { a.jsonOut = next(1); i += 1; }
    else if (k == "--dump-grid") { a.gridOut = next(1); i += 1; }
    else if (k == "--trusted-pose")
    {
      a.hasTrusted = true;
      a.ux = atof(next(1)); a.uy = atof(next(2)); a.uyawDeg = atof(next(3)); i += 3;
    }
    else if (k == "--residuals")
    {
      a.residualsOnly = true;
      a.rx = atof(next(1)); a.ry = atof(next(2)); a.ryawDeg = atof(next(3)); i += 3;
    }
    else if (k == "--truth")
    {
      a.hasTruth = true;
      a.tx = atof(next(1)); a.ty = atof(next(2)); a.tyawDeg = atof(next(3)); i += 3;
    }
    else die("知らない引数: " + k);
  }
  if (a.map.empty() || a.scan.empty()) die("--map と --scan は必須");
  return a;
}

double wrapDeg(double d)
{
  while (d > 180.0) d -= 360.0;
  while (d < -180.0) d += 360.0;
  return d;
}
}  // namespace

int main(int argc, char** argv)
{
  const Args a = parse(argc, argv);

  // ── 参照地図 ──────────────────────────────────────────────────────
  mp2p_icp::metric_map_t ref;
  if (!ref.load_from_file(a.map)) die("地図を読めない: " + a.map);
  std::printf("参照地図 %s\n", a.map.c_str());
  std::printf("  層 %zu 個 / 全点 %zu\n", ref.layers.size(), ref.size());

  // ⚠️ 尤度の出し方は**参照地図の層に載っているパラメータ**が決める
  //    （relocalization.h の注記）。層の型で置き場所が違うので両方を見る。
  //    指定が無ければ触らず、**実際に効いている値を印字する**（黙って変えない）。
  size_t usable = 0;
  auto apply = [&](const char* name, size_t n, const char* unit, double& sg, double& mc,
                   uint32_t& dc, const char* kind) {
    if (a.sigma > 0) sg = a.sigma;
    if (a.maxCorr > 0) mc = a.maxCorr;
    if (a.decimation > 0) dc = static_cast<uint32_t>(a.decimation);
    std::printf("  層 '%s' [%s] %zu %s / sigma %.4f / max_corr %.2f / decim %u\n",
                name, kind, n, unit, sg, mc, dc);
    usable++;
  };
  for (auto& [name, layer] : ref.layers)
  {
    if (auto pts = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(layer); pts)
    {
      auto& o = pts->likelihoodOptions;
      apply(name.c_str(), pts->size(), "点", o.sigma_dist, o.max_corr_distance,
            o.decimation, "CPointsMap");
    }
    else if (auto vox = std::dynamic_pointer_cast<mola::HashedVoxelPointCloud>(layer); vox)
    {
      auto& o = vox->likelihoodOptions;
      // ⚠️ 点数を数えるには全 voxel を走る必要があるので、ここは voxel 数を出す
      apply(name.c_str(), vox->voxels().size(), "voxel", o.sigma_dist,
            o.max_corr_distance, o.decimation, "HashedVoxelPointCloud");
    }
    else
    {
      std::printf("  層 '%s' [%s] は尤度を出せない型なので飛ばす\n",
                  name.c_str(), layer->GetRuntimeClass()->className);
    }
  }
  if (!usable) die("参照地図に尤度を出せる層が無い");

  // ── 観測（base_link 系）────────────────────────────────────────────
  auto cloud = mrpt::maps::CSimplePointsMap::Create();
  if (!cloud->load3D_from_text_file(a.scan)) die("スキャンを読めない: " + a.scan);
  std::printf("観測 %s: %zu 点（base_link 系）\n", a.scan.c_str(), cloud->size());

  // ── 残差を測る道具（--residuals と --relocalize のゲートが共有する）────────
  mola::HashedVoxelPointCloud::Ptr vox;
  for (auto& [name, layer] : ref.layers)
    if (auto v = std::dynamic_pointer_cast<mola::HashedVoxelPointCloud>(layer); v) vox = v;

  // bandLo 未満の点を捨てて（＝壁の帯だけにして）、姿勢を当てて最近傍距離を集める。
  // bandLo に -1e9 を渡せば全点。
  auto residualsAt = [&](double px_, double py_, double yawDeg, double bandLo)
      -> std::tuple<double, size_t, size_t> {   // 中央値 / 引けた点 / 帯に入った点
    if (!vox) die("残差は HashedVoxelPointCloud の層にだけ対応している");
    const double c = std::cos(mrpt::DEG2RAD(yawDeg));
    const double sn = std::sin(mrpt::DEG2RAD(yawDeg));
    std::vector<float> dists;
    size_t inBand = 0;
    for (size_t i = 0; i < cloud->size(); i++)
    {
      float px, py, pz;
      cloud->getPoint(i, px, py, pz);
      if (pz < bandLo) continue;   // ⚠️ 帯は base_link 系の z（= 床からの高さ）
      inBand++;
      const mrpt::math::TPoint3Df q(static_cast<float>(px_ + c * px - sn * py),
                                    static_cast<float>(py_ + sn * px + c * py), pz);
      mrpt::math::TPoint3Df hit;
      float d2 = 0;
      uint64_t id = 0;
      if (vox->nn_single_search(q, hit, d2, id)) dists.push_back(std::sqrt(d2));
    }
    if (dists.empty()) return {-1.0, 0, inBand};
    std::sort(dists.begin(), dists.end());
    return {dists[dists.size() / 2], dists.size(), inBand};
  };

  // ── --residuals: 探索せず、与えた姿勢での残差の分布だけ出す ───────────
  if (a.residualsOnly)
  {
    if (!vox) die("--residuals は HashedVoxelPointCloud の層にだけ対応している");
    const double c = std::cos(mrpt::DEG2RAD(a.ryawDeg));
    const double sn = std::sin(mrpt::DEG2RAD(a.ryawDeg));
    std::vector<float> dists;
    for (size_t i = 0; i < cloud->size(); i++)
    {
      float px, py, pz;
      cloud->getPoint(i, px, py, pz);
      const mrpt::math::TPoint3Df q(static_cast<float>(a.rx + c * px - sn * py),
                                    static_cast<float>(a.ry + sn * px + c * py), pz);
      mrpt::math::TPoint3Df hit;
      float d2 = 0;
      uint64_t id = 0;
      if (vox->nn_single_search(q, hit, d2, id)) dists.push_back(std::sqrt(d2));
    }
    if (dists.empty()) die("最近傍が 1 点も引けなかった");
    std::sort(dists.begin(), dists.end());
    auto pct = [&](double pp) { return dists[static_cast<size_t>(pp * (dists.size() - 1))]; };
    double sum2 = 0;
    for (float d : dists) sum2 += double(d) * d;
    std::printf("\n姿勢 (%.3f, %.3f, %.2f deg) での 点->地図 の最近傍距離 [m]\n",
                a.rx, a.ry, a.ryawDeg);
    std::printf("  引けた点 %zu / %zu\n", dists.size(), cloud->size());
    std::printf("  p10 %.4f  p25 %.4f  中央 %.4f  p75 %.4f  p90 %.4f  p95 %.4f\n",
                pct(0.10), pct(0.25), pct(0.50), pct(0.75), pct(0.90), pct(0.95));
    std::printf("  RMS %.4f  平均 %.4f\n", std::sqrt(sum2 / dists.size()),
                std::accumulate(dists.begin(), dists.end(), 0.0) / dists.size());
    std::printf("\n→ sigma_dist は**標準偏差[m]**（HashedVoxelPointCloud）だが、"
                "**探索の答えは変えない**（段 6）。この残差はゲート用\n");
    return 0;
  }

  auto obs = mrpt::obs::CObservationPointCloud::Create();
  obs->pointcloud = cloud;
  obs->sensorPose = mrpt::poses::CPose3D::Identity();  // 既に base_link 系

  // ── 1. 較正（--relocalize）: 信頼できる姿勢での**帯の**残差中央値 r0 ──────
  double r0 = a.r0;
  if (a.relocalize && r0 <= 0)
  {
    if (!a.hasTrusted)
      die("--relocalize には --r0 か --trusted-pose のどちらかが要る"
          "（ゲートの基準が無いと採否を出せない）");
    const auto [med, matched, inBand] = residualsAt(a.ux, a.uy, a.uyawDeg, a.bandLo);
    if (med < 0) die("較正: 信頼できる姿勢で最近傍が 1 点も引けなかった");
    r0 = med;
    std::printf("\n[1/3] 較正: 姿勢 (%.3f, %.3f, %.2f deg) / 帯 z>%.2f m の %zu 点\n",
                a.ux, a.uy, a.uyawDeg, a.bandLo, inBand);
    std::printf("      基準 r0 = %.4f m（一致 %zu/%zu）\n", r0, matched, inBand);
  }

  // ── 探索 ──────────────────────────────────────────────────────────
  mola::RelocalizationLikelihood_SE2::Input in;
  in.reference_map = ref;
  in.observations.insert(obs);
  in.corner_min = {a.cx - a.roi, a.cy - a.roi, -M_PI};
  in.corner_max = {a.cx + a.roi, a.cy + a.roi, M_PI};
  in.resolution_xy = a.resXY;
  in.resolution_phi = mrpt::DEG2RAD(a.resPhiDeg);

  const double nx = 2 * a.roi / a.resXY + 1;
  const double nphi = 360.0 / a.resPhiDeg;
  std::printf("%s探索 ROI ±%.1f m / 刻み %.2f m・%.0f deg = 約 %.0f 点"
              "（⚠️ **全点で**探索する。帯で探索すると壊れる＝段 6）\n",
              a.relocalize ? "\n[2/3] " : "", a.roi, a.resXY, a.resPhiDeg, nx * nx * nphi);

  const auto out = mola::RelocalizationLikelihood_SE2::run(in);
  std::printf("所要 %.3f s / log尤度 %.3f .. %.3f\n",
              out.time_cost, out.min_log_likelihood, out.max_log_likelihood);

  // 尤度格子を phi 方向の最大で潰して落とす（動画のヒートマップ用）。
  // ⚠️ 中身は CPosePDFGrid のセル値＝**正規化済みの尤度**（対数ではない）。
  //    全セルの和が 1 になるので、絶対値でなく**相対の高低**を見る
  if (!a.gridOut.empty())
  {
    const auto& g = out.likelihood_grid;
    FILE* f = std::fopen(a.gridOut.c_str(), "w");
    if (!f) die("格子を書けない: " + a.gridOut);
    std::fprintf(f, "# xmin ymin res nx ny （値は phi 方向の最大の尤度。和が 1 に正規化済み）\n");
    std::fprintf(f, "%.4f %.4f %.4f %zu %zu\n", g.getXMin(), g.getYMin(),
                 g.getResolutionXY(), g.getSizeX(), g.getSizeY());
    for (size_t iy = 0; iy < g.getSizeY(); iy++)
    {
      for (size_t ix = 0; ix < g.getSizeX(); ix++)
      {
        double best_v = -1e300;
        for (size_t ip = 0; ip < g.getSizePhi(); ip++)
          best_v = std::max(best_v, static_cast<double>(*g.getByIndex(ix, iy, ip)));
        std::fprintf(f, "%.6g%s", best_v, ix + 1 == g.getSizeX() ? "" : " ");
      }
      std::fprintf(f, "\n");
    }
    std::fclose(f);
    std::printf("格子を書いた: %s（%zu x %zu）\n", a.gridOut.c_str(),
                g.getSizeX(), g.getSizeY());
  }

  const auto best = mola::find_best_poses_se2(out.likelihood_grid, 0.99);
  if (best.empty()) die("上位の姿勢が 1 つも返らなかった");

  std::printf("\n上位 %d 件（尤度の高い順）:\n", a.top);
  int shown = 0;
  for (auto it = best.rbegin(); it != best.rend() && shown < a.top; ++it, ++shown)
  {
    const auto& p = it->second;
    std::printf("  %d) x %7.3f  y %7.3f  yaw %7.2f deg   尤度 %.4g",
                shown + 1, p.x, p.y, mrpt::RAD2DEG(p.phi), it->first);
    if (a.hasTruth)
    {
      const double e = std::hypot(p.x - a.tx, p.y - a.ty);
      const double ey = wrapDeg(mrpt::RAD2DEG(p.phi) - a.tyawDeg);
      std::printf("   | 真値との差 %.3f m / %.2f deg", e, ey);
    }
    std::printf("\n");
  }
  if (a.hasTruth)
    std::printf("\n真値: x %7.3f  y %7.3f  yaw %7.2f deg\n", a.tx, a.ty, a.tyawDeg);

  // ── 3. ゲート（--relocalize）: 詰めてから**帯で**測り直して採否を出す ────
  if (a.relocalize)
  {
    const auto& p = best.rbegin()->second;
    double bx = p.x, by = p.y, yawDeg = mrpt::RAD2DEG(p.phi);

    // 詰め: 粗い格子の 1 セルぶんを細かく走り、帯の残差が最小になる姿勢を採る。
    // ⚠️ 詰めずにゲートすると、yaw の量子化だけで残差が 2 倍になり正解を弾く（実測）。
    //    ICP を回さずに済むのは、ゲートの指標そのものを最小化しているから
    if (a.refineXY > 0)
    {
      const auto t0 = std::chrono::steady_clock::now();
      double bestR = 1e9;
      double rbx = bx, rby = by, rbyaw = yawDeg;
      size_t evals = 0;
      for (double dx = -a.resXY / 2; dx <= a.resXY / 2 + 1e-9; dx += a.refineXY)
        for (double dy = -a.resXY / 2; dy <= a.resXY / 2 + 1e-9; dy += a.refineXY)
          for (double dp = -a.resPhiDeg / 2; dp <= a.resPhiDeg / 2 + 1e-9; dp += a.refinePhiDeg)
          {
            const auto [rr, mm, nn] = residualsAt(bx + dx, by + dy, yawDeg + dp, a.bandLo);
            evals++;
            if (rr > 0 && rr < bestR) { bestR = rr; rbx = bx + dx; rby = by + dy; rbyaw = yawDeg + dp; }
          }
      const double secs = std::chrono::duration<double>(
                              std::chrono::steady_clock::now() - t0).count();
      std::printf("\n[3/3] 詰め: %zu 通り / %.2f s -> (%.3f, %.3f, %.2f deg)"
                  "  移動 %.3f m / %.2f deg\n",
                  evals, secs, rbx, rby, rbyaw,
                  std::hypot(rbx - bx, rby - by), wrapDeg(rbyaw - yawDeg));
      bx = rbx; by = rby; yawDeg = rbyaw;
      if (a.hasTruth)
        std::printf("      詰めたあとの真値との差 %.3f m / %.2f deg\n",
                    std::hypot(bx - a.tx, by - a.ty), wrapDeg(yawDeg - a.tyawDeg));
    }

    const auto [r, matched, inBand] = residualsAt(bx, by, yawDeg, a.bandLo);
    const double rate = inBand ? double(matched) / double(inBand) : 0.0;
    std::printf("      ゲート: 姿勢 (%.3f, %.3f, %.2f deg) を帯 z>%.2f m で測り直す\n",
                bx, by, yawDeg, a.bandLo);
    std::printf("      残差 r = %.4f m / 基準 r0 = %.4f m -> **%.2f 倍**"
                "（しきい N = %.1f）\n", r < 0 ? 0.0 : r, r0, r < 0 ? 99.0 : r / r0, a.gateN);
    std::printf("      一致率 %.1f%%（%zu/%zu。しきい %.0f%%）\n",
                100.0 * rate, matched, inBand, 100.0 * a.minMatchRate);

    // ⚠️ 「地図の外」と「部屋の中で間違えた」は別の症状。別々に言う（段 6 実測）
    const char* verdict = "採用";
    int rc = 0;
    if (rate < a.minMatchRate) { verdict = "自信なし（地図の外）"; rc = 3; }
    else if (r < 0 || r > a.gateN * r0) { verdict = "自信なし（部屋の中だが合っていない）"; rc = 3; }

    if (rc == 3 && rate < a.minMatchRate)
      std::printf("\n**判定: %s。**帯の点の %.1f%% しか地図に当たらない\n", verdict, 100.0 * rate);
    else if (rc == 3)
      std::printf("\n**判定: %s。**この姿勢を /relocalize_near_pose に投げてはいけない\n", verdict);
    else
      std::printf("\n**判定: %s。**/relocalize_near_pose に投げる"
                  "（共分散は格子の刻み %.2f m 相当）\n", verdict, a.resXY);

    if (!a.jsonOut.empty())
    {
      FILE* f = std::fopen(a.jsonOut.c_str(), "w");
      if (!f) die("json を書けない: " + a.jsonOut);
      std::fprintf(f,
          "{\n  \"center\": [%.4f, %.4f],\n  \"roi\": %.3f,\n"
          "  \"r0\": %.5f,\n  \"gate_n\": %.2f,\n  \"min_match\": %.3f,\n"
          "  \"band_lo\": %.3f,\n  \"search_seconds\": %.4f,\n"
          "  \"coarse\": [%.4f, %.4f, %.3f],\n"
          "  \"pose\": [%.4f, %.4f, %.3f],\n"
          "  \"residual\": %.5f,\n  \"ratio\": %.4f,\n"
          "  \"match_rate\": %.5f,\n  \"matched\": %zu,\n  \"in_band\": %zu,\n"
          "  \"verdict\": \"%s\",\n  \"accepted\": %s",
          a.cx, a.cy, a.roi, r0, a.gateN, a.minMatchRate, a.bandLo, out.time_cost,
          p.x, p.y, mrpt::RAD2DEG(p.phi), bx, by, yawDeg,
          r < 0 ? -1.0 : r, r < 0 ? -1.0 : r / r0, rate, matched, inBand,
          verdict, rc == 0 ? "true" : "false");
      if (a.hasTruth)
        std::fprintf(f, ",\n  \"truth\": [%.4f, %.4f, %.3f],\n"
                        "  \"error_m\": %.4f,\n  \"error_deg\": %.3f",
                     a.tx, a.ty, a.tyawDeg, std::hypot(bx - a.tx, by - a.ty),
                     wrapDeg(yawDeg - a.tyawDeg));
      std::fprintf(f, "\n}\n");
      std::fclose(f);
      std::printf("json を書いた: %s\n", a.jsonOut.c_str());
    }
    return rc;
  }
  return 0;
}
