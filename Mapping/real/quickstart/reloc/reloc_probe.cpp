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
//   reloc_probe --map <map.mm> --scan <scan.xyz> --center X Y
//               [--roi 3.0] [--res-xy 0.5] [--res-phi 30] [--top 5]
//               [--truth X Y YAW_DEG] [--sigma 0.0025] [--max-corr 1.0] [--decim 10]

#include <mola_relocalization/relocalization.h>
#include <mp2p_icp/metricmap.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservationPointCloud.h>
// ⚠️ map.mm の層は mola::HashedVoxelPointCloud（CPointsMap ではない）。
// リンクしないと load_from_file が「class not registered」で落ちる
#include <mola_metric_maps/HashedVoxelPointCloud.h>

#include <cmath>
#include <cstdio>
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

  auto obs = mrpt::obs::CObservationPointCloud::Create();
  obs->pointcloud = cloud;
  obs->sensorPose = mrpt::poses::CPose3D::Identity();  // 既に base_link 系

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
  std::printf("探索 ROI ±%.1f m / 刻み %.2f m・%.0f deg = 約 %.0f 点\n",
              a.roi, a.resXY, a.resPhiDeg, nx * nx * nphi);

  const auto out = mola::RelocalizationLikelihood_SE2::run(in);
  std::printf("所要 %.3f s / log尤度 %.3f .. %.3f\n",
              out.time_cost, out.min_log_likelihood, out.max_log_likelihood);

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
  return 0;
}
