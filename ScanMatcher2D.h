#ifndef SCAN_MATCHER2D_H_
#define SCAN_MATCHER2D_H_

#include <vector>

#include "LPoint2D.h"
#include "MyUtil.h"
#include "PointCloudMap.h"
#include "Pose2D.h"
#include "PoseEstimatorICP.h"
#include "PoseFuser.h"
#include "RefScanMaker.h"
#include "Scan2D.h"
#include "ScanPointAnalyser.h"
#include "ScanPointResampler.h"

// ICPを用いてスキャンマッチングを行う
class ScanMatcher2D {
 public:
  ScanMatcher2D()
      : cnt(-1),
        scthre(1.0),
        nthre(50),
        dgcheck(false),
        atd(0),
        pcmap(nullptr),
        spres(nullptr),
        spana(nullptr),
        estim(nullptr),
        rsm(nullptr),
        pfu(nullptr) {}

  void setPoseEstimator(PoseEstimatorICP *p) { estim = p; }
  void setPoseFuser(PoseFuser *p) { pfu = p; }

  void setScanPointResampler(ScanPointResampler *s) { spres = s; }

  void setScanPointAnalyser(ScanPointAnalyser *s) { spana = s; }

  void setRefScanMaker(RefScanMaker *r) {
    rsm = r;
    if (pcmap != nullptr) rsm->setPointCloudMap(pcmap);
  }

  void setPointCloudMap(PointCloudMap *m) {
    pcmap = m;
    if (rsm != nullptr) rsm->setPointCloudMap(pcmap);
  }

  void reset() { cnt = -1; }

  void setInitPose(Pose2D &p) { initPose = p; }

  void setDgCheck(bool t) { dgcheck = t; }

  Eigen::Matrix3d &getCovariance() { return cov; }

  std::vector<PoseCov> &getPoseCovs() { return poseCovs; }

  bool matchScan(Scan2D &scan);

  void growMap(const Scan2D &scan, const Pose2D &pose);

 private:
  int cnt;                        // 論理時刻。スキャン番号に対応
  Scan2D prevScan;                // 1つ前のスキャン
  Pose2D initPose;                // 地図の原点の位置。通常(0, 0, 0)
  double scthre;                  // スコア閾値。これより大きいとICP失敗とみなす
  double nthre;                   // 使用点数閾値。これより小さいとICP失敗とみなす
  double atd;                     // 累積走行距離。確認用
  bool dgcheck;                   // 退化処理をするか
  PoseEstimatorICP *estim;        // ロボット位置推定器
  PointCloudMap *pcmap;           // 点群地図
  ScanPointResampler *spres;      // スキャン点間隔均一化
  ScanPointAnalyser *spana;       // スキャン点法線計算
  RefScanMaker *rsm;              // 参照スキャン生成
  PoseFuser *pfu;                 // センサ融合器
  Eigen::Matrix3d cov;            // ロボット移動量の共分散行列
  Eigen::Matrix3d totalCov;       // ロボット位置の共分散行列
  std::vector<PoseCov> poseCovs;  // デバッグ用
};

#endif
