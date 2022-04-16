#ifndef SLAM_FRONT_END_H_
#define SLAM_FRONT_END_H_

#include <boost/circular_buffer.hpp>
#include <vector>

#include "LoopDetector.h"
#include "PointCloudMap.h"
#include "PoseGraph.h"
#include "ScanMatcher2D.h"
#include "SlamBackEnd.h"

// SLAMフロントエンド。ロボット位置推定、地図生成、ループ閉じ込みを取り仕切る
class SlamFrontEnd {
 private:
  int cnt;           // 論理時刻
  int keyframeSkip;  // キーフレーム間隔

  PointCloudMap *pcmap;  // 点群地図
  PoseGraph *pg;         // ポーズグラフ
  ScanMatcher2D *smat;   // スキャンマッチング
  LoopDetector *lpd;     // ループ検出器
  SlamBackEnd sback;     // SLAMバックエンド

 public:
  SlamFrontEnd() : cnt(0), keyframeSkip(10), smat(nullptr), lpd(nullptr) {
    pg = new PoseGraph();
    sback.setPoseGraph(pg);
  }

  ~SlamFrontEnd() {
    delete pg;
  }
  void setScanMatcher(ScanMatcher2D *s) { smat = s; }

  void setLoopDetector(LoopDetector *l) {
    lpd = l;
    lpd->setPoseGraph(pg);
  }

  void setPointCloudMap(PointCloudMap *p) { pcmap = p; }

  void setRefScanMaker(RefScanMaker *r) { smat->setRefScanMaker(r); }

  PointCloudMap *getPointCloudMap() { return (pcmap); }

  PoseGraph *getPoseGraph() { return (pg); }

  int getCnt() { return (cnt); }

  void setDgCheck(bool p) { smat->setDgCheck(p); }

  // デバッグ用
  std::vector<LoopMatch> &getLoopMatches() { return (lpd->getLoopMatches()); }

  // デバッグ用
  std::vector<PoseCov> &getPoseCovs() { return (smat->getPoseCovs()); }

  void init();
  void process(Scan2D &scan);
  bool makeOdometryArc(Pose2D &curPose, const Eigen::Matrix3d &cov);

  void countLoopArcs();
};

#endif
