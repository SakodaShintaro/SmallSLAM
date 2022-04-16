#ifndef LOOP_DETECTOR_SS_H_
#define LOOP_DETECTOR_SS_H_

#include "DataAssociator.h"
#include "LoopDetector.h"
#include "PointCloudMapLP.h"
#include "PoseEstimatorICP.h"
#include "PoseFuser.h"

class LoopDetectorSS : public LoopDetector {
 public:
  LoopDetectorSS() : radius(4), atdthre(10), scthre(0.2) {}

  void setPoseEstimator(PoseEstimatorICP *p) { estim = p; }

  void setPoseFuser(PoseFuser *p) { pfu = p; }

  void setDataAssociator(DataAssociator *d) { dass = d; }

  void setCostFunction(CostFunction *f) { cfunc = f; }

  void setPointCloudMap(PointCloudMapLP *p) { pcmap = p; }

  bool detectLoop(Scan2D *curScan, Pose2D &curPose, int cnt) override;
  void makeLoopArc(LoopInfo &info);
  bool estimateRevisitPose(const Scan2D *curScan, const std::vector<LPoint2D> &refLps, const Pose2D &initPose,
                           Pose2D &revisitPose);

 private:
  double radius;   // 探索半径[m]（現在位置と再訪点の距離閾値）
  double atdthre;  // 累積走行距離の差の閾値[m]
  double scthre;   // ICPスコアの閾値

  PointCloudMapLP *pcmap;   // 点群地図
  CostFunction *cfunc;      // コスト関数
  PoseEstimatorICP *estim;  // ロボット位置推定器（ICP）
  DataAssociator *dass;     // データ対応付け器
  PoseFuser *pfu;           // センサ融合器
};

#endif
