#ifndef _POSEESTIMATOR_ICP_H_
#define _POSEESTIMATOR_ICP_H_

#include <vector>

#include "DataAssociator.h"
#include "LPoint2D.h"
#include "MyUtil.h"
#include "Pose2D.h"
#include "PoseOptimizer.h"
#include "Scan2D.h"

class PoseEstimatorICP {
 public:
  double totalError = 0;  // 誤差合計
  double totalTime = 0;   // 処理時間合計

  void setPoseOptimizer(PoseOptimizer *p) { popt = p; }

  void setDataAssociator(DataAssociator *d) { dass = d; }

  double getPnrate() { return (pnrate); }

  size_t getUsedNum() { return (usedNum); }

  void setScanPair(const Scan2D *c, const Scan2D *r) {
    curScan = c;
    dass->setRefBase(r->lps);  // データ対応づけのために参照スキャン点を登録
  }

  void setScanPair(const Scan2D *c, const std::vector<LPoint2D> &refLps) {
    curScan = c;
    dass->setRefBase(refLps);  // データ対応づけのために参照スキャン点を登録
  }

  double estimatePose(Pose2D &initPose, Pose2D &estPose);

 private:
  const Scan2D *curScan;  // 現在スキャン
  size_t usedNum = 0;     // ICPに使われた点数。LoopDetectorで信頼性チェックに使う
  double pnrate = 0;      // 正しく対応づけされた点の比率

  PoseOptimizer *popt;   // 最適化クラス
  DataAssociator *dass;  // データ対応付けクラス
};

#endif
