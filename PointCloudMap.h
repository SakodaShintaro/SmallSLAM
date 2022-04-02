#ifndef POINT_CLOUD_MAP_H_
#define POINT_CLOUD_MAP_H_

#include <vector>

#include "LPoint2D.h"
#include "MyUtil.h"
#include "Pose2D.h"
#include "Scan2D.h"

// 点群地図の基底クラス
class PointCloudMap {
 public:
  static const int MAX_POINT_NUM = 1000000;  // globalMapの最大点数

  int nthre;                        // 格子テーブルセル点数閾値（GTとLPのみ）
  std::vector<Pose2D> poses;        // ロボット軌跡
  Pose2D lastPose;                  // 最後に推定したロボット位置
  Scan2D lastScan;                  // 最後に処理したスキャン
  std::vector<LPoint2D> globalMap;  // 全体地図。間引き後の点
  std::vector<LPoint2D> localMap;   // 現在位置近傍の局所地図。スキャンマッチングに使う

  PointCloudMap() : nthre(1) { globalMap.reserve(MAX_POINT_NUM); }

  void setNthre(int n) { nthre = n; }

  void setLastPose(const Pose2D &p) { lastPose = p; }

  Pose2D getLastPose() const { return lastPose; }

  void setLastScan(const Scan2D &s) { lastScan = s; }

  virtual void addPose(const Pose2D &p) = 0;
  virtual void addPoints(const std::vector<LPoint2D> &lps) = 0;
  virtual void makeGlobalMap() = 0;
  virtual void makeLocalMap() = 0;
  virtual void remakeMaps(const std::vector<Pose2D> &newPoses) = 0;
};

#endif
