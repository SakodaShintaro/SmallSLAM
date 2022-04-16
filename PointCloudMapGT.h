#ifndef POINT_CLOUD_MAP_GT_H_
#define POINT_CLOUD_MAP_GT_H_

#include <boost/unordered_map.hpp>

#include "NNGridTable.h"
#include "PointCloudMap.h"

// 格子テーブルを用いた点群地図
class PointCloudMapGT : public PointCloudMap {
 public:
  std::vector<LPoint2D> allLps;  // 全スキャン点群
  NNGridTable nntab;             // 格子テーブル

 public:
  PointCloudMapGT() {
    allLps.reserve(MAX_POINT_NUM);  // 最初に確保
  }

  virtual void addPose(const Pose2D &p);
  virtual void addPoints(const std::vector<LPoint2D> &lps);
  virtual void makeGlobalMap();
  virtual void makeLocalMap();
  void subsamplePoints(std::vector<LPoint2D> &sps);
  virtual void remakeMaps(const std::vector<Pose2D> &newPoses);
};

#endif
