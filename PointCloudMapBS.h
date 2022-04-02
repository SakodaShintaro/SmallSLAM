#ifndef _POINT_CLOUD_MAP_BS_H_
#define _POINT_CLOUD_MAP_BS_H_

#include "PointCloudMap.h"

// スキャン点をすべて保存する点群地図
class PointCloudMapBS : public PointCloudMap {
 public:
  PointCloudMapBS() {}

  virtual void addPose(const Pose2D &p);
  virtual void addPoints(const std::vector<LPoint2D> &lps);
  virtual void makeGlobalMap();
  virtual void makeLocalMap();
  virtual void remakeMaps(const std::vector<Pose2D> &newPoses);
};

#endif