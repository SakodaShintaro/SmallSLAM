#ifndef SLAM_BACK_END_H_
#define SLAM_BACK_END_H_

#include <vector>

#include "PointCloudMap.h"
#include "PoseGraph.h"

class SlamBackEnd {
 private:
  std::vector<Pose2D> newPoses;  // ポーズ調整後の姿勢
  PointCloudMap *pcmap;          // 点群地図
  PoseGraph *pg;                 // ポーズグラフ

 public:
  void setPointCloudMap(PointCloudMap *m) { pcmap = m; }

  void setPoseGraph(PoseGraph *g) { pg = g; }

  Pose2D adjustPoses();
  void remakeMaps();
};

#endif
