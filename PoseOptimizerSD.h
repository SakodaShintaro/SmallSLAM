#ifndef _POSE_OPTIMIZER_SD_H_
#define _POSE_OPTIMIZER_SD_H_

#include "PoseOptimizer.h"

// 最急降下法でコスト関数を最小化する
class PoseOptimizerSD : public PoseOptimizer {
 public:
  double optimizePose(Pose2D &initPose, Pose2D &estPose) override;
};

#endif
