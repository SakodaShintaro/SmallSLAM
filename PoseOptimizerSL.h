#ifndef _POSE_OPTIMIZER_SL_H_
#define _POSE_OPTIMIZER_SL_H_

#include "PoseOptimizer.h"

// 直線探索つきの最急降下法でコスト関数を最小化する
class PoseOptimizerSL : public PoseOptimizer {
 public:
  virtual double optimizePose(Pose2D &initPose, Pose2D &estPose);
  double search(double ev0, Pose2D &pose, Pose2D &dp);
  double objFunc(double tt, Pose2D &pose, Pose2D &dp);
};

#endif