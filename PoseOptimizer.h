#ifndef _POSE_OPTIMIZER_H_
#define _POSE_OPTIMIZER_H_

#include <vector>

#include "CostFunction.h"
#include "LPoint2D.h"
#include "MyUtil.h"
#include "Pose2D.h"

class PoseOptimizer {
 public:
  PoseOptimizer() : evthre(0.000001), dd(0.00001), da(0.00001), cfunc(nullptr) {
    allN = 0;
    sum = 0;
  }

  void setCostFunction(CostFunction *f) { cfunc = f; }

  void setEvlimit(double l) { cfunc->setEvlimit(l); }

  void setPoints(std::vector<const LPoint2D *> &curLps, std::vector<const LPoint2D *> &refLps) {
    cfunc->setPoints(curLps, refLps);
  }

  void setEvthre(double inthre) { this->evthre = inthre; }
  double getPnrate() { return (cfunc->getPnrate()); }

  void setDdDa(double d, double a) {
    dd = d;
    da = a;
  }

  virtual double optimizePose(Pose2D &initPose, Pose2D &estPose) = 0;

  int allN;    // 繰り返し総数。テスト用
  double sum;  // 残差合計。テスト用

 protected:
  double evthre;        // コスト変化閾値。変化量がこれ以下なら繰り返し終了
  double dd;            // 数値微分の刻み（並進）
  double da;            // 数値微分の刻み（回転）
  CostFunction* cfunc;  // コスト関数

 private:
};

#endif
