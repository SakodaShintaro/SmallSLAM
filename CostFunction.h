#ifndef _COST_FUNCTION_H_
#define _COST_FUNCTION_H_

#include <vector>

#include "LPoint2D.h"
#include "MyUtil.h"
#include "Pose2D.h"
#include "Scan2D.h"

class CostFunction {
 public:
  CostFunction() : evlimit(0), pnrate(0) {}

  void setEvlimit(double e) { evlimit = e; }

  // DataAssociatorで対応のとれた点群cur, refを設定
  void setPoints(std::vector<const LPoint2D *> &cur, std::vector<const LPoint2D *> &ref) {
    curLps = cur;
    refLps = ref;
  }

  double getPnrate() { return (pnrate); }

  virtual double calValue(double tx, double ty, double th) = 0;

 protected:
  std::vector<const LPoint2D *> curLps;  // 対応がとれた現在スキャンの点群
  std::vector<const LPoint2D *> refLps;  // 対応がとれた参照スキャンの点群
  double evlimit;                        // マッチングで対応がとれたと見なす距離閾値
  double pnrate;                         // 誤差がevlimit以内で対応がとれた点の比率

};

#endif
