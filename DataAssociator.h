#ifndef DATA_ASSOCIATOR_H_
#define DATA_ASSOCIATOR_H_

#include <vector>

#include "LPoint2D.h"
#include "MyUtil.h"
#include "Pose2D.h"
#include "Scan2D.h"

class DataAssociator {
 public:
  std::vector<const LPoint2D *> curLps;  // 対応がとれた現在スキャンの点群
  std::vector<const LPoint2D *> refLps;  // 対応がとれた参照スキャンの点群

  DataAssociator() {}

  ~DataAssociator() {}

  virtual void setRefBase(const std::vector<LPoint2D> &lps) = 0;
  virtual double findCorrespondence(const Scan2D *curScan, const Pose2D &predPose) = 0;
};

#endif
