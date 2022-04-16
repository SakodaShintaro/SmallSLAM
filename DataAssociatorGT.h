#ifndef DATA_ASSOCIATOR_GT_H_
#define DATA_ASSOCIATOR_GT_H_

#include "DataAssociator.h"
#include "NNGridTable.h"

// 格子テーブルを用いて、現在スキャンと参照スキャン間の点の対応づけを行う
class DataAssociatorGT : public DataAssociator {
 private:
  NNGridTable nntab;  // 格子テーブル

 public:
  // 参照スキャンの点rlpsをポインタにしてnntabに入れる
  virtual void setRefBase(const std::vector<LPoint2D> &rlps) {
    nntab.clear();
    for (size_t i = 0; i < rlps.size(); i++) nntab.addPoint(&rlps[i]);  // ポインタにして格納
  }

  virtual double findCorrespondence(const Scan2D *curScan, const Pose2D &predPose);
};

#endif
