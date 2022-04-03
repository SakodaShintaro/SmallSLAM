#ifndef DATA_ASSOCIATOR_LS_H_
#define DATA_ASSOCIATOR_LS_H_

#include "DataAssociator.h"

// 線形探索を用いて、現在スキャンと参照スキャン間の点の対応づけを行う
class DataAssociatorLS : public DataAssociator {
 private:
  std::vector<const LPoint2D *> baseLps;  // 参照スキャンの点を格納しておく。作業用

 public:
  DataAssociatorLS() {}

  ~DataAssociatorLS() {}

  // 参照スキャンの点rlpsをポインタにしてbaseLpsに入れる
  virtual void setRefBase(const std::vector<LPoint2D> &rlps) {
    baseLps.clear();
    for (size_t i = 0; i < rlps.size(); i++) {
      baseLps.push_back(&rlps[i]);  // ポインタにして格納
    }
  }

  virtual double findCorrespondence(const Scan2D *curScan, const Pose2D &predPose);
};

#endif
