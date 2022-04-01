#ifndef SCAN2D_H_
#define SCAN2D_H_

#include <vector>

#include "LPoint2D.h"
#include "MyUtil.h"
#include "Pose2D.h"

// スキャン
struct Scan2D {
  static double MAX_SCAN_RANGE;  // スキャン点の距離値上限[m]
  static double MIN_SCAN_RANGE;  // スキャン点の距離値下限[m]
  int sid;                       // スキャンid
  Pose2D pose;                   // スキャン取得時のオドメトリ値
  std::vector<LPoint2D> lps;     // スキャン点群
  Scan2D() : sid(0) {}

  void setSid(int s) { sid = s; }

  void setLps(const std::vector<LPoint2D>& ps) { lps = ps; }

  void setPose(Pose2D& p) { pose = p; }
};

#endif
