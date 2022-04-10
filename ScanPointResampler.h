#ifndef SCAN_POINT_RESAMPLER_H
#define SCAN_POINT_RESAMPLER_H

#include <vector>

#include "LPoint2D.h"
#include "MyUtil.h"
#include "Scan2D.h"

class ScanPointResampler {
 public:
  ScanPointResampler() : dthreS(0.05), dthreL(0.25), dis(0) {}

  void setDthre(int s, int l) {
    dthreS = s;
    dthreL = l;
  }

  void resamplePoints(Scan2D* scan);
  bool findInterpolatePoint(const LPoint2D& cp, const LPoint2D& pp, LPoint2D& np, bool& inserted);

 private:
  double dthreS;  // 点の距離間隔[m]
  double dthreL;  // 点の距離閾値[m]。この間隔を超えたら補間しない
  double dis;     // 累積距離。作業用
};

#endif
