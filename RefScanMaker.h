#ifndef _REF_SCAN_MAKER_H_
#define _REF_SCAN_MAKER_H_

#include <vector>

#include "LPoint2D.h"
#include "MyUtil.h"
#include "PointCloudMap.h"
#include "Pose2D.h"
#include "Scan2D.h"

class RefScanMaker {
 protected:
  const PointCloudMap *pcmap;  // 点群地図
  Scan2D refScan;              // 参照スキャン本体。これを外に提供

 public:
  RefScanMaker() : pcmap(nullptr) {}

  void setPointCloudMap(const PointCloudMap *p) { pcmap = p; }

  virtual const Scan2D *makeRefScan() = 0;
};

#endif
