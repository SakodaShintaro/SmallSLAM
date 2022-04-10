#ifndef _REF_SCAN_MAKER_BS_H_
#define _REF_SCAN_MAKER_BS_H_

#include "RefScanMaker.h"

class RefScanMakerBS : public RefScanMaker {
 public:
  virtual const Scan2D *makeRefScan();
};

#endif
