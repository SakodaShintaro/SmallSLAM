#include "RefScanMakerLM.h"

using namespace std;

const Scan2D *RefScanMakerLM::makeRefScan() {
  vector<LPoint2D> &refLps = refScan.lps;  // 参照スキャンの点群のコンテナ
  refLps.clear();

  const vector<LPoint2D> &localMap = pcmap->localMap;  // 点群地図の局所地図
  for (size_t i = 0; i < localMap.size(); i++) {
    const LPoint2D &rp = localMap[i];
    refLps.emplace_back(rp);  // 単にコピー
  }

  return &refScan;
}
