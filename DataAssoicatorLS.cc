#include <boost/timer.hpp>

#include "DataAssociatorLS.h"

using namespace std;

// 現在スキャンcurScanの各スキャン点に対応する点をbaseLpsから見つける
double DataAssociatorLS::findCorrespondence(const Scan2D *curScan, const Pose2D &predPose) {
  double dthre = 0.2;  // これより遠い点は除外する[m]
  curLps.clear();      // 対応づけ現在スキャン点群を空にする
  refLps.clear();      // 対応づけ参照スキャン点群を空にする
  for (size_t i = 0; i < curScan->lps.size(); i++) {
    const LPoint2D *clp = &(curScan->lps[i]);  // 現在スキャンの点。ポインタで。

    // スキャン点lpをpredPoseで座標変換した位置に最も近い点を見つける
    LPoint2D glp;                     // clpの予測位置
    predPose.globalPoint(*clp, glp);  // predPoseで座標変換

    double dmin = HUGE_VAL;            // 距離最小値
    const LPoint2D *rlpmin = nullptr;  // 最も近い点
    for (size_t j = 0; j < baseLps.size(); j++) {
      const LPoint2D *rlp = baseLps[j];  // 参照スキャン点
      double d = (glp.x - rlp->x) * (glp.x - rlp->x) + (glp.y - rlp->y) * (glp.y - rlp->y);
      if (d <= dthre * dthre && d < dmin) {  // dthre内で距離が最小となる点を保存
        dmin = d;
        rlpmin = rlp;
      }
    }
    if (rlpmin != nullptr) {  // 最近傍点があれば登録
      curLps.push_back(clp);
      refLps.push_back(rlpmin);
    }
  }

  double ratio = (1.0 * curLps.size()) / curScan->lps.size();  // 対応がとれた点の比率

  return ratio;
}
