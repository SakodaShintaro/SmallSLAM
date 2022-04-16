#include "DataAssociatorGT.h"

#include <boost/timer.hpp>

using namespace std;

// 現在スキャンcurScanの各スキャン点をpredPoseで座標変換した位置に最も近い点を見つける
double DataAssociatorGT::findCorrespondence(const Scan2D *curScan, const Pose2D &predPose) {
  boost::timer tim;  // 処理時間測定用

  curLps.clear();  // 対応づけ現在スキャン点群を空にする
  refLps.clear();  // 対応づけ参照スキャン点群を空にする

  for (size_t i = 0; i < curScan->lps.size(); i++) {
    const LPoint2D *clp = &(curScan->lps[i]);  // 現在スキャンの点。ポインタで。

    // 格子テーブルにより最近傍点を求める。格子テーブル内に距離閾値dthreがあることに注意。
    const LPoint2D *rlp = nntab.findClosestPoint(clp, predPose);

    if (rlp != nullptr) {
      curLps.push_back(clp);  // 最近傍点があれば登録
      refLps.push_back(rlp);
    }
  }

  double ratio = (1.0 * curLps.size()) / curScan->lps.size();  // 対応がとれた点の比率

  //  double t1 = 1000*tim.elapsed();                   // 処理時間
  //  printf("Elapsed time: dassGT=%g\n", t1);

  return (ratio);
}
