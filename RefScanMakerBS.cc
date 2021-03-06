#include "RefScanMakerBS.h"

using namespace std;

const Scan2D *RefScanMakerBS::makeRefScan() {
  vector<LPoint2D> &refLps = refScan.lps;  // 参照スキャンの点群のコンテナ
  refLps.clear();

  Pose2D lastPose = pcmap->getLastPose();  // 点群地図に保存した最後の推定位置
  double(*R)[2] = lastPose.Rmat;
  double tx = lastPose.tx;
  double ty = lastPose.ty;

  // 点群地図に保存した最後のスキャンを参照スキャンにする
  const vector<LPoint2D> &lps = pcmap->lastScan.lps;
  for (size_t i = 0; i < lps.size(); i++) {
    const LPoint2D &mp = lps[i];  // 参照スキャンの点

    // スキャンはロボット座標系なので、地図座標系に変換
    LPoint2D rp;
    rp.x = R[0][0] * mp.x + R[0][1] * mp.y + tx;  // 点の位置
    rp.y = R[1][0] * mp.x + R[1][1] * mp.y + ty;
    rp.nx = R[0][0] * mp.nx + R[0][1] * mp.ny;  // 法線ベクトル
    rp.ny = R[1][0] * mp.nx + R[1][1] * mp.ny;
    refLps.emplace_back(rp);
  }

  return &refScan;
}