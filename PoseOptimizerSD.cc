#include "PoseOptimizerSD.h"

using namespace std;

double PoseOptimizerSD::optimizePose(Pose2D& initPose, Pose2D& estPose) {
  double th = initPose.th;
  double tx = initPose.tx;
  double ty = initPose.ty;
  double txmin = tx, tymin = ty, thmin = th;  // コスト最小の解
  double evmin = HUGE_VAL;                    // コストの最小値
  double evold = evmin;                       // 1つ前のコスト値。収束判定に使う

  double ev = cfunc->calValue(tx, ty, th);  // コスト計算
  int nn = 0;                               // 繰り返し回数。確認用
  double kk = 0.00001;                      // 最急降下法のステップ幅係数
  while (abs(evold - ev) > evthre) {        // 収束判定。1つ前の値との変化が小さいと終了
    nn++;
    evold = ev;

    // 数値計算による偏微分
    double dEtx = (cfunc->calValue(tx + dd, ty, th) - ev) / dd;
    double dEty = (cfunc->calValue(tx, ty + dd, th) - ev) / dd;
    double dEth = (cfunc->calValue(tx, ty, th + da) - ev) / dd;

    // 微分係数にkkをかけてステップ幅にする
    double dx = -kk * dEtx;
    double dy = -kk * dEty;
    double dth = -kk * dEth;
    tx += dx;
    ty += dy;
    th += dth;

    ev = cfunc->calValue(tx, ty, th);  // その位置でコスト計算

    if (ev < evmin) {
      evmin = ev;
      txmin = tx;
      tymin = ty;
      thmin = th;
    }
  }

  ++allN;
  if (allN > 0 && evmin < 100) {
    sum += evmin;
  }

  estPose.setVal(txmin, tymin, thmin);

  return evmin;
}
