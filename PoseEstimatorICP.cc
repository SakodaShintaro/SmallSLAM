#include "PoseEstimatorICP.h"

#include <boost/timer.hpp>

using namespace std;

// 初期値initPoseを与えて、ICPによりロボット位置の推定値estPoseを求める
double PoseEstimatorICP::estimatePose(Pose2D& initPose, Pose2D& estPose) {
  boost::timer tim;

  double evmin = HUGE_VAL;   // コスト最小値。初期値は大きく
  double evthre = 0.000001;  // コスト変化閾値。変化量がこれ以下なら繰り返し終了

  popt->setEvthre(evthre);
  popt->setEvlimit(0.2);  // evlimitは外れ値の閾値[m]

  double ev = 0;         // コスト
  double evold = evmin;  // 1つ前の値。収束判定のために使う。
  Pose2D pose = initPose;
  Pose2D poseMin = initPose;
  for (int i = 0; abs(evold - ev) > evthre && i < 100; i++) {  // i < 100は振動対策
    if (i > 0) {
      evold = ev;
    }
    double mratio = dass->findCorrespondence(curScan, pose);  // データ対応づけ
    Pose2D newPose;
    popt->setPoints(dass->curLps, dass->refLps);  // 対応結果を渡す
    ev = popt->optimizePose(pose, newPose);       // その対応づけにおいてロボット位置の最適化
    pose = newPose;

    if (ev < evmin) {  // コスト最小結果を保存
      poseMin = newPose;
      evmin = ev;
    }
  }

  pnrate = popt->getPnrate();
  usedNum = dass->curLps.size();

  estPose = poseMin;

  printf("finalError=%g, pnrate=%g\n", evmin, pnrate);
  printf("estPose: tx=%g, ty=%g, th=%g\n", pose.tx, pose.ty, pose.th);  // 確認用

  double t1 = 1000 * tim.elapsed();
  printf("PoseEstimatorICP: t1=%g\n", t1);  // 処理時間

  if (evmin < HUGE_VAL) {
    totalError += evmin;
  }

  totalTime += t1;
  printf("totalError=%g, totalTime=%g\n", totalError, totalTime);

  return evmin;
}
