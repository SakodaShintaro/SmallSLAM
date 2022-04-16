#include "LoopDetectorSS.h"

using namespace std;

// ループ検出
// 現在位置curPoseに近く、現在スキャンcurScanに形が一致する場所をロボット軌跡から見つけてポーズアークを張る。
bool LoopDetectorSS::detectLoop(Scan2D* curScan, Pose2D& curPose, int cnt) {
  printf("--detectLoop--\n");

  // 最も近い部分地図を探す
  double atd = pcmap->atd;                         // 現在の実際の累積走行距離
  double atdR = 0;                                 // 下記の処理で軌跡をなぞる時の累積走行距離
  const vector<Submap>& submaps = pcmap->submaps;  // 部分地図
  const vector<Pose2D>& poses = pcmap->poses;      // ロボット軌跡
  double dmin = HUGE_VAL;                          // 前回訪問点までの距離の最小値
  size_t imin = 0, jmin = 0;                       // 距離最小の前回訪問点のインデックス
  Pose2D prevP;                                    // 直前のロボット位置
  for (size_t i = 0; i < submaps.size() - 1; i++) {
    const Submap& submap = submaps[i];  // i番目の部分地図
    for (size_t j = submap.cntS; j <= submap.cntE; j++) {
      Pose2D p = poses[j];
      atdR += sqrt(pow(p.tx - prevP.tx, 2) + pow(p.ty - prevP.ty, 2));
      if (atd - atdR < atdthre) {
        i = submaps.size();
        break;
      }

      prevP = p;

      double d = pow(curPose.tx - p.tx, 2) + pow(curPose.ty - p.ty, 2);
      if (d < dmin) {
        dmin = d;
        imin = i;
        jmin = j;
      }
    }
  }

  printf("dmin=%g, radius=%g, imin=%lu, jmin=%lu\n", sqrt(dmin), radius, imin, jmin);  // 確認用
  if (dmin > pow(radius, 2)) {
    return false;
  }

  Submap& refSubmap = pcmap->submaps[imin];
  const Pose2D& initPose = poses[jmin];
  printf("curPose:  tx=%g, ty=%g, th=%g\n", curPose.tx, curPose.ty, curPose.th);
  printf("initPose: tx=%g, ty=%g, th=%g\n", initPose.tx, initPose.ty, initPose.th);

  // 再訪点の位置を求める
  Pose2D revisitPose;
  bool flag = estimateRevisitPose(curScan, refSubmap.mps, curPose, revisitPose);

  if (flag) {
    Eigen::Matrix3d icpCov;                                              // ICPの共分散
    double ratio = pfu->calIcpCovariance(revisitPose, curScan, icpCov);  // ICPの共分散を計算

    LoopInfo info;
    info.pose = revisitPose;
    info.cov = icpCov;
    info.curId = cnt;
    info.refId = static_cast<int>(jmin);
    makeLoopArc(info);

    // 確認用
    Scan2D refScan;
    Pose2D spose = poses[refSubmap.cntS];
    refScan.setSid(info.refId);
    refScan.setLps(refSubmap.mps);
    refScan.setPose(spose);
    LoopMatch lm(*curScan, refScan, info);
    loopMatches.emplace_back(lm);
    printf("curId=%d, refId=%d\n", info.curId, info.refId);
  }

  return flag;
}

void LoopDetectorSS::makeLoopArc(LoopInfo& info) {
  if (info.arcked) {
    return;
  }

  info.setArcked(true);

  Pose2D srcPose = pcmap->poses[info.refId];                 // 前回訪問点の位置
  Pose2D dstPose(info.pose.tx, info.pose.ty, info.pose.th);  // 再訪点の位置
  Pose2D relPose;
  Pose2D::calRelativePose(dstPose, srcPose, relPose);  // ループアークの高速

  // アークの拘束は始点ノードからの相対位置なので、共分散をループアークの始点ノード座標系に変換
  Eigen::Matrix3d cov;
  CovarianceCalculator::rotateCovariance(srcPose, info.cov, cov, true);

  PoseArc* arc = pg->makeArc(info.refId, info.curId, relPose, cov);  // ループアーク生成
  pg->addArc(arc);

  // 確認用
  printf("makeLoopArc: pose arc added\n");
  printf("srcPose: tx=%g, ty=%g, th=%g\n", srcPose.tx, srcPose.ty, srcPose.th);
  printf("dstPose: tx=%g, ty=%g, th=%g\n", dstPose.tx, dstPose.ty, dstPose.th);
  printf("relPose: tx=%g, ty=%g, th=%g\n", relPose.tx, relPose.ty, relPose.th);
  PoseNode* src = pg->findNode(info.refId);
  PoseNode* dst = pg->findNode(info.curId);
  Pose2D relPose2;
  Pose2D::calRelativePose(dst->pose, src->pose, relPose2);
  printf("relPose2: tx=%g, ty=%g, th=%g\n", relPose2.tx, relPose2.ty, relPose2.th);
}

// 現在スキャンcurScanと部分地図の点群refLpsでICPを行い、再訪点の位置を求める
bool LoopDetectorSS::estimateRevisitPose(const Scan2D* curScan, const vector<LPoint2D>& refLps, const Pose2D& initPose,
                                         Pose2D& revisitPose) {
  dass->setRefBase(refLps);
  cfunc->setEvlimit(0.2);

  printf("initPose: tx=%g, ty=%g, th=%g\n", initPose.tx, initPose.ty, initPose.th);  // 確認用

  size_t usedNumMin = 50;

  // 初期位置initPoseの周囲をしらみつぶしに調べる
  // 効率化のため、ICPは行わず、各位置で単純にマッチングスコアを調べる
  double rangeT = 1;   // 並進の探索範囲[m]
  double rangeA = 45;  // 回転の探索範囲[度]
  double dd = 0.2;     // 並進の探索間隔[m]
  double da = 2;       // 回転の探索間隔[度]
  double pnrateMax = 0;
  vector<double> pnrates;
  double scoreMin = 1000;
  vector<double> scores;
  vector<Pose2D> candidates;  // スコアの良い候補位置
  for (double dy = -rangeT; dy <= rangeT; dy += dd) {
    double y = initPose.ty + dy;
    for (double dx = -rangeT; dx <= rangeT; dx += dd) {
      double x = initPose.tx + dx;
      for (double dth = -rangeA; dth <= rangeA; dth += da) {
        double th = MyUtil::add(initPose.th, dth);
        Pose2D pose(x, y, th);
        double mratio = dass->findCorrespondence(curScan, pose);
        size_t usedNum = dass->curLps.size();
        if (usedNum < usedNumMin || mratio < 0.9) {
          continue;
        }
        cfunc->setPoints(dass->curLps, dass->refLps);
        double score = cfunc->calValue(x, y, th);
        double pnrate = cfunc->getPnrate();

        if (pnrate > 0.8) {
          candidates.emplace_back(pose);
          if (score < scoreMin) {
            scoreMin = score;
          }
          scores.push_back(score);
        }
      }
    }
  }

  printf("candidates.size=%lu\n", candidates.size());  // 確認用
  if (candidates.size() == 0) {
    return false;
  }

  // 候補位置candidatesの中から最もよいものをICPで選ぶ
  Pose2D best;                          // 最良候補
  double smin = 1000000;                // ICPスコア最小値
  estim->setScanPair(curScan, refLps);  // ICPにスキャン設定
  for (size_t i = 0; i < candidates.size(); i++) {
    Pose2D p = candidates[i];         // 候補位置
    printf("score=%g\n", scores[i]);  // 確認用
    Pose2D estP;
    double score = estim->estimatePose(p, estP);                   // ICPでマッチング位置を求める
    double pnrate = estim->getPnrate();                            // ICPでの点の対応率
    size_t usedNum = estim->getUsedNum();                          // ICPで使用した点数
    if (score < smin && pnrate >= 0.9 && usedNum >= usedNumMin) {  // ループ検出は条件厳しく
      smin = score;
      best = estP;
      printf("smin=%g, pnrate=%g, usedNum=%lu\n", smin, pnrate, usedNum);  // 確認用
    }
  }

  // 最小スコアが閾値より小さければ見つけた
  if (smin <= scthre) {
    revisitPose = best;
    return true;
  }

  return false;
}
