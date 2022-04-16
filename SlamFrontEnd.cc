#include "SlamFrontEnd.h"

using namespace std;

// 初期化
void SlamFrontEnd::init() {
  smat->reset();
  smat->setPointCloudMap(pcmap);
  sback.setPointCloudMap(pcmap);
}

// 現在スキャンscanを処理する
void SlamFrontEnd::process(Scan2D &scan) {
  if (cnt == 0) {
    init();
  }

  // スキャンマッチング
  smat->matchScan(scan);

  Pose2D curPose = pcmap->getLastPose();  // これはスキャンマッチングで推定した現在のロボット位置

  // ポーズグラフにオドメトリアークを追加
  if (cnt == 0) {
    pg->addNode(curPose);  // 最初はノードを置くだけ
  } else {
    Eigen::Matrix3d &cov = smat->getCovariance();
    makeOdometryArc(curPose, cov);
  }

  if (cnt % keyframeSkip == 0) {
    // キーフレームのときだけ
    if (cnt == 0) {
      // cnt = 0のときは地図が小さいのでサンプリングを多くする
      pcmap->setNthre(1);
    } else {
      pcmap->setNthre(5);
    }

    // 点群地図の全体地図を生成
    pcmap->makeGlobalMap();
  }

  if (cnt > keyframeSkip && cnt % keyframeSkip == 0) {
    bool flag = lpd->detectLoop(&scan, curPose, cnt);
    if (flag) {
      sback.adjustPoses();  // ループが見つかったらポーズ調整
      sback.remakeMaps();   // 地図やポーズグラフの修正
    }
  }

  printf("pcmap.size=%lu\n", pcmap->globalMap.size());  // 確認用

  countLoopArcs();  // 確認用

  ++cnt;
}

// オドメトリアークの生成
bool SlamFrontEnd::makeOdometryArc(Pose2D &curPose, const Eigen::Matrix3d &fusedCov) {
  if (pg->nodes.size() == 0) {
    return false;
  }

  PoseNode *lastNode = pg->nodes.back();
  PoseNode *curNode = pg->addNode(curPose);

  // 直前ノードと現在ノードの間にオドメトリアークを張る
  Pose2D &lastPose = lastNode->pose;
  Pose2D relPose;
  Pose2D::calRelativePose(curPose, lastPose, relPose);
  printf("sfront: lastPose:  tx=%g, ty=%g, th=%g\n", lastPose.tx, lastPose.ty, lastPose.th);

  Eigen::Matrix3d cov;
  CovarianceCalculator::rotateCovariance(lastPose, fusedCov, cov, true);  // 移動量の共分散に変換
  PoseArc *arc = pg->makeArc(lastNode->nid, curNode->nid, relPose, cov);  // アークの生成
  pg->addArc(arc);                                                        // ポーズグラフにアークを追加

  return true;
}

// ループアーク数を数える。確認用
void SlamFrontEnd::countLoopArcs() {
  vector<PoseArc *> &parcs = pg->arcs;
  int an = 0;
  for (size_t i = 0; i < parcs.size(); i++) {
    PoseArc *a = parcs[i];
    PoseNode *src = a->src;
    PoseNode *dst = a->dst;
    if (src->nid != dst->nid - 1) {
      ++an;
    }
  }

  printf("loopArcs.size = %d\n", an);
}