#include "PoseGraph.h"

using namespace std;

// ポーズグラフにノード追加
PoseNode *PoseGraph::addNode(const Pose2D &pose) {
  PoseNode *n1 = allocNode();
  addNode(n1, pose);
  return n1;
}

// ポーズグラフにノード追加
void PoseGraph::addNode(PoseNode *n1, const Pose2D &pose) {
  n1->setNid((int)nodes.size());  // ノードID付与。ノードの通し番号と同じ
  n1->setPose(pose);              // ロボット位置を付与
  nodes.push_back(n1);            // nodesの最後に追加
}

// ノードID(nid)からノード実体を見つける
PoseNode *PoseGraph::findNode(int nid) {
  for (size_t i = 0; i < nodes.size(); i++) {
    PoseNode *n = nodes[i];
    if (n->nid == nid) {
      return n;
    }
  }

  return nullptr;
}

// ポーズグラフにアークを追加する
void PoseGraph::addArc(PoseArc *arc) {
  arc->src->addArc(arc);  // 始点ノードにarcを追加
  arc->dst->addArc(arc);  // 終点ノードにarcを追加
  arcs.push_back(arc);
}

// 始点ノードsrcNidと終点ノードdstNidの間にアークを生成する
PoseArc *PoseGraph::makeArc(int srcNid, int dstNid, const Pose2D &relPose, const Eigen::Matrix3d &cov) {
  Eigen::Matrix3d inf = MyUtil::svdInverse(cov);  // infはcovの逆行列
  PoseNode *src = nodes[srcNid];                  // 始点ノード
  PoseNode *dst = nodes[dstNid];                  // 終点ノード

  PoseArc *arc = allocArc();
  arc->setup(src, dst, relPose, inf);
  return arc;
}

// 始点ノードがsrcNid、終点ノードがdstNidであるアークを返す
PoseArc *PoseGraph::findArc(int srcNid, int dstNid) {
  for (size_t i = 0; i < arcs.size(); i++) {
    PoseArc *a = arcs[i];
    if (a->src->nid == srcNid && a->dst->nid == dstNid) {
      return a;
    }
  }
  return nullptr;
}

// 確認用
void PoseGraph::printNodes() {
  printf("--- printNodes ---\n");
  printf("nodes.size=%lu\n", nodes.size());
  for (size_t i = 0; i < nodes.size(); i++) {
    PoseNode *node = nodes[i];
    printf("i=%lu: nid=%d, tx=%g, ty=%g, th=%g\n", i, node->nid, node->pose.tx, node->pose.ty, node->pose.th);

    for (size_t j = 0; j < node->arcs.size(); j++) {
      PoseArc *a = node->arcs[j];
      printf("arc j=%lu: srcId=%d, dstId=%d, src=%p, dst=%p\n", j, a->src->nid, a->dst->nid, a->src, a->dst);
    }
  }
}

// 確認用
void PoseGraph::printArcs() {
  printf("--- printArcs ---\n");
  printf("arcs.size=%lu\n", arcs.size());
  for (size_t j = 0; j < arcs.size(); j++) {
    PoseArc *a = arcs[j];
    double dis = (a->src->pose.tx - a->dst->pose.tx) * (a->src->pose.tx - a->dst->pose.tx) +
                 (a->src->pose.ty - a->dst->pose.ty) * (a->src->pose.ty - a->dst->pose.ty);

    Pose2D &rpose = a->relPose;
    printf("j=%lu, srcId=%d, dstId=%d, tx=%g, ty=%g, th=%g\n", j, a->src->nid, a->dst->nid, rpose.tx, rpose.ty,
           rpose.th);
  }
}
