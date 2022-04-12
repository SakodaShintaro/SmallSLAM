#ifndef POSE_GRAPH_H_
#define POSE_GRAPH_H_

#include <vector>

#include "MyUtil.h"
#include "Pose2D.h"

struct PoseArc;

// ポーズグラフの頂点
struct PoseNode {
  int nid;                      // ノードID。PoseGraphのnodesのインデックス（通し番号）
  Pose2D pose;                  // このノードのロボット位置
  std::vector<PoseArc *> arcs;  // このノードに繋がるアーク

  PoseNode() : nid(-1) {}

  PoseNode(const Pose2D &p) : nid(-1) { pose = p; }

  void init() {
    nid = -1;
    arcs.clear();
  }

  void setPose(const Pose2D &p) { pose = p; }

  void setNid(int n) { nid = n; }

  void addArc(PoseArc *a) { arcs.push_back(a); }
};

// ポーズグラフの辺
struct PoseArc {
  PoseNode *src;        // このアークの始点側のノード
  PoseNode *dst;        // このアークの終点側のノード
  Pose2D relPose;       // このアークのもつ相対位置（計測値）
  Eigen::Matrix3d inf;  // 情報行列

  PoseArc() : src(nullptr), dst(nullptr) {}
  PoseArc(PoseNode *s, PoseNode *d, Pose2D &rel, const Eigen::Matrix3d _inf) { setup(s, d, rel, _inf); }

  void setup(PoseNode *s, PoseNode *d, const Pose2D &rel, const Eigen::Matrix3d _inf) {
    src = s;
    dst = d;
    relPose = rel;
    inf = _inf;
  }
};

// ポーズグラフ
class PoseGraph {
 public:
  std::vector<PoseNode *> nodes;  // ノードの集合
  std::vector<PoseArc *> arcs;    // アークの集合、アークは片方向のみもつ

  PoseGraph() {
    nodePool.reserve(POOL_SIZE);  // メモリプールの領域を最初に確保
    arcPool.reserve(POOL_SIZE);
  }

  void reset() {
    nodes.clear();
    arcs.clear();
    nodePool.clear();
    arcPool.clear();
  }

  PoseNode *allocNode() {
    if (nodePool.size() >= POOL_SIZE) {
      printf("Error: exceeds nodePool capacity\n");
      return nullptr;
    }

    PoseNode node;
    nodePool.emplace_back(node);
    return &nodePool.back();
  }

  PoseArc *allocArc() {
    if (arcPool.size() >= POOL_SIZE) {
      printf("Error: exceeds arcPool capacity\n");
      return nullptr;
    }

    PoseArc arc;
    arcPool.emplace_back(arc);
    return &arcPool.back();
  }

  PoseNode *addNode(const Pose2D &pose);
  void addNode(PoseNode *n1, const Pose2D &pose);
  PoseNode *findNode(int nid);

  void addArc(PoseArc *arc);
  PoseArc *makeArc(int srcNid, int dstNid, const Pose2D &relPose, const Eigen::Matrix3d &cov);
  PoseArc *findArc(int srcNid, int dstNid);

  void printNodes();
  void printArcs();

 private:
  static const int POOL_SIZE = 100000;
  std::vector<PoseNode> nodePool;  // ノード生成用のメモリプール
  std::vector<PoseArc> arcPool;    // アーク生成用のメモリプール。アークは片方向のみもつ
};

#endif
