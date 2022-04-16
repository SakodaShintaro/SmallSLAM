#ifndef P2O_DRIVER2D_H_
#define P2O_DRIVER2D_H_

#include <fstream>
#include <iostream>
#include <vector>

#include "MyUtil.h"
#include "Pose2D.h"
#include "PoseGraph.h"
#include "Scan2D.h"

// ポーズグラフ最適化ライブラリを起動する。
class P2oDriver2D {
 public:
  void doP2o(PoseGraph &graph, std::vector<Pose2D> &newPoses, int N);
};

#endif
