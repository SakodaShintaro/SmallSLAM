#include "SlamBackEnd.h"

#include "P2oDriver2D.h"

using namespace std;

Pose2D SlamBackEnd::adjustPoses() {
  newPoses.clear();
  P2oDriver2D p2o;
  p2o.doP2o(*pg, newPoses, 5);
  return newPoses.back();
}

void SlamBackEnd::remakeMaps() {
  // PoseGraphの修正
  vector<PoseNode*> & pnodes = pg->nodes;
  for (size_t i = 0; i < newPoses.size(); i++) {
    Pose2D &npose = newPoses[i];
    PoseNode *pnode = pnodes[i];
    pnode->setPose(npose);
  }

  printf("newPoses.size=%lu, nodes.size=%lu\n", newPoses.size(), pnodes.size());

  // PointCloudMapの修正
  pcmap->remakeMaps(newPoses);
}
