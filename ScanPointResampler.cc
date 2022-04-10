#include "ScanPointResampler.h"

using namespace std;

void ScanPointResampler::resamplePoints(Scan2D *scan) {
  vector<LPoint2D> &lps = scan->lps;
  if (lps.size() == 0) {
    return;
  }

  vector<LPoint2D> newLps;

  dis = 0;
  LPoint2D lp = lps[0];
  LPoint2D prevLp = lp;
  LPoint2D np(lp.sid, lp.x, lp.y);
  newLps.emplace_back(np);

  for (size_t i = 1; i < lps.size(); i++) {
    lp = lps[i];
    bool inserted = false;
    bool exist = findInterpolatePoint(lp, prevLp, np, inserted);
    if (exist) {
      newLps.emplace_back(np);
      prevLp = np;
      dis = 0;
      if (inserted) {
        i--;
      }
    } else {
      prevLp = lp;
    }
  }

  scan->setLps(newLps);

  printf("lps.size=%lu, newLps.size=%lu\n", lps.size(), newLps.size());  // 確認用
}

bool ScanPointResampler::findInterpolatePoint(const LPoint2D &cp, const LPoint2D &pp, LPoint2D &np, bool &inserted) {
  double dx = cp.x - pp.x;
  double dy = cp.y - pp.y;
  double L = sqrt(dx * dx + dy * dy);  // 現在点cpと直前点ppの距離
  if (dis + L < dthreS) {              // 予測累積距離(dis+L)がdthreSより小さい点は削除
    dis += L;                          // disに加算
    return false;
  } else if (dis + L >= dthreL) {  // 予測累積距離がdthreLより大きい点は補間せず、そのまま残す
    np.setData(cp.sid, cp.x, cp.y);
  } else {  // 予測累積距離がdthreSを超えたら、dthreSになるように補間する
    double ratio = (dthreS - dis) / L;
    double x2 = dx * ratio + pp.x;  // 少し伸ばして距離がdthreSになる位置
    double y2 = dy * ratio + pp.y;
    np.setData(cp.sid, x2, y2);
    inserted = true;  // cpより前にnpを入れたというフラグ
  }

  return true;
}
