#ifndef _NN_GRID_TABLE_H_
#define _NN_GRID_TABLE_H_

#include <vector>

#include "MyUtil.h"
#include "Pose2D.h"

struct NNGridCell {
  std::vector<const LPoint2D *> lps;  // このセルに格納されたスキャン点群

  void clear() {
    lps.clear();  // 空にする
  }
};

// 格子テーブル
class NNGridTable {
 private:
  double csize;                   // セルサイズ[m]
  double rsize;                   // 対象領域のサイズ[m]。正方形の1辺の半分。
  int tsize;                      // テーブルサイズの半分
  std::vector<NNGridCell> table;  // テーブル本体

 public:
  NNGridTable() : csize(0.05), rsize(40) {       // セル5cm、対象領域40x2m四方
    tsize = static_cast<int>(rsize / csize);     // テーブルサイズの半分
    size_t w = static_cast<int>(2 * tsize + 1);  // テーブルサイズ
    table.resize(w * w);                         // 領域確保
    clear();                                     // tableの初期化
  }

  void clear() {
    // 各セルを空にする
    for (size_t i = 0; i < table.size(); i++) {
      table[i].clear();
    }
  }

  void addPoint(const LPoint2D *lp);
  const LPoint2D *findClosestPoint(const LPoint2D *clp, const Pose2D &predPose);
  void makeCellPoints(int nthre, std::vector<LPoint2D> &ps);
};

#endif
